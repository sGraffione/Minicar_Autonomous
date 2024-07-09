#include "rclcpp/rclcpp.hpp"
#include "minicar_interfaces/msg/bts_data.hpp"

#include <regex>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cstring>
#include <iostream>
#include <fstream>
#include <numeric>
#include <chrono>
#include <cstdlib>

#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>

#define POSITION_STRING_SIZE 18
#define POSITION_STRING_OFFSET 3
#define BUFFER_SIZE 256

using namespace std::chrono_literals;

class LocalizationTag
{
	public:
		LocalizationTag(const char *addr)
		{
			RCLCPP_DEBUG(rclcpp::get_logger("localization_LocalizationTag"), "Opening device");
			serial_port = open(addr, O_RDWR);

			//check for errors
			if(serial_port<0){
				RCLCPP_ERROR(rclcpp::get_logger("localization_LocalizationTag"), "Error %i from open: %s\n", errno, strerror(errno));
			}

			if(tcgetattr(serial_port, &tty) != 0){
				RCLCPP_ERROR(rclcpp::get_logger("localization_LocalizationTag"), "Error %i from tcgetattr: %s\n",errno,strerror(errno));
			}
			// Control modes
			tty.c_cflag &= ~PARENB; // clear parity bit, disable it, most common
			tty.c_cflag &= ~CSTOPB; // cler stop filed, only one stop bit used in communication (most common)
			tty.c_cflag &= ~CSIZE; //Clear all the size bits, then set it
			tty.c_cflag |= CS8; // 8 bits per byte
			tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common). Flow control checks for a ready signal from hardware when some data is avaiable.
			tty.c_cflag |= CREAD | CLOCAL; // Turn on READ and ignore ctrl lines (CLOCAL = 1)
			// Local modes
			tty.c_lflag &= ~ICANON; // non-canonical input mode. It will not wait for a new line character.
			tty.c_lflag &= ~ECHO;   // Disable echo
			tty.c_lflag &= ~ECHOE;	// Disable erasure
			tty.c_lflag &= ~ECHONL; // Disable new-line echo
			tty.c_lflag &= ~ISIG;	// Disable interpretation of INTR, QUIT and SUSP
			//  Input modes
			tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off software control flow
			tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // Disable any special handling of received bytes
			// Output modes
			tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes
			tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
			// Timeout modes
			tty.c_cc[VTIME] = 5;
			tty.c_cc[VMIN] = 0;
			// Baudrate
			cfsetispeed(&tty, B115200); // input baudrate
			cfsetispeed(&tty, B115200); // output baudrate
			
			// save tty settings
			if (tcsetattr(serial_port, TCSANOW, &tty) != 0){
				RCLCPP_ERROR(rclcpp::get_logger("localization_LocalizationTag"), "Error %i from tcsetattr: %s\n",errno,strerror(errno));
			}

			wakeUpDevice();
		}

		~LocalizationTag(){};

		void wakeUpDevice()
		{
			// Wake up Tag
			unsigned char msg[] = {'\r'};
			unsigned char wu_msg[] = {'\r','\r'};
			unsigned char mode_msg[] = {'l','e','s','\r'}; // Position plus quality factor

			RCLCPP_DEBUG(rclcpp::get_logger("localization_LocalizationTag"), "Waking up device");
			write(serial_port,msg,sizeof(msg));
			sleep(2);
			write(serial_port,wu_msg,sizeof(wu_msg));
			sleep(2);
			write(serial_port,mode_msg,sizeof(mode_msg));
			sleep(1);
		}

		void closeDevice()
		{
			close(serial_port);
		}

		void readMessage(minicar_interfaces::msg::BtsData *message)
		{
			int recBytes = read(serial_port, &read_buf, sizeof(read_buf));
			read_buf[recBytes] = '\0'; //end string terminator at the end of read bytes. It permits to have a clean and constant reading of the incoming datas
			if (recBytes < 0){
				RCLCPP_ERROR(rclcpp::get_logger("localization_LocalizationTag"), "Error reading: %s\n",strerror(errno));
			}
			//printf("Read %i bytes\n",recBytes);
			RCLCPP_DEBUG(rclcpp::get_logger("localization_LocalizationTag"), "%s\n",read_buf);	
			for (size_t i = 0; i < BUFFER_SIZE-2; i++){
				if((read_buf[i] == 'e')&(read_buf[i+1] == 's')&(read_buf[i+2] == 't')){						
					// search for the brackets
					brack1_pos = i+3; // I know that after 'est' comes an open bracket '['
					for (size_t j = i+3; j < BUFFER_SIZE; j++){
						if(read_buf[j] == ']'){
							brack2_pos = j;
							break;
						}
					}
					// get what's in between
					memcpy(part,read_buf+brack1_pos+1,brack2_pos-brack1_pos-1);
					part[brack2_pos-brack1_pos-1] ='\0';
					//printf("%s\n",part);
					
					//#############OK#############//
					
					int iCurPos = 0;
					token = strtok(part,",");
					//printf("iCurPos: %i | token: %s\n", iCurPos, token);
					while(token!=NULL){
						
						if (iCurPos == 3){
							message->quality = (int8_t)atoi(token);
						}else{
							message->position[iCurPos] = (float_t)atof(token);
						}
						iCurPos++;
						
						token = strtok(NULL,",");
						//printf("iCurPos: %i | token: %s\n", iCurPos, token);
					}

					//printf("%f %f %f with quality: %i\n",message->position[0],message->position[1],message->position[2],message->quality);
					break;
				}
			}
		}

	private:
		int serial_port;
		struct termios tty;

		// Prepare read buffer
		char read_buf[BUFFER_SIZE];
		size_t counter = -1;

		char part[BUFFER_SIZE];
		char *token;
		int brack1_pos = 0;
		int brack2_pos = 0;
};

class LocalizationPublisher : public rclcpp::Node
{
	public:
		LocalizationPublisher() : Node("localization_publiher"), count_(0), locTag_("/dev/ttyACM0") // to change according to the assigned serial
		{
			publisher_ = this->create_publisher<minicar_interfaces::msg::BtsData>("localization",10); // implicitly KeepLast
			timer_ = this->create_wall_timer(
				100ms, std::bind(&LocalizationPublisher::data_callback, this)
			);
		}

	private:
		void data_callback()
		{
			locTag_.readMessage(&message_);
			RCLCPP_DEBUG(this->get_logger(), "%f | %f | %f", message_.position[0], message_.position[1], message_.position[2]);
			publisher_->publish(message_);
		}

		rclcpp::TimerBase::SharedPtr timer_;
		rclcpp::Publisher<minicar_interfaces::msg::BtsData>::SharedPtr publisher_;
		size_t count_;
		minicar_interfaces::msg::BtsData message_;
		LocalizationTag locTag_;
};

int main(int argc, char **argv){
	rclcpp::init(argc, argv);
	
	rclcpp::spin(std::make_shared<LocalizationPublisher>());
	rclcpp::shutdown();

	return 0;
}
