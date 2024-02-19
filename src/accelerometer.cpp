#include "rclcpp/rclcpp.hpp"
#include "minicar_interfaces/msg/accelerometer.hpp"

#include <MPU6050.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <numeric>
#include <cstdlib>
#include <chrono>

#define NODE_NAME "accelerometer"

using namespace std::chrono_literals;


double Ts = 0.1;

class AcceleromenterPublisher : public rclcpp::Node
{
	public:
		AcceleromenterPublisher() : Node("accelerometer_publisher"), count_(0), device(0x68)
		{
			publisher_ = this->create_publisher<minicar_interfaces::msg::Accelerometer>("accelerometer", 10);  // implicitly KeepLast
			calibrate_gyro();
			timer_ = this->create_wall_timer(
				100ms, std::bind(&AcceleromenterPublisher::data_callback, this)
			);
		}

		void writeMessage()
		{
			device.getGyro(&gr, &gp, &gy);
			device.getAccel(&ax, &ay, &az);
			message_.omega = gy;//(round((gy-offset)*10000 / GYRO_SENS)/10000)*M_PI/180;
			message_.x_accel = ax;//(round((ax-offsetAx)*10000 / ACCEL_SENS)/10000)*9.81;
			message_.y_accel = ay;//(round((ay-offsetAy)*10000 / ACCEL_SENS)/10000)*9.81;
		}
	
	private:
		void data_callback()
		{
			this->writeMessage();
			RCLCPP_DEBUG(this->get_logger(), "%f | %f | %f", message_.omega, message_.x_accel, message_.y_accel);
			publisher_->publish(message_);
		}

		void calibrate_gyro()
		{
			RCLCPP_DEBUG(this->get_logger(), "Calculating gyroscope offset...");
			/*Gyroscope drift calibration*/
			device.setOffsets();
			// std::vector<double> gyVect, axVect, ayVect;
			// auto start = std::chrono::high_resolution_clock::now();
			// auto time = std::chrono::duration_cast<std::chrono::seconds>(start-start);
			// while(time.count() <= 5){ // 5 seconds of data
			// 	device.getGyroRaw(&gr, &gp, &gy);
			// 	device.getAccelRaw(&ax,&ay,&az);
			// 	gyVect.push_back(gy);
			// 	axVect.push_back(ax);
			// 	ayVect.push_back(ay);

			// 	auto stop = std::chrono::high_resolution_clock::now();
			// 	time = std::chrono::duration_cast<std::chrono::seconds>(stop - start);
			// }
			
			// offset = (std::accumulate(gyVect.begin(),gyVect.end(),0.0)/gyVect.size())*M_PI/180;
			// offsetAx = (std::accumulate(axVect.begin(),axVect.end(),0.0)/axVect.size())*M_PI/180;
			// offsetAy = (std::accumulate(ayVect.begin(),ayVect.end(),0.0)/ayVect.size())*M_PI/180;
			RCLCPP_DEBUG(this->get_logger(), "Done!\n Running...");
		}

		rclcpp::TimerBase::SharedPtr timer_;
		rclcpp::Publisher<minicar_interfaces::msg::Accelerometer>::SharedPtr publisher_;
		size_t count_;
		minicar_interfaces::msg::Accelerometer message_;
		float offset;
		float offsetAx;
		float offsetAy;
		MPU6050 device;
		float ax, ay, az, gr, gp, gy, omega; //Variables to store the accel, gyro and angle values
};

int main(int argc, char **argv){
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<AcceleromenterPublisher>());
	rclcpp::shutdown();

	return 0;
}
