#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"
#include "minicar_interfaces/msg/motors.hpp"
#include "minicar_interfaces/msg/bts_data.hpp"
#include "minicar_interfaces/msg/accelerometer.hpp"
#include "minicar_interfaces/msg/ek_fstate.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/twist.h"

#include <cstdio>
#include <cstdint>
#include <unistd.h>
#include <stdio.h>
#include <sstream>
#include <cmath>
#include <vector>
#include <chrono>

#include <eigen3/Eigen/Dense>
#include <eigen3/unsupported/Eigen/KroneckerProduct>

#define MAX_SPEED 0.3
#define MAX_SPEED_RATE 0.05 // tuned by hand
#define MAX_DUTY_CYCLE 4000
#define MAX_DELTA 0.523599
#define MAX_DELTA_RATE 0.1
#define GAMMA_STEER 30/30
//LQT horizon
#define T 5
#define T_LESS_1 4
#define Np 15
#define Nc 5

int leftMotor = 23;
int dirLeftMotor = 24;
float Ts = 0.1;
float position[3];
float roll = 0, pitch = 0, yaw = 0;
//double MAX_DELTA_RATE = (M_PI/3*Ts)/0.17; // computation based on datasheet of MG996R servo motor

double L = 0.14;

using namespace std::chrono_literals;
using std::placeholders::_1;

struct Motors
{
	float velocity;
	float steering;
};

class Controller{
	public:
		Controller(){
			// Use it to initialize your variables
		}

		Motors getControlAsStruct(){
			Motors ctrlMotors;
			controlMethod1();
			ctrlMotors.velocity = Vel_opt;
			ctrlMotors.steering = delta_opt;
			return ctrlMotors;
		}

	private:
		void controlMethod1(){
			Vel_opt = 0.3;
			delta_opt = 0.1;
		}

		void controlMethod2(){

		}

		float Vel_opt = 0;
		float delta_opt = 0;
};


class ControllerPublisher : public rclcpp::Node, public Controller{
	
	public:
		ControllerPublisher() : Node("Controller"), count_(0){

			declare_parameter("use_kalmanfilter", use_kalmanfilter_);
			get_parameter_or("use_kalmanfilter", use_kalmanfilter_, use_kalmanfilter_);

			if(use_kalmanfilter_){
				ekf_sub_ = this->create_subscription<minicar_interfaces::msg::EKFstate>(
					"/EKFstate", 1, std::bind(&ControllerPublisher::ekf_callback, this, _1)
					);
			}

			ctrl_pub_ = this->create_publisher<minicar_interfaces::msg::Motors>("motorsCtrl", 1);
			timer_ = this->create_wall_timer(
				100ms, std::bind(&ControllerPublisher::publishControl, this)
			);
		}
	
	private:
		void publishControl(){
			Motors ctrlMotors = getControlAsStruct();
			RCLCPP_DEBUG(this->get_logger(), "throttle: %f | steering: %f", ctrlMotors.velocity, ctrlMotors.steering);
			ctrl_message_.throttle = ctrlMotors.velocity;
			ctrl_message_.steering = ctrlMotors.steering;
			ctrl_pub_->publish(ctrl_message_);
		}

		void ekf_callback(const minicar_interfaces::msg::EKFstate ekf_msg){
			position[0] = ekf_msg.state[0];
			position[1] = ekf_msg.state[1];
			yaw = ekf_msg.state[2];
			RCLCPP_DEBUG(this->get_logger(), "(%f %f) - %f", position[0], position[1], yaw);
		}

		rclcpp::Publisher<minicar_interfaces::msg::Motors>::SharedPtr ctrl_pub_;
		rclcpp::Subscription<minicar_interfaces::msg::EKFstate>::SharedPtr ekf_sub_;

		minicar_interfaces::msg::Motors ctrl_message_;
		minicar_interfaces::msg::EKFstate ekf_message_;

		size_t count_;
		rclcpp::TimerBase::SharedPtr timer_;

		bool use_kalmanfilter_ = false;

		Controller ctrl_;
};

int main(int argc, char **argv){
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ControllerPublisher>());
	rclcpp::shutdown();
	
	return 0;
}
