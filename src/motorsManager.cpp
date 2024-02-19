#include <memory>
#include <unistd.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include "rclcpp/rclcpp.hpp"
#include "minicar_interfaces/msg/motors.hpp"
#include <pigpio.h>
#include "bcm2835.h"

using std::placeholders::_1;

#define I2C_ADDRESS 0x40
#define PCA9685_MODE1 0x00
#define PCA9685_MODE2 0x01
#define PCA9685_LED0_ON_L 0x06
#define PCA9685_PRESCALE 0xFE

#define THROTTLE_PWM_CHANNEL_LEFT 0
#define THROTTLE_PWM_CHANNEL_RIGHT 1
#define STEERING_PWM_CHANNEL 15

#define MAX_DELTA_MOTOR 39*M_PI/180 //degree* pi/180 = radians
#define MAX_SPEED 0.3

#define THROTTLE 0
#define STEERING 1

// BCM PinOut | WiringPi PinOut
#define IN1 22 // RIGHT FORWARD - board pin 13
#define IN2 27 // RIGHT BACKWARD - board pin 15
#define IN3 23 // LEFT FORWARD - board pin 18
#define IN4 24  // LEFT BACKWARD - board pin 16

#define ENA 0  // LEFT
#define ENB 1  // RIGHT

#define NODE_LABEL "MotorsManager"


/**
 * @brief This class can be used to map a float 
 * value between two custom values to int value for the 
 * servos based on the PWM frequency.
 */
class Float2ServoMap
{
public:
  using Ptr = std::shared_ptr<Float2ServoMap>;

  /**
   * @brief builder function
   * 
   * @param[in] freq        Desired frequency of the PWM signal
   * @param[in] minMiliSec  Minimum high time of the PWM singal in milliseconds
   * @param[in] maxMiliSec  Maximum hight time of the PWM singnal in milliseconds
   * @param[in] maxPWM      The maximum value for the PWM (defining the resolution for PWM signal generator)
   * @return Float2ServoMap::Ptr 
   */
  static Float2ServoMap::Ptr create(float freq, unsigned int maxPWM, int type)
  {
    return std::make_shared<Float2ServoMap>(freq, maxPWM, type);
  }

  float calPwm(float f)
  {
    return m_ * f + b_;
  }

  Float2ServoMap(float freq, unsigned int maxPWM, int type): // type == 0 -> throttle, type == 1 -> steering
    m_(0), b_(0)
  {
    float minPWMv = 0, maxPWMv = 0, lBound_ = 0, uBound_ = 0;
    if (type == THROTTLE){
      minPWMv = 0;
      maxPWMv = maxPWM;
      lBound_ = 0;
      uBound_ = MAX_SPEED;
    }else if(type == STEERING){
      float minMiliSec = 1.2;
      float maxMiliSec = 1.8;
      float ms_2_percent = float(freq) / 1000.0 /* ms */;
      maxPWMv = minMiliSec * ms_2_percent * maxPWM;
      minPWMv = maxMiliSec * ms_2_percent * maxPWM;
      lBound_ = -MAX_DELTA_MOTOR;
      uBound_ = MAX_DELTA_MOTOR;
    }else{
      RCLCPP_ERROR(rclcpp::get_logger("pca9685"),"Wrong type for ServoMap generation: must be [steering: %i or throttle: %i].", STEERING, THROTTLE);
    }
    m_ = (maxPWMv - minPWMv) / (uBound_ - lBound_);
    b_ = minPWMv - m_ * lBound_;
  }
private:
  float m_, b_;
  float uBound_, lBound_;
};

class PCA9685 : public rclcpp::Node
{
public:
  PCA9685() : Node("pca9685")
  {
    bcmRes = bcm2835_init();
    if(gpioInitialise()<0){
      RCUTILS_LOG_ERROR_NAMED(get_name(), "PiGPIO initialisation failed.\n\r");
	  }
    init_hardware();
    control_subscription_ = this->create_subscription<minicar_interfaces::msg::Motors>(
        "/motorsCtrl", 10, std::bind(&PCA9685::control_callback, this, _1));
  }

  ~PCA9685()
  {
    // Close the I2C device
    set_pwm(0, 0, 0);
    set_pwm(1, 0, 0);
    close(i2c_fd_);
  }

private:
  int bcmRes = -1; // init as it fails

  void init_hardware()
  {

    if (bcmRes != 1) {
      RCUTILS_LOG_ERROR_NAMED(get_name(), "bcm2835_init() failed\n");
    }
    // Set the PWM frequency
    pwm_freq_ = 50.0;
    steeringMap = Float2ServoMap::create(pwm_freq_, 4095u, STEERING);
    throttleMap = Float2ServoMap::create(pwm_freq_, 4095u, THROTTLE);

    // Get the I2C device file descriptor
    i2c_fd_ = open("/dev/i2c-1", O_RDWR);
    while (i2c_fd_ < 0)
    {
      RCUTILS_LOG_ERROR_NAMED(get_name(), "Failed to open I2C device: %d, trying again in 1 sec.", i2c_fd_);
      usleep(1000000);
      i2c_fd_ = open("/dev/i2c-1", O_RDWR);
    }

    // Set the I2C slave address
    dev_address_ = I2C_ADDRESS;
    if (ioctl(i2c_fd_, I2C_SLAVE, dev_address_) < 0)
    {
      RCUTILS_LOG_ERROR_NAMED(get_name(), "Failed to set I2C slave address");
      return;
    }

    RCLCPP_INFO(rclcpp::get_logger(NODE_LABEL),"Setting PinMode");
    // Pin mode settings
    gpioSetMode(ENA, PI_OUTPUT);
    gpioSetMode(ENB, PI_OUTPUT);
    gpioSetMode(IN1, PI_OUTPUT);
    gpioSetMode(IN2, PI_OUTPUT);
    gpioSetMode(IN3, PI_OUTPUT);
    gpioSetMode(IN4, PI_OUTPUT);

    // setting direction pins to 0
    gpioWrite(IN1,PI_LOW);
    gpioWrite(IN2,PI_LOW);
    gpioWrite(IN3,PI_LOW);
    gpioWrite(IN4,PI_LOW);
    
    set_frequency(pwm_freq_);

    // Set the default PWM values
    set_pwm(0, 0, 0);
    set_pwm(1, 0, 0);
    set_pwm(15, 0, 0);
  }

  // Set the PWM frequency (in Hz)
  void set_frequency(uint16_t frequency)
  {
    uint8_t prescale = (uint8_t)(std::round(25000000.0 / (4096.0 * frequency)) - 1);
    write_reg(PCA9685_MODE1, 0b00010000); 
    usleep(1000);
    write_reg(PCA9685_PRESCALE, prescale);
    write_reg(PCA9685_MODE1, 0b10100000);
    usleep(1000);
    // write_reg(PCA9685_MODE2, 0x00);
    // usleep(1000);
  }

  // Set the PWM duty cycle for a specific channel
  void set_pwm(uint8_t channel, uint16_t on_time, uint16_t off_time)
  {
    uint8_t reg_offset = PCA9685_LED0_ON_L + 4 * channel;
    uint8_t on_time_low = on_time & 0xFF;
    uint8_t on_time_high = (on_time >> 8) & 0x0F;
    uint8_t off_time_low = off_time & 0xFF;
    uint8_t off_time_high = (off_time >> 8) & 0x0F;

    write_reg(reg_offset, on_time_low);
    write_reg(reg_offset + 1, on_time_high);
    write_reg(reg_offset + 2, off_time_low);
    write_reg(reg_offset + 3, off_time_high);
  }

  void write_reg(uint8_t reg, uint8_t data)
  {
    uint8_t buf[2] = {reg, data};
    if (write(i2c_fd_, buf, 2) != 2)
    {
      RCUTILS_LOG_ERROR_NAMED(get_name(), "Failed to write I2C register value");
    }
  }

  uint8_t read_reg(uint8_t reg)
  {
    uint8_t buf[] = {reg};
    if (write(i2c_fd_, buf, 1) != 2)
    {
      RCUTILS_LOG_ERROR_NAMED(get_name(), "Failed to write I2C register value");
    }
    if (read(i2c_fd_, buf, 1) != 2)
    {
      RCUTILS_LOG_ERROR_NAMED(get_name(), "Failed to read I2C register value");
    }
    return buf[0];
  }

  void control_callback(const minicar_interfaces::msg::Motors::SharedPtr control_msg)
  {
    float th = control_msg->throttle;
    
    if(th >= 0){
      this->setForward();
    }else{
      this->setBackward();
      th = -1*th;
    }
    setThrottle(th);
    setSteering(control_msg->steering);
  }

  void setForward(){
    gpioWrite(IN1,0);
    gpioWrite(IN2,1);
    gpioWrite(IN3,1);
    gpioWrite(IN4,0);
  }
		
  void setBackward(){
    gpioWrite(IN1,1);
    gpioWrite(IN2,0);
    gpioWrite(IN3,0);
    gpioWrite(IN4,1);
  }

  /**
   * @brief Set the Throttle object
   * 
   * @param[in] throttle between -1 and 1 respectively full backward and full forward
   */
  void setThrottle(float throttle)
  {
    if (std::abs(throttle) < 0.001 ) throttle = 0;
    uint16_t pwm = (uint16_t)(std::round(throttleMap->calPwm(throttle))) & 0x0FFF ;
    RCLCPP_DEBUG(this->get_logger(), "THROTTLE %d, RAW %f", pwm, throttle);
    set_pwm(THROTTLE_PWM_CHANNEL_LEFT, 0, pwm);
    set_pwm(THROTTLE_PWM_CHANNEL_RIGHT, 0, pwm);
  }

  /**
   * @brief Set the Steering object
   * 
   * @param steering between -1 and 1
   */
  void setSteering(float steering)
  {
    if (std::abs(steering) < 0.0001 ) steering = 0;
    uint16_t pwm = (uint16_t)(std::round(steeringMap->calPwm(steering))) & 0x0FFF ;
    RCLCPP_DEBUG(this->get_logger(), "STEERING %d, RAW %f", pwm, steering);
    set_pwm(STEERING_PWM_CHANNEL, 0, pwm);
  }

  rclcpp::Subscription<minicar_interfaces::msg::Motors>::SharedPtr control_subscription_;
  int i2c_fd_;
  uint8_t dev_address_;
  float pwm_freq_;
  Float2ServoMap::Ptr steeringMap;
  Float2ServoMap::Ptr throttleMap;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PCA9685>());
  rclcpp::shutdown();
  return 0;
}