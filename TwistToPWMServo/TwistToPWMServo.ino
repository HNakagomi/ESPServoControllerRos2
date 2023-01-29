// ros2 header
#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>

// pwm output header
#include <Ticker.h>

// PIN define
#define ERROR_LED_PIN 0
#define OUT_PIN_L 23 //13 // connect to Left Sabertooth input
#define OUT_PIN_R 4 //14 // connect to Right Sabertooth input

// ros2 define
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}

// output pwm param
#define OUTPUT_PWM_T_MS 20// T[ms]
#define SERVO_HOME 1491// [us]
#define SERVO_LIMIT_LOW 1050// [us]
#define SERVO_LIMIT_HIGH 1950// [us]

// ROBOT kinematics param
#define RK_D 0.5 // [m] robot rotation radius
#define RK_VMAX 8.0 // [v/sec] max speed in Servo LIMIT HIGH
#define RK_VMIN -8.0 // [v/sec] min speed in Servo LIMIT LOW

// global variables for ros2
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

// global variables for pwm output
Ticker servo_l;
Ticker servo_r;
volatile int time_l_high = SERVO_HOME;
volatile int time_r_high = SERVO_HOME;

// PWM OUT functions
void pwm_l() {
  int time_pwm_l = time_l_high;
  digitalWrite(OUT_PIN_L, HIGH);
  delayMicroseconds(time_pwm_l);
  digitalWrite(OUT_PIN_L, LOW);
}
void pwm_r() {
  int time_pwm_r = time_r_high;
  digitalWrite(OUT_PIN_R, HIGH);
  delayMicroseconds(time_pwm_r);
  digitalWrite(OUT_PIN_R, LOW);
}

void error_loop(){
  while(1){
    digitalWrite(ERROR_LED_PIN, !digitalRead(ERROR_LED_PIN));
    delay(100);
  }
}

//twist message subscriber
void twist_callback(const void *msgin) {
  
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;

  float v = msg->linear.x;//[m/sec]
  float w = msg->angular.z;//[rad/sec]
  float d = RK_D;// (robot_rotation_radius / 2.0) [m]

  // twist to LR Wheel speed (differential drive robot model) 
  float vel_l = v - d*w;
  float vel_r = v + d*w;

  // Wheel speed to PWM high length
  float vmax = RK_VMAX;//[m/sec]
  float vmin = RK_VMIN;//[m/sec]
  time_l_high = (vel_l - vmin)/(vmax - vmin)*(SERVO_LIMIT_HIGH-SERVO_LIMIT_LOW) + SERVO_LIMIT_LOW;
  time_r_high = (vel_r - vmin)/(vmax - vmin)*(SERVO_LIMIT_HIGH-SERVO_LIMIT_LOW) + SERVO_LIMIT_LOW;

  // limit
  time_l_high = time_l_high > SERVO_LIMIT_HIGH ? SERVO_LIMIT_HIGH : time_l_high;
  time_l_high = time_l_high < SERVO_LIMIT_LOW ? SERVO_LIMIT_LOW : time_l_high;
  time_r_high = time_r_high > SERVO_LIMIT_HIGH ? SERVO_LIMIT_HIGH : time_r_high;
  time_r_high = time_r_high < SERVO_LIMIT_LOW ? SERVO_LIMIT_LOW : time_r_high;

  // for debag
  digitalWrite(ERROR_LED_PIN, (msg->linear.x > 0) ? HIGH : LOW);
  
}


void ros2_init() {

  set_microros_transports(); // connect with USB serial
  //set_microros_wifi_transports("aterm-7842f1", "31dfe778e93c0", "192.168.179.3", 8888); // Connect with Wifi. IP: Host PC IP adress runnning micro-ros agent

  digitalWrite(ERROR_LED_PIN, HIGH);
  delay(2000);
  
  allocator = rcl_get_default_allocator();

   //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "micro_ros_arduino_twist_subscriber"));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &twist_callback, ON_NEW_DATA));

}

void setup() {

  // PIN setting
  pinMode(ERROR_LED_PIN, OUTPUT);
  pinMode(OUT_PIN_L, OUTPUT);
  pinMode(OUT_PIN_R, OUTPUT);

  // PWMOut setting
  servo_l.attach_ms(OUTPUT_PWM_T_MS, pwm_l);//"pwm_l"called repeatedly every PWM_T[ms]
  servo_r.attach_ms(OUTPUT_PWM_T_MS, pwm_r);//"pwm_r"called repeatedly every PWM_T[ms]
  time_l_high = SERVO_HOME;
  time_r_high = SERVO_HOME;

  // ROS2 Subscriber setting
  ros2_init();

}

void loop() {
  delay(100);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
