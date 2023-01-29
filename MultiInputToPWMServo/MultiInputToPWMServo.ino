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
#define OUT_PIN_L 23 // connect to Left Sabertooth input // 13
#define OUT_PIN_R 4 // connect to Right Sabertooth input // 14

#define INTERRUPT_PIN_L 25 // connect to receiver "1ch" Left Stick 34
#define INTERRUPT_PIN_R 26 // connect to receiver "3ch" Right Stick 16
#define INTERRUPT_PIN_ER 15 // connect to receiver "5ch"  17

#define ERROR_LED_PIN 0 //12
#define STATUS_LED_ER 2 //15


// read pwm param
#define INPUT_PWM_T_US 20000// T[us]

// output pwm param
#define OUTPUT_PWM_T_MS 20// T[ms]
#define SERVO_HOME 1520// [us]//1491
//#define SERVO_LIMIT_LOW 991// [us] 1050
//#define SERVO_LIMIT_HIGH 1991// [us] 1950

// ros2 define
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}

// ROBOT kinematics param
#define RK_D 0.5 // [m] robot rotation radius
#define RK_VMAX 8.0 // [v/sec] max speed in Servo LIMIT HIGH
#define RK_VMIN -8.0 // [v/sec] min speed in Servo LIMIT LOW

volatile int SERVO_LIMIT_LOW = SERVO_HOME - 400;
volatile int SERVO_LIMIT_HIGH = SERVO_HOME + 400;

// enable rc (block ros2) flag
volatile bool er_flag = false;

// for read pwm
volatile int time_er_high = LOW;
volatile int time_er_change = 0;
volatile int time_er_change_prev = 0;
volatile int time_er_T = 0;

volatile int time_l_high = SERVO_HOME;
volatile int time_l_change = 0;
volatile int time_l_change_prev = 0;
volatile int time_l_T = 0;

volatile int time_r_high = SERVO_HOME;
volatile int time_r_change = 0;
volatile int time_r_change_prev = 0;
volatile int time_r_T = 0;

// global variables for pwm output
Ticker servo_l;
Ticker servo_r;

// global variables for ros2
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

volatile int ros2_time_l_high = SERVO_HOME;
volatile int ros2_time_r_high = SERVO_HOME;


// interrupt functions for reading puls width
void interrupt_er_change(){// enable rc
  time_er_change = micros();
  int diff = 0;
  if(time_er_change_prev != 0){
    diff = time_er_change - time_er_change_prev;
    if(diff < int(INPUT_PWM_T_US/4)){
      time_er_high = diff;
    }else{
      time_er_T = time_er_high + diff;
    }
  }
  time_er_change_prev = time_er_change;

  if(time_er_high > SERVO_LIMIT_HIGH){ 
    er_flag = true;
    digitalWrite(STATUS_LED_ER, HIGH);
  }
  else {
    er_flag = false;
    digitalWrite(STATUS_LED_ER, LOW);
  }
}
void interrupt_l_change(){// left stick
  time_l_change = micros();
  int diff = 0;
  if(time_l_change_prev != 0){
    diff = time_l_change - time_l_change_prev;
    if(diff < int(INPUT_PWM_T_US/4)){
      time_l_high = diff;
    }else{
      time_l_T = time_l_high + diff;
    }
  }
  time_l_change_prev = time_l_change;
}
void interrupt_r_change(){// right stick
  time_r_change = micros();
  int diff = 0;
  if(time_r_change_prev != 0){
    diff = time_r_change - time_r_change_prev;
    if(diff < int(INPUT_PWM_T_US/4)){
      time_r_high = diff;
    }else{
      time_r_T = time_r_high + diff;
    }
  }
  time_r_change_prev = time_r_change;
}

// PWM OUT functions
void pwm_l() {

  // rc/ros2 selection
  int out_l_high = (er_flag == true) ? time_l_high : ros2_time_l_high;  
  
  // limit
  out_l_high = out_l_high > SERVO_LIMIT_HIGH ? SERVO_LIMIT_HIGH : out_l_high;
  out_l_high = out_l_high < SERVO_LIMIT_LOW ? SERVO_LIMIT_LOW : out_l_high;

  digitalWrite(OUT_PIN_L, HIGH);
  delayMicroseconds(out_l_high);
  digitalWrite(OUT_PIN_L, LOW);
}
void pwm_r() {

  // rc/ros2 selection
  int out_r_high = (er_flag == true) ? time_r_high : ros2_time_r_high;  

  
  // limit
  out_r_high = out_r_high > SERVO_LIMIT_HIGH ? SERVO_LIMIT_HIGH : out_r_high;
  out_r_high = out_r_high < SERVO_LIMIT_LOW ? SERVO_LIMIT_LOW : out_r_high;

  digitalWrite(OUT_PIN_R, HIGH);
  delayMicroseconds(out_r_high);
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

  if(er_flag != true) {
    ros2_time_l_high = (vel_l - vmin)/(vmax - vmin)*(SERVO_LIMIT_HIGH-SERVO_LIMIT_LOW) + SERVO_LIMIT_LOW;
    ros2_time_r_high = (vel_r - vmin)/(vmax - vmin)*(SERVO_LIMIT_HIGH-SERVO_LIMIT_LOW) + SERVO_LIMIT_LOW;
  }
  
  // for debag
  digitalWrite(ERROR_LED_PIN, (msg->linear.x > 0) ? HIGH : LOW);
}

void pwm_read_init(){
  attachInterrupt(INTERRUPT_PIN_ER,  interrupt_er_change, CHANGE);
  attachInterrupt(INTERRUPT_PIN_L,  interrupt_l_change, CHANGE);
  attachInterrupt(INTERRUPT_PIN_R,  interrupt_r_change, CHANGE);
}

void pwm_write_init(){
  
  servo_l.attach_ms(OUTPUT_PWM_T_MS, pwm_l);//"pwm_l"called repeatedly every PWM_T[ms]
  servo_r.attach_ms(OUTPUT_PWM_T_MS, pwm_r);//"pwm_r"called repeatedly every PWM_T[ms]

  time_l_high = SERVO_HOME;
  time_r_high = SERVO_HOME;

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
  // put your setup code here, to run once:
  //Serial.begin(115200);

  // PIN setting
  pinMode(INTERRUPT_PIN_ER, INPUT_PULLUP);// read pwm pin
  pinMode(INTERRUPT_PIN_L, INPUT_PULLUP);// read pwm pin
  pinMode(INTERRUPT_PIN_R, INPUT_PULLUP);// read pwm pin

  pinMode(OUT_PIN_L, OUTPUT);// write pwm pin
  pinMode(OUT_PIN_R, OUTPUT);// write pwm pin

  pinMode(STATUS_LED_ER, OUTPUT);// show enable rc pin
  pinMode(ERROR_LED_PIN, OUTPUT);// ros2 error pin

  // pwm read setting
  pwm_read_init();

  // pwm write setting
  pwm_write_init();

  // ROS2 Subscriber setting
  ros2_init();

}

void loop() {
  delay(100);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));

  /*
  static volatile int count = 0;
  if(count > 100000) {
    Serial.print("L:T[");
    Serial.print(time_l_T); // 19929us(19.9ms)
    Serial.print("(us)],high[");
    Serial.print(time_l_high); // home: 1491us(1.49ms), forward: 1880us(1.88ms), backward: 1091us(1.09ms)
    Serial.println("(us)]");
    Serial.print("R:T[");
    Serial.print(time_r_T); // 19929us(19.9ms)
    Serial.print("(us)],high[");
    Serial.print(time_r_high); // home: 1496us(1.49ms), forward: 1896us(1.96ms), backward: 1117us(1.11ms)
    Serial.println("(us)]");
    count = 0;
  }
  count ++;
  */
}
