#include <Ticker.h>

// read pwm param
#define INTERRUPT_PIN_L 34 // connect to receiver "3ch" Left Stick
#define INTERRUPT_PIN_R 16 // connect to receiver "5ch" Right Stick
#define INPUT_PWM_T_US 20000// T[us]

// output pwm param
#define OUT_PIN_L 13 // connect to Left Sabertooth input
#define OUT_PIN_R 14 // connect to Right Sabertooth input
#define OUTPUT_PWM_T_MS 20// T[ms]
#define SERVO_HOME 1491// [us]
#define SERVO_LIMIT_LOW 1050// [us]
#define SERVO_LIMIT_HIGH 1950// [us]

// for read pwm
volatile int time_l_change = 0;
volatile int time_l_change_prev = 0;
volatile int time_l_T = 0;
volatile int time_l_high = 0;

volatile int time_r_change = 0;
volatile int time_r_change_prev = 0;
volatile int time_r_T = 0;
volatile int time_r_high = 0;

// for output pwm
Ticker servo_l;
Ticker servo_r;

// interrupt functions for reading puls width
void interrupt_l_change(){
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
void interrupt_r_change(){
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
  
  int time_pwm_l = SERVO_HOME;
  time_pwm_l = (time_l_high < SERVO_LIMIT_LOW) ? SERVO_LIMIT_LOW : time_l_high;
  time_pwm_l = (time_l_high > SERVO_LIMIT_HIGH) ? SERVO_LIMIT_HIGH : time_l_high;
  
  digitalWrite(OUT_PIN_L, HIGH);
  delayMicroseconds(time_pwm_l);
  digitalWrite(OUT_PIN_L, LOW);
}
void pwm_r() {

  int time_pwm_r = SERVO_HOME;
  time_pwm_r = (time_r_high < SERVO_LIMIT_LOW) ? SERVO_LIMIT_LOW : time_r_high;
  time_pwm_r = (time_r_high > SERVO_LIMIT_HIGH) ? SERVO_LIMIT_HIGH : time_r_high;

  digitalWrite(OUT_PIN_R, HIGH);
  delayMicroseconds(time_pwm_r);
  digitalWrite(OUT_PIN_R, LOW);
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  // interruption setting
  pinMode(INTERRUPT_PIN_L, INPUT_PULLUP);
  pinMode(INTERRUPT_PIN_R, INPUT_PULLUP);
  attachInterrupt(INTERRUPT_PIN_L,  interrupt_l_change, CHANGE);
  attachInterrupt(INTERRUPT_PIN_R,  interrupt_r_change, CHANGE);
  
  // PWM OUT setting
  pinMode(OUT_PIN_L, OUTPUT);
  pinMode(OUT_PIN_R, OUTPUT);
  servo_l.attach_ms(OUTPUT_PWM_T_MS, pwm_l);//"pwm_l"called repeatedly every PWM_T[ms]
  servo_r.attach_ms(OUTPUT_PWM_T_MS, pwm_r);//"pwm_r"called repeatedly every PWM_T[ms]
}

void loop() {
  // put your main code here, to run repeatedly:

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
  
}
