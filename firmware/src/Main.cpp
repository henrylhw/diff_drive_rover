#include <Arduino.h>

// https://youtu.be/SLDJAOEjVt4
// https://youtu.be/n9yU7u55zGg
// http://wiki.ros.org/differential_drive

#include <PID_v1.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int16.h>

#include "Config.hpp"
#include "Motor.hpp"
#include "ROSserial.h"

#define LOOPTIME 10
#define ENCODER_COUNT_PER_METER_IN_LOOP_TIME (12900 / LOOPTIME)

ros::MyNodeHandle nh;

int updatenh = 0;
Motor left(MOTOR_LEFT_GPIO_PIN1, MOTOR_LEFT_GPIO_PIN2, MOTOR_LEFT_PWM_PIN, GPIO_PIN_SPEED_LA, GPIO_PIN_SPEED_LB);
Motor right(MOTOR_RIGHT_GPIO_PIN1, MOTOR_RIGHT_GPIO_PIN2, MOTOR_RIGHT_PWM_PIN, GPIO_PIN_SPEED_RA, GPIO_PIN_SPEED_RB);


volatile long encoderLeftPos = 0;  // encoder 1
volatile long encoderRightPos = 0;  // encoder 2

double left_kp = 17, left_ki = 0, left_kd = 0.0;  // modify for optimal performance
double right_kp = 20, right_ki = 0, right_kd = 0.0;

float demandx = 0;  // in meter per second
float demandz = 0;  // radian per second

double demand_speed_left;
double demand_speed_right;

double right_input = 0, right_output = 0, right_setpoint = 0;
PID rightPID(&right_input, &right_output, &right_setpoint, right_kp, right_ki, right_kd, DIRECT);

double left_input = 0, left_output = 0, left_setpoint = 0;
PID leftPID(&left_input, &left_output, &left_setpoint, left_kp, left_ki, left_kd, DIRECT);

unsigned long currentMillis;
unsigned long prevMillis;

float encoderLeftDiff;
float encoderRightDiff;

float encoderLeftPrev;
float encoderRightPrev;

// float encoderLeftError;
// float encoderRightError;

void cmd_vel_cb(const geometry_msgs::Twist& twist) {
  demandx = twist.linear.x;
  demandz = twist.angular.z;
}
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", cmd_vel_cb);
//
std_msgs::Int16 left_wheel_msg;
ros::Publisher left_wheel_pub("lwheel", &left_wheel_msg);
std_msgs::Int16 right_wheel_msg;
ros::Publisher right_wheel_pub("rwheel", &right_wheel_msg);
//
double pos_act_left = 0;
double pos_act_right = 0;

// ************** encoders interrupts **************

// ************** encoder 1 *********************

void change_left_a() {
  // look for a low-to-high on channel A
  if (digitalRead(left.en_a) == HIGH) {
    // check channel B to see which way encoder is turning
    if (digitalRead(left.en_b) == LOW) {
      encoderLeftPos = encoderLeftPos + 1;  // CW
    } else {
      encoderLeftPos = encoderLeftPos - 1;  // CCW
    }
  } else  // must be a high-to-low edge on channel A
  {
    // check channel B to see which way encoder is turning
    if (digitalRead(left.en_b) == HIGH) {
      encoderLeftPos = encoderLeftPos + 1;  // CW
    } else {
      encoderLeftPos = encoderLeftPos - 1;  // CCW
    }
  }
}

void change_left_b() {
  // look for a low-to-high on channel B
  if (digitalRead(left.en_b) == HIGH) {
    // check channel A to see which way encoder is turning
    if (digitalRead(left.en_a) == HIGH) {
      encoderLeftPos = encoderLeftPos + 1;  // CW
    } else {
      encoderLeftPos = encoderLeftPos - 1;  // CCW
    }
  }
  // Look for a high-to-low on channel B
  else {
    // check channel B to see which way encoder is turning
    if (digitalRead(left.en_a) == LOW) {
      encoderLeftPos = encoderLeftPos + 1;  // CW
    } else {
      encoderLeftPos = encoderLeftPos - 1;  // CCW
    }
  }
}

// ************** encoder 2 *********************

void change_right_a() {
  // look for a low-to-high on channel A
  if (digitalRead(right.en_a) == HIGH) {
    // check channel B to see which way encoder is turning
    if (digitalRead(right.en_b) == LOW) {
      encoderRightPos = encoderRightPos - 1;  // CW
    } else {
      encoderRightPos = encoderRightPos + 1;  // CCW
    }
  } else  // must be a high-to-low edge on channel A
  {
    // check channel B to see which way encoder is turning
    if (digitalRead(right.en_b) == HIGH) {
      encoderRightPos = encoderRightPos - 1;  // CW
    } else {
      encoderRightPos = encoderRightPos + 1;  // CCW
    }
  }
}

void change_right_b() {
  // look for a low-to-high on channel B
  if (digitalRead(right.en_b) == HIGH) {
    // check channel A to see which way encoder is turning
    if (digitalRead(right.en_a) == HIGH) {
      encoderRightPos = encoderRightPos - 1;  // CW
    } else {
      encoderRightPos = encoderRightPos + 1;  // CCW
    }
  }
  // Look for a high-to-low on channel B
  else {
    // check channel B to see which way encoder is turning
    if (digitalRead(right.en_a) == LOW) {
      encoderRightPos = encoderRightPos - 1;  // CW
    } else {
      encoderRightPos = encoderRightPos + 1;  // CCW
    }
  }
}

void publishPos(double time) {
  left_wheel_msg.data = pos_act_left;
  right_wheel_msg.data = pos_act_right;
  left_wheel_pub.publish(&left_wheel_msg);
  right_wheel_pub.publish(&right_wheel_msg);
}

void loop() {
  currentMillis = millis();
  if (currentMillis - prevMillis >= LOOPTIME) {
    prevMillis = currentMillis;

    // convert distance unit to encoder counts

    /*
      There are 12900 encoder counts in one meter
      That is 129 per 10 millisecound loop as 1 meter/second velocity

      Demand in meter/second but we need to convert to encoder counts per 10ms loop
    */
    /*
      Distance between wheels is 290mm, half of that is 145mm
      Circumference of 290mm circle is 910mm, to turn 180 (pi radians),
      each wheel needs to drive half of that, which is 455mm.
      To turn on radian, earch wheel needs to drive 455/pi = 145mm (per second for 1 rad/s)
    */
    encoderLeftDiff = encoderLeftPos - encoderLeftPrev;
    encoderLeftPrev = encoderLeftPos;
    pos_act_left = encoderLeftDiff / ENCODER_COUNT_PER_METER_IN_LOOP_TIME;

    encoderRightDiff = encoderRightPos - encoderRightPrev;
    encoderRightPrev = encoderRightPos;
    pos_act_right = encoderRightDiff / ENCODER_COUNT_PER_METER_IN_LOOP_TIME;

    demand_speed_left = demandx - (demandz * 0.145);
    demand_speed_right = demandx + (demandz * 0.145);
    // encoderLeftError = (demand_speed_left * ENCODER_COUNT_PER_METER_IN_LOOP_TIME) - encoderLeftDiff;
    // encoderRightError = (demand_speed_right * ENCODER_COUNT_PER_METER_IN_LOOP_TIME) - encoderRightDiff;

    left_setpoint = demand_speed_left * ENCODER_COUNT_PER_METER_IN_LOOP_TIME;
    right_setpoint = demand_speed_right * ENCODER_COUNT_PER_METER_IN_LOOP_TIME;

    left_input = encoderLeftDiff;
    right_input = encoderRightDiff;

    leftPID.Compute();
    rightPID.Compute();

    left.turn(left_output);
    right.turn(right_output);
    //  Console.print(pos_act_left);
    //  Console.print(",");
    //  Console.println(pos_act_right);

    publishPos(LOOPTIME);
    if (updatenh > 10) {
      nh.spinOnce();
      updatenh = 0;
    } else {
      updatenh++;
    }
  }
}

// void loop() {
//   currentMillis = millis();
//   if (currentMillis - prevMillis > 100) {
//     prevMillis = currentMillis;
//     Console.println("left "+String(encoderLeftPos));
//     Console.println("right "+String(encoderRightPos));
//   }
// }

void setupROSserial() {
  Console.println(F("ROSserial init, wait..."));
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(left_wheel_pub);
  nh.advertise(right_wheel_pub);
  nh.negotiateTopics();
  Console.println(F("ROSserial init, done!"));
}

void setup() {
  Console.begin(115200);

  setupROSserial();

  rightPID.SetMode(AUTOMATIC);
  rightPID.SetSampleTime(10);
  rightPID.SetOutputLimits(-200, 200);

  leftPID.SetMode(AUTOMATIC);
  leftPID.SetSampleTime(10);
  leftPID.SetOutputLimits(-200, 200);

  //  Console.println("Basic Encoder Test:");
  attachInterrupt(digitalPinToInterrupt(left.en_a), change_left_a, RISING);
  attachInterrupt(digitalPinToInterrupt(left.en_b), change_left_b, RISING);
  attachInterrupt(digitalPinToInterrupt(right.en_a), change_right_a, RISING);
  attachInterrupt(digitalPinToInterrupt(right.en_b), change_right_b, RISING);
  right.turn(-100);
  left.turn(-100);
  //while(1);
}
