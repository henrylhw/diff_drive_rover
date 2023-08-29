/*
  Motor.h - Library for working with the Cytron SPG30E-30K.
  Created by Vinay Lanka, January 27, 2021.

  Import Motor - Cytron SPG30E-30K
*/
#ifndef Motor_h
#define Motor_h

class Motor {
 public:
  // Constructor - Plus and Minus are the Motor output / en_a and en_b are the encoder inputs
  Motor(int plus, int minus, int pwm, int en_a, int en_b) ;

  void turn(int value);
  // Console.println("weird function");
  // Motor Outputs - plus is one direction and minus is the other
  int pwm;
  int plus;
  int minus;
  // Encoder Inputs
  int en_a;
  int en_b;
};

Motor::Motor(int plus, int minus, int pwm, int en_a, int en_b) {
  pinMode(plus, OUTPUT);
  pinMode(minus, OUTPUT);
  pinMode(pwm, OUTPUT);
  pinMode(en_a, INPUT_PULLDOWN);
  pinMode(en_b, INPUT_PULLDOWN);
  Motor::pwm = pwm;
  Motor::plus = plus;
  Motor::minus = minus;
  Motor::en_a = en_a;
  Motor::en_b = en_b;
}

void Motor::turn(int value) {
  // Console.print("pwm value:  ");
  // Console.println(value);
  if (value >= 0) {
    analogWrite(pwm, value);
    digitalWrite(plus, 1);
    digitalWrite(minus, 0);
  } else {
    analogWrite(pwm, abs(value));
    digitalWrite(plus, 0);
    digitalWrite(minus, 1);
  }
}
#endif