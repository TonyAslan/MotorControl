#include "DCMotor.h"
#include <Arduino.h>
#define LOW 0
#define HIGH 1
DCMotor::DCMotor(){};
DCMotor::~DCMotor(){};
void DCMotor::attach(unsigned int IN1, unsigned int IN2, unsigned int PWM) {
  m_in1 = IN1;
  m_in2 = IN2;
  m_pwm = PWM;
  pinMode(m_in1, OUTPUT);
  pinMode(m_in2, OUTPUT);
  pinMode(m_pwm, OUTPUT);
  m_isAttached = true;
}
void DCMotor::invertDirection() {
  if (!m_isAttached) return;
  m_isInvertdirection = !m_isInvertdirection;
}
void DCMotor::setSpeed(bool direction, unsigned int value) {
  if (!m_isAttached) return;
  digitalWrite(m_in1, direction ^ m_isInvertdirection);
  digitalWrite(m_in2, !direction ^ m_isInvertdirection);
  if (value > 255) value = 255;
  analogWrite(m_pwm, value);
  m_isSetdirection = true;
}
void DCMotor::setSpeed(unsigned int value) {
  if (!m_isAttached) return;
  if (m_isSetdirection)
    analogWrite(m_pwm, value);
}
void DCMotor::brake() {
  if (!m_isAttached) return;
  digitalWrite(m_in1, HIGH);
  digitalWrite(m_in2, HIGH);
  m_isSetdirection = 0;
}
void DCMotor::stop() {
  if (!m_isAttached) return;
  digitalWrite(m_in1, LOW);
  digitalWrite(m_in2, LOW);
  m_isSetdirection = 0;
}
