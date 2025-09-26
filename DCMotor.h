#ifndef DCMotor_H
#define DCMotor_H

class DCMotor{
  public:
    DCMotor();
    ~DCMotor();
    void attach(unsigned int IN1,unsigned int IN2,unsigned int PWMPIN);
    void invertDirection();
    void setSpeed(unsigned int value);
    void setSpeed(bool direction,unsigned int value);
    void stop();
    void brake();
  private:
    unsigned int m_in1,m_in2,m_pwm;
    bool m_isAttached = false;
    bool m_isInvertdirection = false;
    bool m_isSetdirection = false;
};

#endif