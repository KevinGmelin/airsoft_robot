#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Empty.h>
#include <airsoft_robot/YawPitchMotors.h>
#include <Servo.h>

#define encAYawPin 2
#define encBYawPin 4
#define encAPitchPin 3
#define encBPitchPin 5

#define motorYawPin 13
#define motorPitchPin 11



Servo pitch_motor, yaw_motor;
ros::NodeHandle  nh;

std_msgs::Int16 pitch_motor_msg, yaw_motor_msg;

std_msgs::Int16 yaw_encoder_topic_data;
std_msgs::Int16 pitch_encoder_topic_data;
ros::Publisher yawEncoderPub("yaw_encoder", &yaw_encoder_topic_data);
ros::Publisher pitchEncoderPub("pitch_encoder", &pitch_encoder_topic_data);

unsigned long publishingTimer = 0;

void recvCb( const airsoft_robot::YawPitchMotors &rcv_msg);
ros::Subscriber<airsoft_robot::YawPitchMotors> sub("arduinoIn", &recvCb );

void recvPitchUpperBound(const std_msgs::Empty &empty_msg);
ros::Subscriber<std_msgs::Empty> pitchUpperBoundSub("set_pitch_upper_bound", &recvPitchUpperBound);

void recvPitchLowerBound(const std_msgs::Empty &empty_msg);
ros::Subscriber<std_msgs::Empty> pitchLowerBoundSub("set_pitch_lower_bound", &recvPitchLowerBound);



volatile signed long encYawCount = 0;
volatile signed long encPitchCount = 0;

int pitchUpperBound = 1389;
int pitchLowerBound = -1389;


void setup()
{
  pinMode(encAYawPin, INPUT_PULLUP);
  pinMode(encBYawPin, INPUT_PULLUP);
  pinMode(encAPitchPin, INPUT_PULLUP);
  pinMode(encBPitchPin, INPUT_PULLUP);

  attachInterrupt(0, EncoderYawEvent, CHANGE);
  attachInterrupt(1, EncoderPitchEvent, CHANGE);

  nh.getHardware()->setBaud(115200);
  pitch_motor.attach(motorPitchPin);
  yaw_motor.attach(motorYawPin);

  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.subscribe(sub);
  nh.subscribe(pitchUpperBoundSub);
  nh.subscribe(pitchLowerBoundSub);
  nh.advertise(yawEncoderPub);
  nh.advertise(pitchEncoderPub);
  
  pitch_motor.write(90);
  yaw_motor.write(90);
  delay(10);


}

void loop()
{
  nh.spinOnce();
  
  if((millis() - publishingTimer) >= 50)
  {
    yaw_encoder_topic_data.data = encYawCount;
    pitch_encoder_topic_data.data = encPitchCount;
    yawEncoderPub.publish(&yaw_encoder_topic_data);
    pitchEncoderPub.publish(&pitch_encoder_topic_data);
    publishingTimer = millis();
  }
  
  delay(1);
}

void recvCb( const airsoft_robot::YawPitchMotors &rcv_msg)
{
  pitch_motor_msg.data = rcv_msg.pitch_motor;
  yaw_motor_msg.data = rcv_msg.yaw_motor;
  if(pitch_motor_msg.data > 0)
  {
    if(encPitchCount > pitchUpperBound)
    {
      pitch_motor_msg.data = 0;
    }
    else
    {
      pitch_motor_msg.data += 60;
    }
  }
  else if(pitch_motor_msg.data < 0)
  {
    if(encPitchCount < pitchLowerBound)
    {
      pitch_motor_msg.data = 0;
    }
    else
    {
      pitch_motor_msg.data -= 60;
    }
  }
  if(yaw_motor_msg.data > 0)
  {
    yaw_motor_msg.data += 60;
  }
  else if(yaw_motor_msg.data < 0)
  {
    yaw_motor_msg.data -= 60;
  }
  pitch_motor.writeMicroseconds(-pitch_motor_msg.data + 1500);
  yaw_motor.writeMicroseconds(-yaw_motor_msg.data + 1500);

}

void EncoderYawEvent()
{
  if (digitalRead(encAYawPin) == HIGH)
  {
    if (digitalRead(encBYawPin) == LOW)
    {
      encYawCount--;
    }
    else
    {
      encYawCount++;
    }
  }
  else
  {
    if (digitalRead(encBYawPin) == LOW)
    {
      encYawCount++;
    }
    else
    {
      encYawCount--;
    }
  }
}

void EncoderPitchEvent()
{
  if (digitalRead(encAPitchPin) == HIGH)
  {
    if (digitalRead(encBPitchPin) == LOW)
    {
      encPitchCount++;
    }
    else
    {
      encPitchCount--;
    }
  }
  else
  {
    if (digitalRead(encBPitchPin) == LOW)
    {
      encPitchCount--;
    }
    else
    {
      encPitchCount++;
    }
  }
}

void recvPitchUpperBound(const std_msgs::Empty &empty_msg)
{
  pitchUpperBound = encPitchCount - 100;
}

void recvPitchLowerBound(const std_msgs::Empty &empty_msg)
{
  pitchLowerBound = encPitchCount + 100;
}
