#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int16.h>
#include <DualVNH5019MotorShield.h>

#define STATE_WAITING         1
#define STATE_TRIGGER_BACK    2
#define STATE_TRIGGER_FORWARD 3
#define STATE_RELOAD_BACK     4
#define STATE_RELOAD_FORWARD  5

#define RELOAD_LENGTH         1460
#define RELOAD_BACK_SPEED     400
#define RELOAD_FORWARD_SPEED  -400
#define FIRE_BACK_SPEED       400
#define FIRE_FORWARD_SPEED    -400

int turretState = STATE_WAITING;
bool fireFlag = false;

#define encAReloadPin 3
#define encBReloadPin 5
#define buttonReloadPin 11
#define buttonFirePin 13


const int potPin = 2;

DualVNH5019MotorShield md;

volatile signed long encReloadCount = 0;

ros::NodeHandle  nh;

void recvFireCmdCB(const std_msgs::Empty &empty_msg);
ros::Subscriber<std_msgs::Empty> sub("turret_shoot", &recvFireCmdCB );

std_msgs::Int16 reload_encoder_topic_data;
std_msgs::Int16 shooting_encoder_topic_data;
std_msgs::Int16 reload_current_topic_data;
std_msgs::Int16 shooting_current_topic_data;
ros::Publisher reloadEncoderPub("reload_encoder", &reload_encoder_topic_data);
ros::Publisher shootingEncoderPub("shooting_encoder", &shooting_encoder_topic_data);
ros::Publisher reloadCurrentPub("reload_current", &reload_current_topic_data);
ros::Publisher shootingCurrentPub("shooting_current", &shooting_current_topic_data);

unsigned long publishingTimer = 0;


void setup() {
  pinMode(encAReloadPin, INPUT_PULLUP);
  pinMode(encBReloadPin, INPUT_PULLUP);
  pinMode(buttonReloadPin, INPUT_PULLUP);
  pinMode(buttonFirePin, INPUT_PULLUP);

  md.init();
  

  attachInterrupt(1, EncoderReloadEvent, CHANGE);

  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(reloadEncoderPub);
  nh.advertise(shootingEncoderPub);
  nh.advertise(reloadCurrentPub);
  nh.advertise(shootingCurrentPub);

  delay(10);
}


void loop() {
  nh.spinOnce();
//  Serial.print("Encoder Count: ");
//  Serial.print(encReloadCount);
//  Serial.print(", Pot Value: ");
//  Serial.println(analogRead(potPin));

  if((millis() - publishingTimer) >= 20)
  {
    reload_encoder_topic_data.data = encReloadCount;
    shooting_encoder_topic_data.data = analogRead(potPin);
    reload_current_topic_data.data = md.getM1CurrentMilliamps();
    shooting_current_topic_data.data = md.getM2CurrentMilliamps();
    reloadEncoderPub.publish(&reload_encoder_topic_data);
    shootingEncoderPub.publish(&shooting_encoder_topic_data);
    reloadCurrentPub.publish(&reload_current_topic_data);
    shootingCurrentPub.publish(&shooting_current_topic_data);
    publishingTimer = millis();
  }

  
  if(turretState == STATE_WAITING)
  {
    md.setSpeeds(0, 0);
    md.setBrakes(400, 400);
    if(fireFlag)
    {
      turretState = STATE_TRIGGER_BACK;
    }

  }
  else if(turretState == STATE_TRIGGER_BACK)
  {
    md.setSpeeds(0, FIRE_BACK_SPEED);
    if(analogRead(potPin) <= 300)
    {
      md.setSpeeds(0, FIRE_FORWARD_SPEED);
      turretState = STATE_TRIGGER_FORWARD;
    }

  }
  else if(turretState == STATE_TRIGGER_FORWARD)
  {
    md.setSpeeds(0, FIRE_FORWARD_SPEED);
    if(analogRead(potPin) >= 390)
    {
      md.setSpeeds(0,0);
      md.setBrakes(0, 400);
      turretState = STATE_RELOAD_BACK;
    }
  }
  else if(turretState == STATE_RELOAD_BACK)
  {
    md.setBrakes(0, 400);
    md.setSpeeds(RELOAD_BACK_SPEED,0);
    if(encReloadCount >= RELOAD_LENGTH)
    {
      md.setSpeeds(RELOAD_FORWARD_SPEED,0);
      turretState = STATE_RELOAD_FORWARD;
    }
  }
  else if(turretState == STATE_RELOAD_FORWARD)
  {
    md.setSpeeds(RELOAD_FORWARD_SPEED,0);
    if(encReloadCount <= 0)
    {
      md.setSpeeds(0,0);
      md.setBrakes(400,0);
      turretState = STATE_WAITING;
      fireFlag = false;
    }
  }
  else
  {
    fireFlag = false;
    turretState = STATE_WAITING;
  }

  delay(1);
}


void recvFireCmdCB(const std_msgs::Empty &empty_msg)
{
  fireFlag = true;
}


void EncoderReloadEvent()
{
  if (digitalRead(encAReloadPin) == HIGH)
  {
    if (digitalRead(encBReloadPin) == LOW)
    {
      encReloadCount--;
    }
    else
    {
      encReloadCount++;
    }
  }
  else
  {
    if (digitalRead(encBReloadPin) == LOW)
    {
      encReloadCount++;
    }
    else
    {
      encReloadCount--;
    }
  }
}
