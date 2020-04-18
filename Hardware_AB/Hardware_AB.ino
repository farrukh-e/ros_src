#include <MeAuriga.h>
#include <ros.h>
#include <ros/time.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>

MeEncoderOnBoard Encoder_1(SLOT1);
MeEncoderOnBoard Encoder_2(SLOT2);
long ticks_time;
long vel_count;
double lin_vel, ang_vel;
static char out_linx[6], out_angZ[6];

ros::NodeHandle   nh;
double straight_rpm, turn_rpm;
double tire_Rad = 0.03;

void isr_process_encoder1(void)
{
  if (digitalRead(Encoder_1.getPortB()) == 0)
  {
    Encoder_1.pulsePosMinus();
  }
  else
  {
    Encoder_1.pulsePosPlus();;
  }
}

void isr_process_encoder2(void)
{
  if (digitalRead(Encoder_2.getPortB()) == 0)
  {
    Encoder_2.pulsePosMinus();
  }
  else
  {
    Encoder_2.pulsePosPlus();
  }
}


void velCallback(const geometry_msgs::Twist& vel) {
  /* converting lin_vel to rpm*/
  straight_rpm = 9.549 * vel.linear.x / 0.03;
  turn_rpm = 9.549 *  vel.angular.z * .21 / 0.06;
  // rpm = rad/sec * m * sec/min * rev/m


  Encoder_1.runSpeed(-straight_rpm - turn_rpm);
  Encoder_2.runSpeed(straight_rpm - turn_rpm);


  dtostrf(straight_rpm, 5, 2, out_linx);
  dtostrf(turn_rpm, 5, 2, out_angZ);
  nh.loginfo(out_linx);   nh.loginfo(out_angZ);

}

std_msgs::Int16 left_ticks_msg;
std_msgs::Int16 right_ticks_msg;
std_msgs::Float32 vel_left_msg;
std_msgs::Float32 vel_right_msg;

ros::Subscriber <geometry_msgs::Twist> vel_sub("/cmd_vel", velCallback);
ros::Publisher left_ticks_pub("/left_ticks", &left_ticks_msg);
ros::Publisher right_ticks_pub("/right_ticks", &right_ticks_msg);
ros::Publisher vel_right_pub("/vel_right", &vel_right_msg);
ros::Publisher vel_left_pub("/vel_left", &vel_left_msg);

void setup()
{

  Serial.begin(57600);
  nh.initNode();
  nh.subscribe(vel_sub);
  nh.advertise(left_ticks_pub);
  nh.advertise(right_ticks_pub);
  nh.advertise(vel_left_pub);
  nh.advertise(vel_right_pub);

  // Encoder Set up
  attachInterrupt(Encoder_1.getIntNum(), isr_process_encoder1, RISING);
  attachInterrupt(Encoder_2.getIntNum(), isr_process_encoder2, RISING);

  //Set PWM 8KHz
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);

  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);

  Encoder_1.setPulse(9);
  Encoder_2.setPulse(9);
  Encoder_1.setRatio(45);
  Encoder_2.setRatio(45);
  Encoder_1.setPosPid(0.18, 0, 0);
  Encoder_2.setPosPid(0.18, 0, 0);
  Encoder_1.setSpeedPid(0.18, 0, 0);
  Encoder_2.setSpeedPid(0.18, 0, 0);
}


void loop() {
  // publish wheel pos in ticks for each wheel
  nh.spinOnce();
  if (millis() >= ticks_time)
  {
    left_ticks_msg.data = Encoder_2.getPulsePos();
    left_ticks_pub.publish(&left_ticks_msg);
    right_ticks_msg.data = -1 * (Encoder_1.getPulsePos()); //compensate for reverse wiring of a motor
    right_ticks_pub.publish(&right_ticks_msg);
    ticks_time = millis() + 10;
  }

  if (millis() >= vel_count)
  {

    vel_right_msg.data = -0.03 * Encoder_1.getCurrentSpeed() / 9.549;
    vel_left_msg.data = 0.03 * Encoder_2.getCurrentSpeed() / 9.549;

    vel_right_pub.publish(&vel_right_msg);
    vel_left_pub.publish(&vel_left_msg);
    vel_count = millis() + 100;

  }

  Encoder_1.loop();
  Encoder_2.loop();
}
