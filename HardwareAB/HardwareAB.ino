#include <MeAuriga.h>
#include <ros.h>
#include <ros/time.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <nav_msgs/Odometry.h>

MeEncoderOnBoard Encoder_1(SLOT1);
MeEncoderOnBoard Encoder_2(SLOT2);
long ticks_time;
float linx, angZ;
float lin_vel, ang_vel;
static char out_linx[6], out_angZ[6];
ros::NodeHandle		nh;


void velCallback(const geometry_msgs::Twist& vel){

	linx = vel.linear.x; 		angZ = vel.angular.z;
	dtostrf(linx, 5, 2, out_linx);
	dtostrf(angZ, 5, 2, out_angZ);
	nh.loginfo(out_linx);   nh.loginfo(out_angZ);
  	
  	Encoder_1.loop();
  	Encoder_2.loop();
  	
  	if (abs(angZ) > 0 && abs(linx + .5) > 0){
  	}
  	else if (abs(angZ) > 0){
  		Encoder_1.setMotorPwm(angZ * 50);
  		Encoder_2.setMotorPwm(angZ * -50);
  	}
  	else if (abs(linx + .5) > 0 || abs(linx - .5) > 0 ){
  		Encoder_1.setMotorPwm(linx * 50);
  		Encoder_2.setMotorPwm(linx * 50);
  	}
  	else {
  	  Encoder_1.setMotorPwm(0);
  	  Encoder_2.setMotorPwm(0);
  	  }
}
std_msgs::Int16 left_ticks_msg;
std_msgs::Int16 right_ticks_msg;

ros::Subscriber <geometry_msgs::Twist> vel_sub("/cmd_vel", velCallback);
ros::Publisher left_ticks_pub("/left_ticks", &left_ticks_msg);
ros::Publisher right_ticks_pub("/right_ticks", &right_ticks_msg);

void setup()
{
  Serial.begin(57600);
  nh.initNode();
  nh.subscribe(vel_sub);
  nh.advertise(left_ticks_pub);
  nh.advertise(right_ticks_pub);

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
  Encoder_1.setPosPid(0.18,0,0);
  Encoder_2.setPosPid(0.18,0,0);
  Encoder_1.setSpeedPid(0.18,0,0);
  Encoder_2.setSpeedPid(0.18,0,0);
}


void loop(){
      // publish wheel pos in ticks for each wheel
  if(millis() >= ticks_time){
    left_ticks_msg.data = Encoder_2.getPulsePos();
    left_ticks_pub.publish(&left_ticks_msg);
    right_ticks_msg.data = -1 *(Encoder_1.getPulsePos()); //compensate for reverse wiring of a motor
    right_ticks_pub.publish(&right_ticks_msg);
    ticks_time = millis()+10;
  }
  nh.spinOnce();

/*
  Serial.print("Spped 1:");
  Serial.print(Encoder_1.getCurrentSpeed());
  Serial.print("Spped 1:");
  Serial.println(Encoder_2.getCurrentSpeed());
  delay(10);
  */
}


void isr_process_encoder1(void)
{
  if(digitalRead(Encoder_1.getPortB()) == 0)
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
  if(digitalRead(Encoder_2.getPortB()) == 0)
  {
    Encoder_2.pulsePosMinus();
  }
  else
  {
    Encoder_2.pulsePosPlus();
  }
}
