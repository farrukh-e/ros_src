#include <MeAuriga.h>
#include <ros.h>
#include <ros/time.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>

MeEncoderOnBoard Encoder_1(SLOT1);
MeEncoderOnBoard Encoder_2(SLOT2);

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
  	if (abs(angZ) > 0){
  		Encoder_1.setMotorPwm(angZ * -50);
  		Encoder_2.setMotorPwm(angZ * 50);
  	}
  	else if (abs(angZ) == 0 && abs(linx) > 0){
  		Encoder_1.setMotorPwm(linx * 50);
  		Encoder_2.setMotorPwm(linx * 50);
  	}
  	else {}
}

ros::Subscriber <geometry_msgs::Twist> vel_sub("/cmd_vel", velCallback);

void setup()
{
  Serial.begin(57600);
  nh.initNode();
  nh.subscribe(vel_sub);

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
  Encoder_1.setRatio(39.267);
  Encoder_2.setRatio(39.267);
  Encoder_1.setPosPid(0.18,0,0);
  Encoder_2.setPosPid(0.18,0,0);
  Encoder_1.setSpeedPid(0.18,0,0);
  Encoder_2.setSpeedPid(0.18,0,0);
}


void loop(){
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
