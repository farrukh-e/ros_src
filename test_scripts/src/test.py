#!/usr/bin/env python


#encoder velocity passes eye test
#filtering needs to be imlemented with scipy
#remote control and harware abstraction works 
#odometry transform needs to published 
#odometry callculations need to added



import rospy
from std_msgs.msg import String
from std_msgs.msg import Int16

class driver:
	def __init__(self):

		self.right_ticks = 0
		self.left_ticks = 0 
		rospy.init_node('driver', anonymous=True)
		rospy.Subscriber('/left_ticks', Int16, self.left_encoder_callback)
		rospy.Subscriber('/right_ticks', Int16, self.left_encoder_callback)
		self.velocity_converter()

	def left_encoder_callback(self, left_ticks):
		self.left_ticks = abs(left_ticks.data)
		#rospy.loginfo("Left_encoder_reading: %s", left_ticks.data)
	
	def right_encoder_callback(self, right_ticks):
		self.right_ticks = abs(right_ticks.data)
		#rospy.loginfo("Right_encoder_reading: %s", right_ticks.data)

	def velocity_converter(self):

		rate = rospy.Rate(10)
		while not rospy.is_shutdown():

			current_ticks = [self.left_ticks, self.right_ticks]
			current_time = rospy.get_time()

			#Calculate velocity from angular velocity
			dt = (current_time - past_time).to_sec()
			dist_per_rev = (2 * 3.14 * .03) / 540
			vel_left = dist_per_rev * ((current_ticks[0] - past_ticks[0]) / dt )
			vel_right = dist_per_rev * ((current_ticks[1] - past_ticks[1]) / dt )
			
			#Average velocities and compensate for slip
			vx = 1.05 * (vel_right + vel_left)/2 
			vy = 0
			vth = 1.05 * (vel_right - vel_left)/len_between_wheels
			
			#Use orientation to calculate heading
			delta_x = (vx * cos(th)) * dt
			delta_y = (vx * sin(th)) * dt
			delta_th = vth * dt

			# Summarize 
			x += delta_x
			y += delta_y
			th += delta_th

			#Quaterion from th 
			#Publish blank tf
			#tf broadcaster
			#publish odom
			#set pos
			#set vel
			#publish tf
			#publish odom message
			rospy.loginfo("Velocity left and right: %s, %s", vel_left, vel_right)
			past_ticks = current_ticks
			past_time = current_time
			rate.sleep()

if __name__ == '__main__':
    try:
        d = driver()
        rospy.spin()

    except rospy.ROSInterruptException: 
        pass





            # get encoder data
            # get current time
            # get delta_t
            # get delta_theta
            # calcualte vel
            # past_time = curent time
