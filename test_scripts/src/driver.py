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

		rate = rospy.Rate(3)
		while not rospy.is_shutdown():

			past_ticks = [self.left_ticks, self.right_ticks]
			past_time = rospy.get_time()
			rate.sleep()

			current_time = rospy.get_time()
			d_time = current_time - past_time
			delta_ticks = [self.left_ticks, self.right_ticks]

			const = 2 * 3.14 * .03
			vel_left = const * ((self.left_ticks - past_ticks[0]) / d_time / 540 )
			vel_right = const * ((self.right_ticks - past_ticks[1]) / d_time / 540)
			
			rospy.loginfo("Velocity left and right: %s, %s", vel_left, vel_right)

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

