#!/usr/bin/env python


#encoder velocity passes eye test
#filtering needs to be imlemented with scipy
#remote control and harware abstraction works 
#odometry transform needs to published 
#odometry callculations need to added


import math
import rospy
import tf
from std_msgs.msg import String
from std_msgs.msg import Int16
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3


class driver:
	def __init__(self):

		self.right_ticks = 0
		self.left_ticks = 0 
		self.len_between_wheels = .185

		rospy.init_node('driver', anonymous=True)
		rospy.Subscriber('/left_ticks', Int16, self.left_encoder_callback)
		rospy.Subscriber('/right_ticks', Int16, self.right_encoder_callback)
		#tf broadcaster
		self.odom_pub = rospy.Publisher("odom", Odometry, queue_size = 50)
		self.odom_broadcaster = tf.TransformBroadcaster()

		self.velocity_converter()

	def left_encoder_callback(self, left_ticks):
		self.left_ticks = abs(left_ticks.data)
		#rospy.loginfo("Left_encoder_reading: %s", left_ticks.data)
	
	def right_encoder_callback(self, right_ticks):
		self.right_ticks = abs(right_ticks.data)
		#rospy.loginfo("Right_encoder_reading: %s", right_ticks.data)

	def velocity_converter(self):
		past_time = rospy.Time.now()
		past_ticks = [0,0]
		x,y,th = 0,0,0
		past_ticks_left, past_ticks_right = 0,0 
		vel_left = 0.0
		rate = rospy.Rate(3)
		while not rospy.is_shutdown():

			#current_ticks = [self.left_ticks, self.right_ticks]
			
			current_time = rospy.Time.now()

			#Calculate velocity from angular velocity
			dt = (current_time - past_time).to_sec()
			dist_per_rev = (2 * 3.14 * .03) / 540
			vel_left = dist_per_rev * ((self.left_ticks - past_ticks_left) / dt )
			vel_right = dist_per_rev * ((self.right_ticks - past_ticks_right) / dt )
			
			#Average velocities and compensate for slip
			vx = 1.05 * (vel_right + vel_left)/2 
			vy = 0
			vth = 1.05 * (vel_right - vel_left)/self.len_between_wheels
			
			#Use orientation to calculate heading
			delta_x = (vx * math.cos(th)) * dt
			delta_y = (vx * math.sin(th)) * dt
			delta_th = vth * dt

			# Summarize 
			x += 10* delta_x
			y += delta_y
			th += delta_th

			#Quaterion from th 
			odom_quat = tf.transformations.quaternion_from_euler(0,0,th)
			#Publish blank tf
			self.odom_broadcaster.sendTransform(
				(x,y, 0.),
				odom_quat,
				current_time,
				"link_chassis",
				"odom")

			
			#publish odom
			odom = Odometry()
			odom.header.stamp = current_time
			odom.header.frame_id = "odom"
			#set pos
			odom.pose.pose = Pose(Point(x,y,0.), Quaternion(*odom_quat))
			#set vel
			odom.child_frame_id = "link_chassis"
			odom.twist.twist = Twist(Vector3(vx,vy, 0), Vector3(0, 0, vth))
			
			#publish odom message
			self.odom_pub.publish(odom)

			rospy.loginfo("Velocity left and right: %s, %s", vel_left, vel_right)
			#rospy.loginfo("velcoity x and vth, delta_th: %s, %s, %s", vx, vth, delta_th)
			past_ticks_left = self.left_ticks
			past_ticks_right = self.right_ticks
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