#!/usr/bin/env python


import math
import rospy
import tf
from std_msgs.msg import String
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3


class driver:


	def __init__(self):

 		self.len_between_wheels = .185
 		self.left_velocity, self.right_velocity = 0.0, 0.0
		rospy.Subscriber('/left_velocity', Float32, self.left_encoder_callback)
		rospy.Subscriber('/right_velocity', Float32, self.right_encoder_callback)
		#tf broadcaster
		self.odom_pub = rospy.Publisher("odom", Odometry, queue_size = 50)
		self.odom_broadcaster = tf.TransformBroadcaster()

		self.velocity_converter()

	def left_encoder_callback(self, left_velocity):
		self.left_velocity = abs(left_velocity.data)
		#rospy.loginfo("Left_encoder_reading: %s", left_velocity.data)
	
	def right_encoder_callback(self, right_velocity):
		self.right_velocity = abs(right_velocity.data)
		#rospy.loginfo("Right_encoder_reading: %s", right_velocity.data)

	def velocity_converter(self):
		
		past_time = rospy.Time.now()
		x,y,th = 0,0,0
		rate = rospy.Rate(5)
		while not rospy.is_shutdown():

			current_time = rospy.Time.now()

			#Calculate velocity from angular velocity
			dt = (current_time - past_time).to_sec()
			
			#Average velocities and compensate for slip
			vx = 1.05 * (self.right_velocity + self.left_velocity)/2 
			vy = 0
			vth = 1.05 * (self.right_velocity - self.left_velocity)/self.len_between_wheels
			
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

			rospy.loginfo("Velocity left and right: %s, %s", self.left_velocity, self.right_velocity)
			#rospy.loginfo("velcoity x and vth, delta_th: %s, %s, %s", vx, vth, delta_th)
	
			past_time = current_time
			rate.sleep()

if __name__ == '__main__':
    rospy.init_node('driver', anonymous=True)
    try:
        driver()
        rospy.spin()

    except rospy.ROSInterruptException: 
        pass





            # get encoder data
            # get current time
            # get delta_t
            # get delta_theta
            # calcualte vel
            # past_time = curent time