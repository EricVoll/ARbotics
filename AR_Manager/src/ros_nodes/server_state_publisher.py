#!/usr/bin/env python
import rospy	from std_msgs.msg import String

def ARServerStatePublisher():
		
		pub = rospy.Publisher('state', String, queue_size=10)
		
		rospy.init_node('ar_state_publisher', anonymous=True)
		
		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
				#rospy.loginfo(hello_str)

				pub.publish( hello_str )
				
				
				rate.sleep()

if __name__ == '__main__':
		try:
				talker()
		except rospy.ROSInterruptException:
				pass	