#!/usr/bin/env python3

# Python libs
import numpy as np
import cv2

# Ros libraries
import rospy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError


class ImgProcessor:
	"""
	Subscirbes to compressed color and depth topics and transforms img messages to raw formats.
	"""
	def __init__(self):
		self.bridge = CvBridge()

		# subscribe to color image topic
		self.color_image_topic = "/local_map/semantic/compressed"
		print('Subscribing to', self.color_image_topic)
		self.subscriber = rospy.Subscriber(self.color_image_topic, CompressedImage, self.callback_color,  queue_size=1)

		# topic where we publish
		self.color_pub = rospy.Publisher("/local_map/semantic/image_raw", Image, queue_size=1)

		# subscribe to depth image topic
		self.depth_image_topic = "/local_map/depth/compressed"
		print('Subscribing to', self.depth_image_topic)
		self.subscriber = rospy.Subscriber(self.depth_image_topic, CompressedImage, self.callback_depth,  queue_size=1)

		# topic where we publish
		self.depth_pub = rospy.Publisher("/local_map/depth/image_raw", Image, queue_size=1)

	def callback_color(self, img_msg):
		try:
			img_compressed = self.bridge.compressed_imgmsg_to_cv2(img_msg, "bgr8")
			color_msg = self.bridge.cv2_to_imgmsg(img_compressed, "bgr8")
			color_msg.header = img_msg.header
			self.color_pub.publish(color_msg)
		except CvBridgeError as e:
		    print(e)

	def callback_depth(self, img_msg):
		try:
			img_compressed = self.bridge.compressed_imgmsg_to_cv2(img_msg, "bgr8")
			img_compressed = cv2.cvtColor(img_compressed, cv2.COLOR_BGR2GRAY)
			depth_image = np.interp(img_compressed, (img_compressed.min(), img_compressed.max()), (0, 65535))
			depth_msg = self.bridge.cv2_to_imgmsg(np.uint16(depth_image), "16UC1")
			depth_msg.header = img_msg.header
			self.depth_pub.publish(depth_msg)
		except CvBridgeError as e:
		    print(e)


if __name__ == '__main__':
    rospy.init_node('img_msg_converter', anonymous=True)
    proc = ImgProcessor() 
    rospy.spin()
