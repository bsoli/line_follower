#!/usr/bin/env python
"""This script uses computer vision to follow a yellow line
    with a PD controller"""
import rospy, cv2, cv_bridge
import numpy as np
from sensor_msgs.msg import Image, CompressedImage, LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class Follower:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw/compressed', CompressedImage, self.image_callback)
        self.lidar_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.cant_see_publish = rospy.Publisher('can_see', Bool, queue_size=1)
        self.stop_move_base_publisher = rospy.Publisher('stop_move_base', Bool, queue_size=1)
        self.twist = Twist()
        self.has_followed = False
        self.time_start = rospy.Time.now().to_sec()
        self.wall_ahead = False
        self.can_see = True
    # use lidar to look ahead for an obstace
    def scan_callback(self, msg):
        filtered_ranges = [r if r < msg.range_max or r > msg.range_min else msg.range_max for r in msg.ranges]
        quarter = len(filtered_ranges)/4
        ahead = filtered_ranges[int(quarter):int(3*quarter)]
        self.wall_ahead = min(ahead) < .75

    def image_callback(self, msg):
        # get image from CompressedImage
        comp_image = np.fromstring(msg.data, np.uint8)
        # Convert to open_cv compatible format
        image = cv2.imdecode(comp_image, cv2.IMREAD_COLOR)
        # filter out everything that's not yellow
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([ 30, 0, 0])
        upper_yellow = np.array([ 120, 255, 255])
        mask = cv2.inRange(hsv,  lower_yellow, upper_yellow)
        masked = cv2.bitwise_and(image, image, mask=mask)
    # clear all but a 20 pixel band near the top of the image
        h, w, d = image.shape
        search_top = 3 * h //4
        search_bot = search_top + 15
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0

    # Compute the "centroid" and display a red circle to denote it
        M = cv2.moments(mask)
        #this line checks that there is a centroid 
        if M['m00'] > 0:
            # if you lost the line but see it again, stop move_base
            if not self.can_see:
                self.stop_move_base_publisher.publish(True)
            cx = int(M['m10']/M['m00']) + 100
            cy = int(M['m01']/M['m00'])
            cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
            self.has_followed = True
            self.can_see = True
            # pid controler for line following
            err = cx - w/2
            self.twist.linear.x = 0.2
            self.twist.angular.z = -float(err) / 1000
            self.cmd_vel_pub.publish(self.twist)
        
        
        # if you've seen a line before but don't see one now, turn around    
        elif self.has_followed and not self.wall_ahead:
                self.twist.linear.x = 0
                self.twist.angular.z = .3
                self.cmd_vel_pub.publish(self.twist)
         #elif wall ahead and has_followed 
         #let move base take over until you are back at the line, 
        #if no line after x amount of time, treat it like the end of the line and 
        #go back to where you were
        elif (self.wall_ahead and self.has_followed):
            self.cant_see_publish.publish(True)
            self.can_see = False       

        # starting and there is no line immediately visible
        # go in a circle and hope for the best
        else:
            self.twist.linear.x = .3
            self.twist.angular.z = .3
            self.cmd_vel_pub.publish(self.twist)
        
        


rospy.init_node('follower')

follower = Follower()
rospy.spin()