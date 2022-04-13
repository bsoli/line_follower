#!/usr/bin/env python
"""This script uses move base to direct the robot in cases where the line 
is no longer visible"""
import rospy, actionlib
import numpy as np
from std_msgs.msg import String, Bool
from sensor_msgs.msg import LaserScan
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from point import Point


def get_goal_pose(pose):
    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id = 'map'
    goal_pose.target_pose.pose.position.x = pose[0]
    goal_pose.target_pose.pose.position.y = pose[1]
    goal_pose.target_pose.pose.position.z = pose[2]
    goal_pose.target_pose.pose.orientation.w = pose[3]
    goal_pose.target_pose.pose.orientation.x = pose[4]
    goal_pose.target_pose.pose.orientation.y = pose[5]
    goal_pose.target_pose.pose.orientation.z = pose[6]
    return goal_pose

def scan_callback(msg):
    filtered_ranges = [r if r < msg.range_max or r > msg.range_min else msg.range_max for r in msg.ranges]
    quarter = len(filtered_ranges)/4
    ahead = filtered_ranges[int(quarter):int(3*quarter)]

def odom_callback(msg):
    global current_position, current_pose
    x = round(msg.pose.pose.position.x, 2)
    y = round(msg.pose.pose.position.y, 2)
    current_pose = [msg.pose.pose.position.x,
                    msg.pose.pose.position.y, 
                    msg.pose.pose.position.z, 
                    msg.pose.pose.orientation.w,
                    msg.pose.pose.orientation.x,
                    msg.pose.pose.orientation.y, 
                    msg.pose.pose.orientation.z]
    current_position = Point(x, y)

# stop the move_base if recieves signal that a line is visible
def stop_move_callback(msg):
    global stop
    print('hello stop')
    stop = msg.data
    if stop:
        client.cancel_all_goals()
        client.wait_for_result()
        stop = False
# if the robot cannot see, let move_base take over
def can_see_callback(msg):
    print('called')
    starting_point = current_position.to_numpy()
    starting_pose = current_pose
    stop = False
    print(starting_pose)
    prev_point = prev_position.to_numpy()
    #take the previous point to determine current trajectory
    slope = starting_point-prev_point
    # calculate the x and y coordinates of the a location two units away along 
    # the same line
    slope = (slope / np.linalg.norm(slope))
    goal_pose = np.array(starting_pose)
    #use the trajectory to determine where the robot should move
    goal_pose[0] += 2 * slope[0]
    goal_pose[1] += 2 * slope[1]
    print(goal_pose)
    goal_pose = get_goal_pose(goal_pose)
    client.send_goal(goal_pose)
    client.wait_for_result()


    

rospy.init_node('no_vision_move')
can_see_sub = rospy.Subscriber('can_see', Bool, can_see_callback)
odom_sub = rospy.Subscriber('odom', Odometry, odom_callback)
stop_move_base_sub = rospy.Subscriber('stop_move_base', Bool, stop_move_callback)
rate = rospy.Rate(10)
then = rospy.Time.now().to_sec()
current_position = Point(0, 0)
prev_position = current_position
current_pose = [0, 0, 0, 0, 0, 0, 0]
can_see = True
goal_pose = None
starting_pose = None 
stop = False
client = actionlib.SimpleActionClient('move_base', MoveBaseAction) 
client.wait_for_server()


while not rospy.is_shutdown():
    now = rospy.Time.now().to_sec()
    if now - then > 1:
        prev_position = current_position
        then = now
    rate.sleep()


