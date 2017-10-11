#!/usr/bin/env python

 

import argparse

import os

import sys

import time

import math
 

import rospy

import tf

from gazebo_msgs.msg import LinkStates

 

 

def link_states_callback(data):

    """ Subscriber to /gazebo/link_states

 

    This subscriber is responsible for printing the location and orientation

    of the base_link. A Gazebo LinkStates message has the following format:

 

        # broadcast all link states in world frame

        string[] name                 # link names

        geometry_msgs/Pose[] pose     # desired pose in world frame

        geometry_msgs/Twist[] twist   # desired twist in world frame

 

    A Pose looks like this:

 

        # A representation of pose in free space, composed of position and orientation.

        Point position

        Quaternion orientation

 

    And a Twist like this:

 

        # This expresses velocity in free space broken into its linear and angular parts.

        Vector3  linear

        Vector3  angular

 

    Effectively, for a give link, the pose gives the position and orientation of the

    link in world coordinates, and the twist gives linear and angular velocity.

    """

 

    # Get the index of the base_link

    # data.name contains all link names found in the URDF (or SDF) file

    base_link_index = data.name.index('rover::chassis')

    base_link_position = data.pose[base_link_index].position

    base_link_orientation = data.pose[base_link_index].orientation

 
    """
    print 'position.x    :' + str(base_link_position.x)

    print 'position.y    :' + str(base_link_position.y)

    print 'position.z    :' + str(base_link_position.z)

    print ''

    print 'orientation.x :' + str(base_link_orientation.x)

    print 'orientation.y :' + str(base_link_orientation.y)

    print 'orientation.z :' + str(base_link_orientation.z)

    print 'orientation.w :' + str(base_link_orientation.w)

    print ''
    """
 

    # Convert the quaternion data to roll, pitch, yaw

    quat = (

        base_link_orientation.x,

        base_link_orientation.y,

        base_link_orientation.z,

        base_link_orientation.w)

    roll, pitch, yaw = tf.transformations.euler_from_quaternion(quat)

 

    #print 'roll          :' + str(roll)

    #print 'pitch         :' + str(pitch)

    print 'yaw           :' + str(math.degrees(yaw))

 

if __name__ == '__main__':

 

    # Initialize this node

    rospy.init_node('read_link_states', anonymous=True)

 

    # Listen to the link states published by Gazebo

    rospy.Subscriber('gazebo/link_states', LinkStates, link_states_callback)

 

    # Spin so that this node does not exit

    rospy.spin()
