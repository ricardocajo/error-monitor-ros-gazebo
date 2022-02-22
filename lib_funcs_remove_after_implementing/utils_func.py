#! /usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Point
from gazebo_msgs.msg import ModelStates

# A dictionary with the indexes of the objects in the gazebo ModelStates
modelStates_indexes = {}

def update_modelStates_info(topic_msg: ModelStates):
    """ Update the dictionary with the information about the /gazebo/model_states topic
    :param topic_msg: The last message received from the ModelStates topic
    """
    global modelStates_indexes
    names = topic_msg.name
    indexes = {}
    index_counter = 0
    for name in names:
        indexes[name] = index_counter
        index_counter += 1
    modelStates_indexes = indexes.copy()
    
def get_modelStates_pos(object_name: str, topic_msg: ModelStates):
    """ The position of the object in gazebo simulation
    :param object_name: The name of the object in the ros environment
    :param topic_msg: The last message received from the ModelStates topic
    """
    object_index = modelStates_indexes[object_name]
    return topic_msg.pose[object_index].position

def get_topic_msg(topic: str, msg_type):
    """ The last message received in the topic "topic"
    :param topic: The topic we want to get information from. Ex. '/topic_xpto'
    :param msg_type: The message type of the message received from the topic
    """
    return rospy.wait_for_message(topic, msg_type)
    
def distance_between_gazebo(object1_name: str, object2_name: str, topic_msg: ModelStates):
    """ The absolute distance between two objects in the gazebo simulation for the x/y axis
    :param object1_name: A string with the object name in the ros environment
    :param object2_name: A string with the object name in the ros environment
    :param topic_msg: The last message received from the ModelStates topic
    """
    update_modelStates_info(topic_msg)
    object1_pos = get_modelStates_pos(object1_name, topic_msg)
    object2_pos = get_modelStates_pos(object2_name, topic_msg)
    return distance_xy(object1_pos,object2_pos)
    
def distance_xy(point1: Point, point2: Point):
    """ The absolute distance between 2 points in the cartesian coordinate system
    """
    return math.sqrt((point1.x - point2.x)*(point1.x - point2.x) + (point1.y - point2.y)*(point1.y - point2.y))

def position_x_gazebo(robot_name: str, topic_msg: ModelStates):
    """ The position in the x axis of the robot in the gazebo simulation
    :param robot_name: A string with the robot name in the ros environment
    :param topic_msg: The last message received from the ModelStates topic
    """
    update_modelStates_info(topic_msg)
    return get_modelStates_pos(robot_name, topic_msg).x

def position_y_gazebo(robot_name: str, topic_msg: ModelStates):
    """ The position in the y axis of the robot in the gazebo simulation
    :param robot_name: A string with the robot name in the ros environment
    :param topic_msg: The last message received from the ModelStates topic
    """
    update_modelStates_info(topic_msg)
    return get_modelStates_pos(robot_name, topic_msg).y