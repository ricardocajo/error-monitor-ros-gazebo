#! /usr/bin/env python

""" A set of usefull functions that can be called by the compiler """

import warnings
from . import config
from . import utils_func
from gazebo_msgs.msg import ModelStates

__author__ = "Ricardo Cordeiro"
__email__ = "ricas.cordeiro@gmail.com"
__credits__ = ["Alcides Fonseca","Christopher S. Timperley","Paulo Canelas"]
__status__ = "Prototype"


def distance_between(object1_name: str, object2_name: str, simulator_topic_msg):
    """ The absolute distance between two objects in the simulation for the x/y axis
    :param object1_name: A string with the object name in the ros environment
    :param object2_name: A string with the object name in the ros environment
    :param simulator_topic_msg: The last message received from the localization topic of the simulator being used
    """
    if config.simulator == config.simulators_dic["gazebo"]:
        return utils_func.distance_between_gazebo(object1_name, object2_name, simulator_topic_msg)
    elif config.simulator == config.simulators_dic["unity"]:
        warnings.warn("distance_between function for this simulator not implemented.")
        return None
    else:
        warnings.warn("The simulator defined in config.py file doesn't exist.")
    return None
    
def get_topic_msg_modelStates():
    """ The last message received in the '/gazebo/model_states' topic
    """
    return utils_func.get_topic_msg('/gazebo/model_states', ModelStates)
          
def localization_error(robot_name: str, robot_topic_msg, simulator_topic_msg):
    """ The localization error between the robot and the simulation
    :param object_name: A string with the robot name in the ros environment
    :param robot_topic_msg: The last message received from the localization topic of the robot
    :param simulator_topic_msg: The last message received from the localization topic of the simulator being used
    """
    if config.simulator == config.simulators_dic["gazebo"]:#TODO
        utils_func.update_modelStates_info(simulator_topic_msg)
        robot_pos = utils_func.get_modelStates_pos(robot_name, simulator_topic_msg)
        return "teste"
    elif config.simulator == config.simulators_dic["unity"]:
        warnings.warn("localization_error function for this simulator not implemented.")
        return None
    else:
        warnings.warn("The simulator defined in config.py file doesn't exist.")
    return None

    
"""                             Possible functions                             """
#def rotation_between(object1: [str,int], object2: [str,int]):
#def velocity_error(object_index: int): #The velocity error between the robot and the gazebo simulation