#!/usr/bin/python
"""
By convention, firmware modules publish sensor data to ROS topics in the
namespace `/sensors` and listen to actuator commands on ROS topics in the
namespace `/actuators`. This is very useful for low level tasks such as
debugging/testing your hardware but not so useful for getting a high level
overview of the environmental conditions of your system. For this, we would
like to use topics namespaced by the ID of the environment on which the piece
of hardware acts (e.g. /environment_1/air_temperature). This module connects
topics so as to ensure that both of these system views work as expected. There
should be exactly one instance of this module in the system
"""
import sys
import time
import rospy
import rosgraph
import rostopic
from openag.cli.config import config as cli_config
from openag.utils import synthesize_firmware_module_info
from openag.models import FirmwareModule, FirmwareModuleType
from openag.db_names import FIRMWARE_MODULE, FIRMWARE_MODULE_TYPE
from couchdb import Server
from std_msgs.msg import Bool, Float32, Float64

from openag_brain import params
from openag_brain.srv import Empty
from openag_brain.util import resolve_message_type

def connect_topics(
    src_topic, dest_topic, src_topic_type, dest_topic_type, multiplier=1,
    threshold=0
):
    rospy.loginfo("Connecting topic {} to topic {}".format(
        src_topic, dest_topic
    ))
    pub = rospy.Publisher(dest_topic, dest_topic_type, queue_size=10)
    def callback(src_item):
        val = src_item.data
        val *= multiplier
        if dest_topic_type == Bool:
            val = (val > threshold)
        dest_item = dest_topic_type(val)
        pub.publish(dest_item)
    sub = rospy.Subscriber(src_topic, src_topic_type, callback)
    return sub, pub

def connect_pid_topics(pid_variable, sensor, actuator):
    # connect sensor's measured to PID's variable state
    src_topic = "/environment_1/measured/{}".format(sensor)
    src_topic_type = Float64
    dest_topic = "/{}".format(pid_variable)
    dest_topic_type = Float64
    connect_topics(
        src_topic, dest_topic, src_topic_type, dest_topic_type,
        multiplier=1,
        threshold=0
    )
    # connect sensor's desired to PID's desired
    src_topic = "/environment_1/desired/{}".format(sensor)
    src_topic_type = Float64
    dest_topic = "/{}/desired".format(pid_variable)
    dest_topic_type = Float64
    connect_topics(
        src_topic, dest_topic, src_topic_type, dest_topic_type,
        multiplier=1,
        threshold=0
    )
    # connect PID's output to actuator's command input
    src_topic = "/{}_cmd".format(pid_variable)
    src_topic_type = Float64
    dest_topic = "/actuators/{}/cmd".format(actuator)
    dest_topic_type = Bool
    connect_topics(
        src_topic, dest_topic, src_topic_type, dest_topic_type,
        multiplier=1,
        threshold=0
    )

if __name__ == '__main__':
    rospy.init_node("pid_connector")
    connect_pid_topics('light_illuminance', 'light_illuminance', 'light_actuator_1')
    rospy.spin()
