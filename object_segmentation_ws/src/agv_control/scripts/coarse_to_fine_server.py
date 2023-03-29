#!/usr/bin/env python2
import numpy as np
import rospy
import geometry_msgs.msg
import tf
from tf.transformations import euler_from_quaternion
from agv_control import AgvControl
import actionlib
import karispro_msgs.msg

def load_station_parameters():
    try:
        station_dict = rospy.get_param('/station_locations')
        return station_dict
    except KeyError:
        rospy.logwarn("Parameters not sufficiently set! This may cause the node to not working properly")

class Coarse_to_Fine_Server():
    def __init__(self):
        self.station_dict = load_station_parameters()
        # agvcontrol = AgvControl("/karisagiprobot/move_base_simple/goal", "/karisagiprobot/navigation_status", "/karisagiprobot/fine_position_server","marker_detection_srv")
        self.fine_navigation_server = actionlib.SimpleActionServer(str("/karisagiprobot/coarse_to_fine_nav"), karispro_msgs.msg.FinePosNavAction, self.move, auto_start = False)
        self.fine_navigation_server.start()
        print("fine_navigation_server started")
        self.agvcontrol = AgvControl("/karisagiprobot/move_base_simple/goal", "/karisagiprobot/navigation_status", "/karisagiprobot/fine_position_server","marker_detection_srv")
        print("agvcontrol started")

    def move(self,data):
        # create messages that are used to publish feedback/result
        feedback = karispro_msgs.msg.FinePosNavFeedback()
        result = karispro_msgs.msg.FinePosNavResult()
        data_key = data.station_name
        if data_key in self.station_dict:
            station_goal = self.station_dict[data_key]
            print("station_goal", type(station_goal), station_goal)
            
            self.agvcontrol.undocking()
            feedback.state = "coarse navigation"
            self.fine_navigation_server.publish_feedback(feedback)
            coarse_success = self.agvcontrol.drive_to_coarse_goal(station_goal)
            while coarse_success is not True:
                rospy.loginfo("wait for global navigation")
            rospy.sleep(2)

            feedback.state = "fine navigation"
            self.fine_navigation_server.publish_feedback(feedback)
            fine_success = self.agvcontrol.dock_phase(station_goal)
            if fine_success is not True:
                rospy.loginfo("could not find fine pose")
                rospy.sleep(3)

            feedback.state = "dock navigation"
            self.fine_navigation_server.publish_feedback(feedback) 
            self.agvcontrol.pub_cmd_vel_directly(1)
            rospy.sleep(5)

            result.success = True
        else:
            print("receive unknown station goal: ", data_key)
            feedback.state = "could not find station"
            self.fine_navigation_server.publish_feedback(feedback) 
            result.success = False

        return self.fine_navigation_server.set_succeeded(result)


if __name__ == '__main__':
    rospy.init_node('fine_poistion_server')
    server = Coarse_to_Fine_Server()
    rospy.spin()
    
