#!/usr/bin/env python2
from cmath import cos, sin
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import String
from utillib.msg import NaviStatus
from marker_detection_system.srv import marker_detect
from karis_msg.msg import FPPhase, DetectionConfig, PlanningConfig, DrivingConfig, RefineConfig, VerificationConfig, Pose2D, doFineposActionGoal , doFineposActionResult, doFineposGoal, doFineposResult, doFineposActionFeedback, Battery
import tf
from tf.transformations import euler_from_quaternion


class AgvControl:
    def __init__(self, goalPoseTopic, navStatusTopic, finePoseServer, markerService):
        self.name = "Hello world"
        self.marker_service = markerService
        self.goal_pose_pub = rospy.Publisher(goalPoseTopic, PoseStamped, queue_size=10, latch=True)
        self.nav_status_sub = rospy.Subscriber(navStatusTopic, NaviStatus, self.nav_status_callback)
        self.finepos_status_sub = rospy.Subscriber(finePoseServer+"/result", doFineposActionResult, self.finepos_status_callback)
        self.finepos_action_pub = rospy.Publisher(finePoseServer+"/goal", doFineposActionGoal, queue_size=1)
        self.finepos_cmd_vel_pub = rospy.Publisher("/karisagiprobot/splc_fp_velocity_command", TwistStamped, queue_size=1)
        self.nav_status = None
        self.fine_pos_status = -1

    def drive_to_coarse_goal(self, station):
        goal_msg = PoseStamped()
        goal_msg.header.stamp = rospy.Time.now()
        goal_msg.header.frame_id = "map"
        goal_msg.pose.position.x = station["position"][0]
        goal_msg.pose.position.y = station["position"][1]
        goal_msg.pose.position.z = station["position"][2]
        goal_msg.pose.orientation.x = station["orientation"][0]
        goal_msg.pose.orientation.y = station["orientation"][1]
        goal_msg.pose.orientation.z = station["orientation"][2]
        goal_msg.pose.orientation.w = station["orientation"][3]

        self.goal_pose_pub.publish(goal_msg)
        rospy.loginfo("Driving to goal pose of: %s", station["name"])
        rospy.sleep(1)
        success = self.wait_for_coarse_arrival()
        return success

    def wait_for_coarse_arrival(self):
        while self.nav_status == None:
            rospy.sleep(0.2)
        while self.nav_status != 3 and not rospy.is_shutdown():
            # rospy.loginfo("Current navigation status is %i", self.nav_status)
            rospy.sleep(0.2)
        rospy.loginfo("Reached coarse destination! Status is now %i", self.nav_status)
        return True

    def docking(self):
        self.fine_pos_status = -1
        rospy.loginfo("Started docking sequence")
        self.do_docking()
        # for i in range(0,10):
        #     angular_error = self.do_docking()
        #     print(angular_error)
        #     rospy.sleep(0.2)
        #rospy.sleep(1)
        #while self.fine_pos_status != 3 and self.fine_pos_status != 4 and not rospy.is_shutdown():
            #rospy.loginfo("Currently Docking with status %i", self.fine_pos_status)
        #    rospy.sleep(0.1)
        #rospy.loginfo("Stopped Docking with status %i", self.fine_pos_status)
        #self.fine_pos_status = -1
        #rospy.loginfo("Successfully docked at station")
        #pose = self.detect_station_marker()
        #rospy.loginfo("Final orientation error: %f", pose[2])

    def undocking(self):
        self.fine_pos_status = -1
        rospy.loginfo("Started undocking sequence")
        self.do_undocking()
        rospy.sleep(1)
        while self.fine_pos_status != 3 and self.fine_pos_status != 4 and not rospy.is_shutdown():
            # rospy.loginfo("Currently Docking with status %i", self.fine_pos_status)
            rospy.sleep(0.1)
        self.fine_pos_status = -1
        rospy.loginfo("Successfully undocked from station")

    def nav_status_callback(self, msg):
        self.nav_status = msg.status

    def finepos_status_callback(self, msg):
        self.fine_pos_status = msg.status.status
        rospy.loginfo("Received finepose status %s", self.fine_pos_status)

    def detect_station_marker(self):
        rospy.wait_for_service(self.marker_service, 10)
        detect_marker = rospy.ServiceProxy(self.marker_service, marker_detect)
        try:
            res = detect_marker(False)  # send service request and try to get an answer
            if res.success is True:
                ret = (res.trans.x, res.trans.y, res.rot.z, res.success)
                return ret
            else:
                ret = (0, 0, 0, False)
                return ret
        except rospy.ServiceException as exc:
            ret = (0, 0, 0, False)
            rospy.logwarn("Not able to call marker service on: %s", self.marker_service)
            return ret
            #return ret

    def do_docking(self):
        rospy.loginfo("Starting docking process")
        rate = rospy.Rate(2)
        #action und goal msg definieren
        actionmsg = doFineposActionGoal()
        goalmsg = doFineposGoal()

        #phase1 definieren und laden
        phase1 = FPPhase()
        detmsg = DetectionConfig()
        planmsg = PlanningConfig()
        drmsg = DrivingConfig()
        refmsg = RefineConfig()
        vermsg = VerificationConfig()

        detmsg.detector_type = 2
        detmsg.initial_guess = Pose2D(0.3, 0.1, 0.0) # Pose2D(3.41006089751,9.21755156519,0.0539139)
        detmsg.estimation_error = Pose2D(0.3, 0.3, 0.3) # Pose2D(0.3, 0.3, 0.15)
        detmsg.detection_file = 'd_vzug_station'
        detmsg.matching_file = 'm_vzug_station_vorne'
        detmsg.num_detections = 0
        detmsg.laser_source = 0

        #pose = self.detect_station_marker()
        rospy.loginfo("Starting orientation error")
        planmsg.trajectory.append(Pose2D(0.2, 0.0, 0.0))
        planmsg.opti = False
        planmsg.backwards = False
        planmsg.control_mode = 0  # 0 nur drehung oder nur translation

        drmsg.tv_max = 0.05
        drmsg.rv_max = 0.05

        refmsg.matching_file = 'm_vzug'
        refmsg.num_detections = 0
        refmsg.estimation_error = Pose2D(0.0, 0.0, 0.0)
        refmsg.laser_source = 0

        vermsg.check_safety_button = False
        vermsg.check_position = False
        vermsg.num_detections = 0
        vermsg.num_verifications = 0
        vermsg.positioning_error = Pose2D(0.0, 0.0, 0.0)

        phase1.detection = detmsg
        phase1.planning = planmsg
        phase1.driving = drmsg
        phase1.refine = refmsg
        phase1.verify = vermsg

        goalmsg.phases.append(phase1)

        #Phase2 definieren und laden
        # phase2 = FPPhase()
        # drmsg2 = DrivingConfig()
        # planmsg2 = PlanningConfig()
        # planmsg2.trajectory.append(Pose2D(0.05, 0, 0))
        # planmsg2.opti = False
        # planmsg2.backwards = False
        # planmsg2.control_mode = 0  # 0 nur drehung oder nur translatio
        #
        # drmsg2.tv_max = 0.02
        # drmsg2.rv_max = 0.1
        #
        # phase2.detection = detmsg
        # phase2.planning = planmsg2
        # phase2.driving = drmsg2
        # phase2.refine = refmsg
        # phase2.verify = vermsg
        #
        # goalmsg.phases.append(phase2)

        actionmsg.header.seq = 0
        actionmsg.header.stamp = rospy.Time.now()
        actionmsg.header.frame_id = ""
        actionmsg.goal_id.stamp = rospy.Time.now()
        actionmsg.goal_id.id = ""
        actionmsg.goal = goalmsg

        while not rospy.is_shutdown():
            self.finepos_action_pub.publish(actionmsg)
            actionmsg.header.seq = actionmsg.header.seq + 1
            if(actionmsg.header.seq >= 2):
                rospy.loginfo("publish fine pose")
                break
            rate.sleep()
        #return pose[2]

    def do_undocking(self):
        rate = rospy.Rate(2)
        #action und goal msg definieren
        actionmsg = doFineposActionGoal()
        goalmsg = doFineposGoal()

        #phase2 definieren und laden
        phase2 = FPPhase()
        detmsg2 = DetectionConfig()
        planmsg2 = PlanningConfig()
        drmsg2 = DrivingConfig()
        refmsg2 = RefineConfig()
        vermsg2 = VerificationConfig()

        detmsg2.detector_type = 0
        detmsg2.initial_guess = Pose2D(4.3138282116,9.26936589738,0.0)
        detmsg2.estimation_error = Pose2D(0.2, 0.2, 0.2)
        detmsg2.detection_file = 'd_vzug'
        detmsg2.matching_file = 'm_vzug'
        detmsg2.num_detections = 0
        detmsg2.laser_source = 0

        planmsg2.trajectory.append(Pose2D(-0.5, 0.0, 0.0))#-1.5
        planmsg2.opti = False
        planmsg2.backwards = True
        planmsg2.control_mode = 0

        drmsg2.tv_max = 0.25
        drmsg2.rv_max = 0.25

        refmsg2.matching_file = 'm_vzug'
        refmsg2.num_detections = 0
        refmsg2.estimation_error = Pose2D(0.0, 0.0, 0.0)
        refmsg2.laser_source = 0

        vermsg2.check_safety_button = False
        vermsg2.check_position = False
        vermsg2.num_detections = 0
        vermsg2.num_verifications = 0
        vermsg2.positioning_error = Pose2D(0.0, 0.0, 0.0)

        phase2.detection = detmsg2
        phase2.planning = planmsg2
        phase2.driving = drmsg2
        phase2.refine = refmsg2
        phase2.verify = vermsg2

        goalmsg.phases.append(phase2)

        actionmsg.header.seq = 0
        actionmsg.header.stamp = rospy.Time.now()
        actionmsg.header.frame_id = ""
        actionmsg.goal_id.stamp = rospy.Time.now()
        actionmsg.goal_id.id = ""
        actionmsg.goal = goalmsg

        while not rospy.is_shutdown():
            self.finepos_action_pub.publish(actionmsg)
            actionmsg.header.seq = actionmsg.header.seq + 1
            if(actionmsg.header.seq >= 2):
                break
            rate.sleep()


    def get_dock_pose(self, station):
        listener = tf.TransformListener()
        dock_topic = 'station_' + station["docking_pose"]
        bool_get_transform = False
        cout_get_transform = 0
        relativ_pos = [0.0, 0.0, 0.0]
        while bool_get_transform == False and cout_get_transform < 500:
            try: 
                (trans, rot) = listener.lookupTransform('/base_link',dock_topic, rospy.Time(0))
                # rospy.sleep(2)
                # (trans, rot) = listener.lookupTransform('/base_link',dock_topic, rospy.Time(0))
                bool_get_transform = True
            except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.loginfo("could not find the transform %s", dock_topic)
                # rospy.sleep(2)
                cout_get_transform += 1
                continue

            (roll, pitch, yaw) = euler_from_quaternion (rot)
            relativ_pos[0] = trans[0]
            relativ_pos[1] = trans[1]
            relativ_pos[2] = yaw
            rospy.loginfo("cout %d relativ_pos %f, %f, %f", cout_get_transform, relativ_pos[0],relativ_pos[1], relativ_pos[2])
            rospy.loginfo("bool_get_transform %d ", bool_get_transform)
        return bool_get_transform, relativ_pos

    def dock_new(self, fine_pos):
        self.fine_pos_status = -1
        rospy.loginfo("Started docking sequence")
        rate = rospy.Rate(2)
        #action und goal msg definieren
        actionmsg = doFineposActionGoal()
        goalmsg = doFineposGoal()

        #phase1 definieren und laden
        phase1 = FPPhase()
        detmsg = DetectionConfig()
        planmsg = PlanningConfig()
        drmsg = DrivingConfig()
        refmsg = RefineConfig()
        vermsg = VerificationConfig()

        detmsg.detector_type = 2
        detmsg.initial_guess = Pose2D(0.3, 0.1, 0.0) # Pose2D(3.41006089751,9.21755156519,0.0539139)
        detmsg.estimation_error = Pose2D(0.3, 0.3, 0.3) # Pose2D(0.3, 0.3, 0.15)
        detmsg.detection_file = 'd_vzug_station'
        detmsg.matching_file = 'm_vzug_station_vorne'
        detmsg.num_detections = 0
        detmsg.laser_source = 0

        # rospy.loginfo("fine pose %f, %f %f", fine_pos[0], fine_pos[1], fine_pos[2])
        planmsg.trajectory.append(Pose2D(fine_pos[0], fine_pos[1], fine_pos[2]))
        planmsg.opti = False
        planmsg.backwards = False 
        if(fine_pos[0]<0):
            planmsg.backwards = True 
        planmsg.control_mode = 0  # 0 nur drehung oder nur translation

        drmsg.tv_max = 0.05
        drmsg.rv_max = 0.05

        refmsg.matching_file = 'm_vzug'
        refmsg.num_detections = 0
        refmsg.estimation_error = Pose2D(0.0, 0.0, 0.0)
        refmsg.laser_source = 0

        vermsg.check_safety_button = False
        vermsg.check_position = False
        vermsg.num_detections = 0
        vermsg.num_verifications = 0
        vermsg.positioning_error = Pose2D(0.0, 0.0, 0.0)

        phase1.detection = detmsg
        phase1.planning = planmsg
        phase1.driving = drmsg
        phase1.refine = refmsg
        phase1.verify = vermsg

        goalmsg.phases.append(phase1)

        actionmsg.header.seq = 0
        actionmsg.header.stamp = rospy.Time.now()
        actionmsg.header.frame_id = ""
        actionmsg.goal_id.stamp = rospy.Time.now()
        actionmsg.goal_id.id = ""
        actionmsg.goal = goalmsg

        while not rospy.is_shutdown():
            self.finepos_action_pub.publish(actionmsg)
            actionmsg.header.seq = actionmsg.header.seq + 1
            if(actionmsg.header.seq >= 2):
                rospy.loginfo("publish fine pose, repeat")
                break
            rate.sleep()

    def dock_phase(self, station):
        bool_reach_goal = False
        bool_get_transform = False
        nav_pos = [0.0, 0.0, 0.0]
        angle_thre = 1.5/180.0*np.pi  #1.5
        y_thre = 0.02 #2cm
        while bool_reach_goal == False:
            (bool_get_transform, fine_pos) = self.get_dock_pose(station)
            if not bool_get_transform:
                rospy.loginfo("could not find the transform to %s", station["name"])
                break
                # continue
                
            # fistly check distance
            if fine_pos[0]>0.6 or fine_pos[1]>0.6 :
                nav_pos[2] = fine_pos[2]
                nav_pos[0] = fine_pos[0]-0.5*np.cos(fine_pos[2])
                nav_pos[1] = fine_pos[1]-0.5*np.sin(fine_pos[2])
                rospy.loginfo("phase 1: nav_pos: %f, %f, %f", nav_pos[0],nav_pos[1], nav_pos[2])
                self.dock_new(nav_pos)
                bool_get_transform = False
            # then check angle and y 
            elif abs(fine_pos[2]) >= angle_thre or abs(fine_pos[1]) >= y_thre: # angle difference large, move back
                nav_pos[2] = fine_pos[2] 
                nav_pos[0] = fine_pos[0]-1.0*np.cos(nav_pos[2])
                nav_pos[1] = fine_pos[1]-1.0*np.sin(nav_pos[2])
                rospy.loginfo("phase loop: angle_diff: %f, y_diff %f", fine_pos[2]/np.pi*180, fine_pos[1])
                rospy.loginfo("phase loop: nav_pos: %f, %f, %f", nav_pos[0],nav_pos[1], nav_pos[2])
                self.dock_new(nav_pos)
                bool_get_transform = False
            # slowly move forward
            elif fine_pos[0]>=0.48 and fine_pos[0]<=0.53:
                nav_pos[2] = 0
                nav_pos[0] = fine_pos[0]-0.40
                nav_pos[1] = 0
                rospy.loginfo("phase 2: nav_pos: %f, %f, %f", nav_pos[0],nav_pos[1], nav_pos[2])
                self.dock_new(nav_pos)
                bool_get_transform = False
            elif fine_pos[0]<0.48 and fine_pos[1]<0.48:
                bool_reach_goal = True
                rospy.loginfo("reach goal! stop!")
            else:
                rospy.loginfo("unknown error, please check")
            
            while self.fine_pos_status != 3: # status = 3 arriving
                # br.sendTransform((nav_pos[0], nav_pos[1],0), tf.transformations.quaternion_from_euler(0,0,nav_pos[2]), rospy.Time.now(),"goal", "base_link")
                rospy.loginfo("fine_pos_status: %d ",self.fine_pos_status)
                rospy.sleep(3)
                continue

        return bool_reach_goal

    def pub_cmd_vel_directly(self, duration):
        # rospy.Timer(rospy.Duration(1.0/50.0), self.publish)
        cmdvel_msg = TwistStamped()
        cmdvel_msg.twist.linear.x = 0.05
        rate = rospy.Rate(50)
        pause_seconds = rospy.Time.now() + rospy.Duration(duration)
        while rospy.Time.now() < pause_seconds:
            self.finepos_cmd_vel_pub.publish(cmdvel_msg)
            cmdvel_msg.header.seq = cmdvel_msg.header.seq + 1
            # rospy.loginfo("pub")
            rate.sleep()
        rospy.loginfo("finish publish cmd vel")
        
        

if __name__ == '__main__':
    rospy.init_node('dauertest')
    agvcontrol = AgvControl("/karisagiprobot/move_base_simple/goal", "/karisagiprobot/navigation_status", "/karisagiprobot/fine_position_server","marker_detection_srv")
    print(agvcontrol.name)
    rospy.sleep(1)
    agvcontrol.docking()
    rospy.spin()
