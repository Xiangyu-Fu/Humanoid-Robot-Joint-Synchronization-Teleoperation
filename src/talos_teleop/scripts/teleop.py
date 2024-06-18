#!/usr/bin/env python3

import rospy
import numpy as np
import pinocchio as pin
import tf

# simulator
import pybullet as pb
from simulator.pybullet_wrapper import PybulletWrapper
from simulator.robot import Robot

# robot configs
from tsid_wrapper import TSIDWrapper
from talos import Talos, Side, other_foot_id
from strategies import AnkleStrategy, HipStrategy

# import markers msg
from visualization_msgs.msg import Marker, MarkerArray

# modules
from foot_trajectory import *
# from footstep_planner import *
# from lip_mpc import *

import talos_conf as conf

from ndcurves import polynomial

class TalosTeleop:
    def __init__(self):
        # setup the simulator
        self.simulator = PybulletWrapper(render=True)
        # setup the robot
        self.robot = Talos(self.simulator)

        # setup some ROS stuff
        self.listener = tf.TransformListener()
        self.marker_pub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10)

        # do some init task
        self.robot.tsid_wrapper.setPostureRef(conf.q_posture)
        self.robot.addHandTask()
        self.traj = SwingFootTrajectory(pin.SE3.Identity(), pin.SE3.Identity(), 0.5)

        # Save some variables
        self.ComInit = self.robot.tsid_wrapper.comState().value()
        self.ComInit[2] = 0.88
        self.robot.tsid_wrapper.setComRefState(self.ComInit)
        self.lifting_foot = None
        self.lifting_foot_dur = 0.0
        self._use_ankle_strategy = False
        self._use_hip_strategy = False
        self._set_left_com = False
        self._set_right_com = False

        # Parameters for the foot tracking
        self.lifting_foot = None
        self.lifting_foot_dur = 0.0
        self._lift_foot = False # to make sure that only remove the contact in tsid once
        self._lifting = False # to make sure that the human is lifting the foot, aviod some noise
        self._prev_lift_foot = None # save the previous lifting foot for put it back to the ground

    def addAnkleStrategy(self):
        self.ankle_strategy = AnkleStrategy()
        self._use_ankle_strategy = True

    def	addHipStrategy(self):
        self.hip_strategy = HipStrategy()
        self._use_hip_strategy = True

    def publish_marker(self, pos, ori=[0,0,0,1]):
        pos_marker = Marker()
        pos_marker.header.frame_id = "world"
        pos_marker.header.stamp = rospy.Time.now()
        pos_marker.ns = "my_namespace"
        pos_marker.type = Marker.SPHERE
        pos_marker.action = Marker.ADD
        pos_marker.pose.position.x = pos[0]
        pos_marker.pose.position.y = pos[1]
        pos_marker.pose.position.z = pos[2]
        pos_marker.pose.orientation.x = ori[0]
        pos_marker.pose.orientation.y = ori[1]
        pos_marker.pose.orientation.z = ori[2]
        pos_marker.pose.orientation.w = ori[3]
        pos_marker.scale.x = 0.05
        pos_marker.scale.y = 0.05
        pos_marker.scale.z = 0.05
        pos_marker.color.a = 1.0
        pos_marker.color.r = 1.0
        marker_array = MarkerArray()
        marker_array.markers.append(pos_marker)
        self.marker_pub.publish(marker_array)

    def checkFootState(self, lf_pose, rf_pose, rosreq, pose_res=[0.1, 0.1, 0.1]):
        # check if there exists a lifting foot
        error = lf_pose - rf_pose
        # print('error:',error)
        if error[0] > pose_res[0] or error[2] > pose_res[2]+0.016:
            self.lifting_foot = Side.LEFT
            self.lifting_foot_dur += 1/rosreq

        elif error [0] < -pose_res[0] or error[2] < -pose_res[2]:
            self.lifting_foot = Side.RIGHT
            self.lifting_foot_dur += 1/rosreq

        else:
            self.lifting_foot = None
            self.lifting_foot_dur = 0.0
            self._lifting == False

    def setLiftingState(self, rosreq):
        if self.lifting_foot_dur > 4 * 1/rosreq:
            self._lifting = True
    
    def liftFoot(self, lf_pose, rf_pose, robot, simulator, pose_res=[0.1, 0.1, 0.1]):
        if self._lifting == True:
            if self.lifting_foot == Side.RIGHT:
                if self._lift_foot == False:
                    print("lifting right foot")
                    robot.setSwingFoot(self.lifting_foot)
                    # save the previous lifting foot
                    self._prev_lift_foot = self.lifting_foot
                    self._lift_foot = True

                    # gen the trajectory to lift the foot
                    rf_lift_init_pose = robot.tsid_wrapper.get_placement_RF().translation
                    self.traj.reset(rf_lift_init_pose, rf_pose)

                    for i in range(500):
                        # set the tsid reference
                        traj_pos, traj_vel, traj_acll = self.traj.evaluate(i/1000)
                        robot.tsid_wrapper.set_RF_pos_ref(traj_pos, traj_vel, traj_acll)
                        simulator.step()
                        robot.update()

                # simulator.addSphereMarker(rf_pose, radius=0.02, color=[0, 0, 1, 1])
                p_RF = robot.tsid_wrapper.get_placement_RF().translation
                p_RF[0] = rf_pose[0]
                p_RF[2] = rf_pose[2]
                robot.tsid_wrapper.set_RF_pos_ref(rf_pose, np.array([0, 0, 0]), np.array([0, 0, 0]))

            elif self.lifting_foot == Side.LEFT:
                if self._lift_foot == False:
                    print("lifting left foot")
                    robot.setSwingFoot(self.lifting_foot)
                    # save the previous lifting foot
                    self._prev_lift_foot = self.lifting_foot
                    self._lift_foot = True

                    # gen the trajectory to lift the foot
                    lf_lift_init_pose = robot.tsid_wrapper.get_placement_LF().translation
                    self.traj.reset(lf_lift_init_pose, lf_pose)

                    for i in range(500):
                        # set the tsid reference
                        traj_pos, traj_vel, traj_acll = self.traj.evaluate(i/1000)
                        robot.tsid_wrapper.set_LF_pos_ref(traj_pos, traj_vel, traj_acll)

                        simulator.step()
                        robot.update()

                # simulator.addSphereMarker(lf_pose, radius=0.02, color=[0, 1, 1, 1])
                p_LF = robot.tsid_wrapper.get_placement_LF().translation
                p_LF[0] = lf_pose[0]
                p_LF[2] = lf_pose[2]
                robot.tsid_wrapper.set_LF_pos_ref(lf_pose, np.array([0, 0, 0]), np.array([0, 0, 0]))
            else:
                self._lifting = False # reset the Human lifting state
            
    def checkRobotLifting(self, robot, simulator, RF_init_pose, LF_init_pose):
        if self._prev_lift_foot is not None and self.lifting_foot is None:
            
            # Detect the robot is lifting the foot, we need to put the foot back to the ground
            robot_lifting = True
            rospy.loginfo("Robot is lifting the foot, put the foot back to the ground")

            loop_count = 0

            # begin to put the foot back to the ground
            while robot_lifting is True:
                if self._prev_lift_foot == Side.RIGHT:
                    # set tsid reference
                    robot.tsid_wrapper.set_RF_pos_ref(RF_init_pose, np.array([0, 0, 0]), np.array([0, 0, 0]))

                    # error check
                    right_foot_current_pose = robot.tsid_wrapper.get_placement_RF().translation
                    if np.linalg.norm(RF_init_pose - right_foot_current_pose) < 0.06:
                        # robot.setSupportFoot(self._prev_lift_foot)
                        self._lift_foot = False
                        self._lifting = False

                        # reset the robot lifting state and break the loop
                        robot_lifting = False
    
                if self._prev_lift_foot == Side.LEFT:
                    # set tsid reference
                    robot.tsid_wrapper.set_LF_pos_ref(LF_init_pose, np.array([0, 0, 0]), np.array([0, 0, 0]))

                    # error check
                    left_foot_current_pose = robot.tsid_wrapper.get_placement_LF().translation
                    if np.linalg.norm(LF_init_pose - left_foot_current_pose) < 0.06:
                        robot.setSupportFoot(self._prev_lift_foot)
                        self._lift_foot = False
                        self._lifting = False

                        # reset the robot lifting state and break the loop
                        robot_lifting = False

                loop_count += 1

                if loop_count > 8:
                    self.robot.tsid_wrapper.setComRefState(self.ComInit)

                # run some steps
                for _ in range(100):
                    simulator.step()
                    robot.update()

            # Add contact to the foot in tsid 
            self.robot.setSupportFoot(self._prev_lift_foot)
            self._prev_lift_foot = None

            for _ in range(100):
                simulator.step()
                robot.update()
        

    def shiftCOM(self, RF_init_pose, LF_init_pose,robot,simulator):
        """
        shift the com reference for lift the foot 
        """
        com_maker = robot.tsid_wrapper.comState().value()
        # simulator.addSphereMarker(com_maker+np.array([0,0,0.85]), radius=0.02, color=[0, 0, 0 , 1])
        if self.lifting_foot == Side.RIGHT:
            if self._set_left_com == False:

                p_com = robot.tsid_wrapper.comState().value()
                p_com[:2] = LF_init_pose[:2]  # change the XY position of COM to the position of the right foot
                p_com[2] = 0.88
                # self.robot.tsid_wrapper.setComRefState(p_com)
                # self._set_left_com = True

                current_com = robot.tsid_wrapper.comState().value()
                current_com[2] = 0.88
                left_com = p_com
                coeffs = np.matrix([current_com, left_com]).T
                com_spline = polynomial(coeffs)
                for i in range(100):
                    com_p = com_spline(i/100)
                    # com_v = com_spline.derivate(i/500, 1)
                    # com_a = com_spline.derivate(i/500, 2)
                    com_p[2] = 0.88
                    robot.tsid_wrapper.setComRefState(com_p)
                    
                    simulator.step()
                    robot.update()   
                self._set_left_com = True

        elif self.lifting_foot == Side.LEFT:
            if self._set_right_com == False:
                p_com = self.robot.tsid_wrapper.comState().value()
                p_com[:2] = RF_init_pose[:2]
                p_com[2] = 0.88
                # self.robot.tsid_wrapper.setComRefState(p_com)
                # self._set_right_com = True


                current_com = robot.tsid_wrapper.comState().value()
                current_com[2] = 0.88
                right_com = p_com
                print('current_com:',current_com)
                print('right_com:',right_com)
                coeffs = np.matrix([current_com, right_com]).T
                com_spline = polynomial(coeffs)
                for i in range(1000):
                    com_p = com_spline(i/1000)
                    # com_v = com_spline.derivate(i/500, 1)
                    # com_a = com_spline.derivate(i/500, 2)
                    com_p[2] = 0.88
                    robot.tsid_wrapper.setComRefState(com_p)
                    # self._set_right_com = True
                    simulator.step()
                    robot.update()

                for i in range(700):
                    if i == 400:
                        p_com = self.robot.tsid_wrapper.comState().value()
                        p_com[:2] = RF_init_pose[:2]
                        p_com[2] = 0.88
                    # self.robot.tsid_wrapper.setComRefState(p_com)
                        robot.tsid_wrapper.setComRefState(p_com)
                    simulator.step()  
                    robot.update()
                # robot.tsid_wrapper.setComRefState(p_com)
                self._set_right_com = True 
        else:
            # print("No foot is lifting, reset the com reference"")
            # robot.tsid_wrapper.setComRefState(self.ComInit)
            self._set_right_com = False
            self._set_left_com = False
            self._lift_foot = False

    def run(self):
        feq = 10
        rate = rospy.Rate(feq) # 10hz
        sim_step = 1000 / feq

        # Get the initial pose of the feet for put the foot back to the ground
        while True:
            try:
                RF_init_pose = self.robot.tsid_wrapper.get_placement_RF().translation
                # RF_init_pose[1] = -0.1
                LF_init_pose = self.robot.tsid_wrapper.get_placement_LF().translation
                # LF_init_pose[1] = 0.1
                break
            except:
                pass

        # Main loop
        while True:
            t = self.simulator.simTime()
            dt = self.simulator.stepTime()

            # HAND TRACKING
            try:
                (trans, rot) = self.listener.lookupTransform('body_ics', 'l_wrist_ics', rospy.Time(0))
                trans[1] += 0.1 
                trans[2] += 0.85
                LH_pos_ref = np.array(trans)
                # self.publish_marker(LH_pos_ref)
                # self.simulator.addSphereMarker(LH_pos_ref, radius=0.02, color=[0, 1, 0, 1])
                self.robot.setLeftHandPose(LH_pos_ref, np.array([0, 0, 0]), np.array([0, 0, 0]))

                (trans, rot) = self.listener.lookupTransform('body_ics', 'r_wrist_ics', rospy.Time(0))
                trans[0] += 0.1
                trans[1] -= 0.1 
                trans[2] += 0.85
                RH_pos_ref = np.array(trans)
                # self.publish_marker(RH_pos_ref)
                # self.debug_point_id = self.simulator.addSphereMarker(RH_pos_ref, radius=0.02)
                self.robot.setRightHandPose(RH_pos_ref, np.array([0, 0, 0]), np.array([0, 0, 0]))

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                # rospy.logwarn("Failed to get transformation between body_ics and l_wrist_ics.")
                pass


            if self._use_ankle_strategy: # BUG: Strategy is not right maybe
                # Compute the desired CoM velocity using the Ankle Strategy
                x_d = self.robot.baseCoMPosition()[:2]
                x_ref = np.array([0, 0])  
                p = self.robot.compute_ZMP(0.1) 
                p_ref = np.array([0, 0])  
                x_ref_dot = np.array([0, 0]) 

                desired_velocity = self.ankle_strategy.compute_desired_velocity(x_d, x_ref, p, p_ref, x_ref_dot)
                desired_velocity = np.append(desired_velocity, 0)  # Assuming the CoM velocity in the z direction is zero
                
                p_com_n = self.tsid_wrapper.comState().value()
                p_com_n[:2] = x_d
                self.tsid_wrapper.setComRefState(p_com_n, desired_velocity)

            # FOOT TRACKING 
            try:
                # get the pose of the left foot 
                (lf_trans, lf_rot) = self.listener.lookupTransform('body_ics', 'l_ankle_ics', rospy.Time(0))
                # outlier rejection
                if lf_trans[2] > -0.3:
                    continue
                lf_trans[2] += 0.85 # height compensation of body_ics frame
                LF_pos_ref = np.array(lf_trans)

                (rf_trans, rf_rot) = self.listener.lookupTransform('body_ics', 'r_ankle_ics', rospy.Time(0))
                # outlier rejection
                if rf_trans[2] > -0.3:
                    continue
                rf_trans[2] += 0.85 # height compensation of body_ics frame
                RF_pos_ref = np.array(rf_trans)

                self.checkFootState(LF_pos_ref, RF_pos_ref, feq)
                self.setLiftingState(feq)
                self.liftFoot(LF_pos_ref, RF_pos_ref, self.robot, self.simulator)
                self.checkRobotLifting(self.robot, self.simulator, RF_init_pose, LF_init_pose)
                self.shiftCOM(RF_init_pose, LF_init_pose,self.robot, self.simulator)
                
                # self.setFootPose(LF_pos_ref, RF_pos_ref)

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logwarn("Failed to get transformation ...  CHECK THE TF TREE")
                pass

            # TODO: add more tasks, e.g. Squatting, walking, etc.

            for _ in range(int(sim_step)):
                self.simulator.step()
                self.robot.update()
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('talos_teleop')
    teleop = TalosTeleop()
    teleop.run()