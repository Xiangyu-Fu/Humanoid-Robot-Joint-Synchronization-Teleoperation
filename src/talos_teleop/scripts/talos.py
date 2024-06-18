import numpy as np
import pinocchio as pin
from enum import Enum

# simulator
import pybullet as pb
from simulator.robot import Robot

# whole-body controller
from tsid_wrapper import TSIDWrapper

# robot configs
import talos_conf as conf

# from footstep_planner import Side, other_foot_id

# ROS visualizations
import rospy
import tf
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped, TransformStamped
from visualization_msgs.msg import MarkerArray, Marker


class Side(Enum):
    """Side
    Describes which foot to use
    """
    LEFT=0
    RIGHT=1

def other_foot_id(id):
    if id == Side.LEFT:
        return Side.RIGHT
    else:
        return Side.LEFT

class Talos:
    """Talos robot
    combines wbc with pybullet, functions to read and set
    sensor values.
    """
    def __init__(self, simulator):
        self.conf = conf
        self.sim = simulator
        
        # Like allways create the tsid wrapper
        self.tsid_wrapper = TSIDWrapper(conf)
        
        # spawn robot in simulation
        # Create the pybullet robot for the simulation
        self.robot = Robot( simulator,
                            conf.urdf,
                            self.tsid_wrapper.model,
                            basePosition=[0, 0, 1.1],  # type: ignore
                            baseQuationerion=[0, 0, 0, 1], # type: ignore
                            q=conf.q_home,
                            verbose=True,
                            useFixedBase=False)

        self.data = self.robot._model.createData()
        self.model = self.tsid_wrapper.model
        
        ########################################################################
        # state
        ########################################################################
        # self.support_foot = Side.RIGHT
        # self.swing_foot = Side.LEFT

        self.init_LF_pos = self.tsid_wrapper.get_placement_LF()
        self.init_RF_pos = self.tsid_wrapper.get_placement_RF()
        
        ########################################################################
        # estimators
        ########################################################################
        self.zmp = self.compute_ZMP()
        self.dcm = self.compute_dcm()
        
        ########################################################################
        # sensors
        ########################################################################
        # ft sensors
        # Turn on the force torque sensor in the robots feet
        self.robot.enableFootForceTorqueSensors() # in pybullet

        ########################################################################
        # visualizations
        ########################################################################
        
        # joint state publisher
        self.joint_state_publisher = rospy.Publisher("/joint_states", JointState, queue_size=1)
        # floating base broadcaster 
        self.base_broadcaster = tf.TransformBroadcaster()
        # zmp and dcm point publisher 
        #>>>> Hint: Use visualization_msgs::MarkerArray, SPHERE to visualize 
        self.dcm_publisher = rospy.Publisher("/dcm", MarkerArray, queue_size=1)
        self.zmp_publisher = rospy.Publisher("/zmp", MarkerArray, queue_size=1)

        # wrench publisher for left and right foot
        #>>>> Hint: use geometry_msgs::Wrench
        self.wrench_publisher_right = rospy.Publisher("/wrench_right", WrenchStamped, queue_size=1)
        self.wrench_publisher_left = rospy.Publisher("/wrench_left", WrenchStamped, queue_size=1)

           
    def update(self):
        """updates the robot
        """
        t = self.sim.simTime()
        dt = self.sim.stepTime()

        # update the pybullet robot
        self.robot.update()

        # update the estimators
        self._update_zmp_estimate()
        self._update_dcm_estimate()
        
        # update wbc and send back to pybullet
        self._solve(t, dt)

    ########################################################################
    # Foot tasks
    ########################################################################    
        
    def setSupportFoot(self, side):
        """sets the the support foot of the robot on given side
        """
        
        # The support foot is in rigid contact with the ground and should 
        # hold the weight of the robot
        self.support_foot = side
        print("Set support foot to ", side)
        # self.swing_foot = other_foot_id(side)
        
        # Activate the foot contact on the support foot
        # At the same time deactivate the motion task on the support foot # BUG
        if self.support_foot == Side.LEFT:
            print("Set support foot to LEFT")
            # self.tsid_wrapper.remove_motion_LF()
            self.tsid_wrapper.add_contact_LF()
        else:
            print("Set support foot to RIGHT")
            # self.tsid_wrapper.remove_motion_RF()
            self.tsid_wrapper.add_contact_RF()

    def setSwingFoot(self, side):
        """sets the swing foot of the robot on given side
        """
        
        # The swing foot is not in contact and can move
        self.swing_foot = side
        print("Set swing foot to ", self.swing_foot)
        
        # Deactivate the foot contact on the swing foot
        # At the same time turn on the motion task on the swing foot
        if self.swing_foot == Side.LEFT:
            self.tsid_wrapper.add_contact_RF()
            self.tsid_wrapper.remove_contact_LF()
            # self.tsid_wrapper.add_motion_LF()
        elif self.swing_foot == Side.RIGHT:
            self.tsid_wrapper.remove_contact_RF()
            self.tsid_wrapper.add_contact_LF()
            # self.tsid_wrapper.add_motion_RF()

    def updateSwingFootRef(self, T_swing_w, V_swing_w, A_swing_w):
        """updates the swing foot motion reference
        """
        
        # set the pose, velocity and acceleration on the swing foots
        # motion task
        if self.swing_foot == Side.LEFT:
            self.tsid_wrapper.set_LF_pos_ref(T_swing_w, V_swing_w, A_swing_w)

        elif self.swing_foot == Side.RIGHT:
            self.tsid_wrapper.set_RF_pos_ref(T_swing_w, V_swing_w, A_swing_w)

    def swingFootPose(self):
        """return the pose of the current swing foot
        """
        # return correct foot pose
        if self.swing_foot == Side.LEFT:
            return self.tsid_wrapper.get_placement_LF()
        elif self.swing_foot == Side.RIGHT:
            return self.tsid_wrapper.get_placement_RF()
    
    def supportFootPose(self):
        """return the pose of the current support foot
        """
        # return correct foot pose
        if self.support_foot == Side.LEFT:
            return self.tsid_wrapper.get_placement_LF()
        elif self.support_foot == Side.RIGHT:
            return self.tsid_wrapper.get_placement_RF()
        
    ########################################################################
    # Hand tasks
    ########################################################################        
    def addHandTask(self):
        """adds a hand task
        """
        self.tsid_wrapper.add_motion_LH()
        self.tsid_wrapper.add_motion_RH()

    def removeHandTask(self):
        """removes the hand task
        """
        self.tsid_wrapper.remove_motion_LH()
        self.tsid_wrapper.remove_motion_RH()

    def setLeftHandPose(self, T_lh_w, V_lh_w=None, A_lh_w=None):
        """sets the left hand pose
        """
        self.tsid_wrapper.set_LH_pos_ref(T_lh_w, V_lh_w, A_lh_w)

    def setRightHandPose(self, T_rh_w, V_rh_w=None, A_rh_w=None):
        """sets the right hand pose
        """
        self.tsid_wrapper.set_RH_pos_ref(T_rh_w, V_rh_w, A_rh_w)

    def getLeftHandPose(self):
        """returns the left hand pose
        """
        return self.tsid_wrapper.get_pose_LH()
    
    def getRightHandPose(self):
        """returns the right hand pose
        """
        return self.tsid_wrapper.get_pose_RH()
    
        
    ########################################################################
    # Publisher tasks
    ########################################################################    
    def remap_names(self, q):
        res = np.hstack([q[18:23], q[23:30], q[0:2], q[4:11], q[11:18], q[2:4]])
        return res
    
    def publish_tf(self, T_b_w):
        # Assuming the orientation is identity because it's not provided
        rotation = tf.transformations.quaternion_from_euler(0, 0, 0)
        
        # Send the transformation
        self.base_broadcaster.sendTransform(T_b_w, rotation, rospy.Time.now(), "base_link", "world")


    
    def publish(self, T_b_w):
        """publishes the robot state to ROS
        args:
            T_b_w (np.array): the pose of the robot base in world frame
        """
        # TODO publish jointstate
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = self.remap_names(self.robot.actuatedJointNames())
        msg.position = self.robot.q()[7:]
        msg.velocity = self.robot.v()[7:]
        self.joint_state_publisher.publish(msg)

        # broadcast odometry， publish T_b_w to tf
        self.publish_tf(T_b_w)
        
        # publish feet wrenches
        wrench_right, wrench_left = self.robot.readForce()
        wrench_msg_right = WrenchStamped()
        wrench_msg_right.header.stamp = rospy.Time.now()
        wrench_msg_right.header.frame_id = "leg_right_6_link"
        wrench_msg_right.wrench.force.x = wrench_right.linear[0]
        wrench_msg_right.wrench.force.y = wrench_right.linear[1]
        wrench_msg_right.wrench.force.z = wrench_right.linear[2]
        wrench_msg_right.wrench.torque.x = wrench_right.angular[0]
        wrench_msg_right.wrench.torque.y = wrench_right.angular[1]
        wrench_msg_right.wrench.torque.z = wrench_right.angular[2]
        self.wrench_publisher_right.publish(wrench_msg_right)

        wrench_msg_left = WrenchStamped()
        wrench_msg_left.header.stamp = rospy.Time.now()
        wrench_msg_left.header.frame_id = "leg_left_6_link"
        wrench_msg_left.wrench.force.x = wrench_left.linear[0]
        wrench_msg_left.wrench.force.y = wrench_left.linear[1]
        wrench_msg_left.wrench.force.z = wrench_left.linear[2]
        wrench_msg_left.wrench.torque.x = wrench_left.angular[0]
        wrench_msg_left.wrench.torque.y = wrench_left.angular[1]
        wrench_msg_left.wrench.torque.z = wrench_left.angular[2]
        self.wrench_publisher_left.publish(wrench_msg_left)

        # publish dcm and zmp marker
        marker_array = MarkerArray()
        dcm_marker = Marker()
        dcm_marker.header.frame_id = "world"
        dcm_marker.type = dcm_marker.SPHERE
        dcm_marker.action = dcm_marker.ADD
        dcm_marker.scale.x = 0.1
        dcm_marker.scale.y = 0.1
        dcm_marker.scale.z = 0.1
        dcm_marker.color.a = 1.0
        dcm_marker.color.r = 1.0
        dcm_marker.pose.position.x = self.dcm[0]
        dcm_marker.pose.position.y = self.dcm[1]
        dcm_marker.pose.orientation.w = 1.0
        marker_array.markers.append(dcm_marker)

        self.dcm_publisher.publish(marker_array)

        marker_array = MarkerArray()
        zmp_marker = Marker()
        zmp_marker.header.frame_id = "world"
        zmp_marker.type = zmp_marker.SPHERE
        zmp_marker.action = zmp_marker.ADD
        zmp_marker.scale.x = 0.1
        zmp_marker.scale.y = 0.1
        zmp_marker.scale.z = 0.1
        zmp_marker.color.a = 1.0
        zmp_marker.color.g = 1.0
        zmp_marker.pose.position.x = self.zmp[0]
        zmp_marker.pose.position.y = self.zmp[1]
        zmp_marker.pose.orientation.w = 1.0
        marker_array.markers.append(zmp_marker)

        self.zmp_publisher.publish(marker_array)

    def setComRefState(self, pos, vel=None, acc=None):
        # limit the position of the COM
        self.tsid_wrapper.setComRefState(pos, vel, acc)

    def shiftCoM(self, support_foot):
        if support_foot == Side.RIGHT:
            p_RF = self.tsid_wrapper.get_placement_RF().translation
            p_com = self.tsid_wrapper.comState().value()
            p_com[:2] = p_RF[:2]  # change the XY position of COM to the position of the right foot
            self.tsid_wrapper.setComRefState(p_com)
        else:
            p_LF = self.tsid_wrapper.get_placement_LF().translation
            p_com = self.tsid_wrapper.comState().value()
            p_com[:2] = p_LF[:2]
            self.tsid_wrapper.setComRefState(p_com)
    
    
    ############################################################################
    # private funcitons
    ############################################################################

    def _solve(self, t, dt):
        # get the current state
        q = self.robot.q()
        v = self.robot.v()
        
        # solve the whole body qp
        # sovle the wbc and command the torque to pybullet robot
        self.tau_sol, dq_sol = self.tsid_wrapper.update(q, v, t)
        self.robot.setActuatedJointTorques(self.tau_sol)

    def _update_zmp_estimate(self):
        """update the estimated zmp position
        """
        # compute the zmp based on force torque sensor readings
        self.zmp = self.compute_ZMP()  
        
    def _update_dcm_estimate(self):
        """update the estimated dcm position
        """
        # compute the com based on current center of mass state
        self.dcm = self.compute_dcm()

    def compute_ZMP(self, d=0.1):
        wr_rankle, wl_lankle = self.robot.readForce()

        self.right_foot_force = wr_rankle
        self.left_foot_force = wl_lankle

        # Extract the forces and torques from the sensor data
        fx_right, fy_right, fz_right = wr_rankle.linear[0], wr_rankle.linear[1], wr_rankle.linear[2]
        tau_x_right, tau_y_right, tau_z_right = wr_rankle.angular[0], wr_rankle.angular[1], wr_rankle.angular[2]
        
        fx_left, fy_left, fz_left = wl_lankle.linear[0], wl_lankle.linear[1], wl_lankle.linear[2]
        tau_x_left, tau_y_left, tau_z_left = wl_lankle.angular[0], wl_lankle.angular[1], wl_lankle.angular[2]
        
        # Compute the ZMP for the right and left feet
        px_right = (-tau_y_right - fx_right * d) / fz_right
        py_right = (tau_x_right - fy_right * d) / fz_right
        
        px_left = (-tau_y_left - fx_left * d) / fz_left
        py_left = (tau_x_left - fy_left * d) / fz_left
        
        # For the case of double support (the robot standing on two flat feet over flat ground),
        # the ZMP can be computed from the ZMP of each foot
        px = (px_right * fz_right + px_left * fz_left) / (fz_right + fz_left)
        py = (py_right * fz_right + py_left * fz_left) / (fz_right + fz_left)

        zmp = np.array([px, py])
        return zmp
    
    def compute_cmp(self, f_w=[0, 0, 0], g=conf.g):
        # calculate the mg
        F_gr = np.array([0, 0, self.robot.mass() * g]) + f_w
        # calculate the CoM position
        com_pos = pin.centerOfMass(self.model, self.data, self.robot.q(), self.robot.v()) # type: ignore
        # calculate the CMP
        cmp_pos = com_pos[:2] - (F_gr[:2] / F_gr[2]) * com_pos[2]

        # return the CMP position
        return cmp_pos
    
    def compute_dcm(self, g=9.81):
        # calculate the CoM position and velocity
        com_pos = self.robot.baseCoMPosition()
        com_vel = self.robot.baseCoMVelocity()

        # calculate omega
        omega = np.sqrt(g / com_pos[2])

        # calculate the DCM
        dcm = com_pos[:2] + com_vel[:2] / omega

        # return the DCM
        return dcm
    

