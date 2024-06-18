#!/usr/bin/env python3
"""
talos walking simulation
"""

import rospy
import numpy as np
import pinocchio as pin

# simulator
#>>>>TODO: import simulator
import pybullet as pb
from simulator.pybullet_wrapper import PybulletWrapper
from simulator.robot import Robot

# robot configs
#>>>>TODO: Import talos robot
from tsid_wrapper import TSIDWrapper
from talos import Talos

# modules
#>>>>TODO: Import all previous modules
from foot_trajectory import *
from footstep_planner import *
from lip_mpc import *


################################################################################
# main
################################################################################  
    
def main(): 
    
    ############################################################################
    # setup
    ############################################################################
    
    # setup the simulator
    simulator = PybulletWrapper(render=True)
    # setup the robot
    robot = Talos(simulator)

    # inital footsteps  
    T_swing_w = robot.swingFootPose() #>>>>TODO: set intial swing foot pose to left foot
    T_support_w =  robot.supportFootPose() #>>>>TODO: set intial support foot pose to right foot 
    T_swing_w.translation[2] = 0.0
    T_support_w.translation[2] = 0.0    
    
    # setup the plan
    no_steps = 20
    planner = FootStepPlanner(conf)#>>>>TODO: Create the planner
    plan = planner.planLine(T_support_w, Side.RIGHT, no_steps) #>>>>TODO: Create the plan
    #>>>>TODO: Append the two last steps once more the plan so our mpc horizon will never run out 
    # plan = plan[1:]
    plan.extend(plan[-2:])

    # init foot_trajectory
    foot_trajectory = SwingFootTrajectory(T_swing_w, plan[2].pose, conf.step_dur)     
    # foot_trajectory.plot() # for Debug

    # generate reference
    #>>>>TODO: Generate the mpc reference
    no_samples_per_step = 8
    ZMP_ref = generate_zmp_reference(plan, no_samples_per_step) 
    #>>>>TODO: plot the plan
    planner.plot(simulator) 

    # setup the lip models
    mpc = LIPMPC(conf)#>>>>TODO: setup mpc

    # Assume the com is over the first support foot
    x0 = np.array([T_support_w.translation[0], 0.0, 
                   T_support_w.translation[1], 0.0])#>>>>TODO: Build the intial mpc state vector
    interpolator = LIPInterpolator(x0, conf)#>>>>TODO: Create the interpolator and set the inital state
    
    # set the com task over the support foot
    c, c_dot, c_ddot = interpolator.comState()
    #>>>>TODO: Set the COM reference to be over supporting foot 
    robot.setComRefState(c, c_dot, c_ddot)

    ############################################################################
    # logging
    ############################################################################

    pre_dur = 3.0   # Time to wait befor walking should start
    N_pre = int(pre_dur / simulator.stepTime())#>>>>TODO: number of sim steps before walking starts 
    N_sim = int((no_steps * 0.8) / simulator.stepTime())#>>>>TODO: total number of sim steps during walking
    N_mpc = int(N_sim / 100)#>>>>TODO: total number of mpc steps during walking
    no_sim_per_mpc = int(N_sim / N_mpc)
    
    #>>>>TODO: Create vectors to log all the data of the simulation
    # - COM_POS, COM_VEL, COM_ACC (from the planned reference, pinocchio and pybullet)
    # - Angular momentum (from pinocchio)
    # - Left and right foot POS, VEL, ACC (form planned reference, pinocchio) 
    # - ZMP (from planned reference, from estimator )
    # - DCM (from estimtor)
    # - Normal forces in right and left foot (from pybullet ft sensors, from pinocchio)

    TIME = np.nan*np.empty(N_sim)
    COM_PB_POS = np.nan*np.empty((N_sim, 3))
    COM_PB_VEL = np.nan*np.empty((N_sim, 3))
    COM_PIN_POS = np.nan*np.empty((N_sim, 3))
    COM_PIN_VEL = np.nan*np.empty((N_sim, 3))
    COM_PIN_ACC = np.nan*np.empty((N_sim, 3))
    COM_REF_POS = np.nan*np.empty((N_sim, 3))
    COM_REF_VEL = np.nan*np.empty((N_sim, 3))
    COM_REF_ACC = np.nan*np.empty((N_sim, 3))
    MOMENTUM = np.nan*np.empty((N_sim, 3))
    LEFT_FOOT_POS = np.nan*np.empty((N_sim, 3))
    LEFT_FOOT_VEL = np.nan*np.empty((N_sim, 3))
    RIGHT_FOOT_POS = np.nan*np.empty((N_sim, 3))
    RIGHT_FOOT_VEL = np.nan*np.empty((N_sim, 3))
    ZMP_PB = np.nan*np.empty((N_sim, 2))
    DCM_PB = np.nan*np.empty((N_sim, 2))
    ZMP_REF = np.nan*np.empty((N_sim, 2))
    LEFT_FOOT_FORCE = np.nan*np.empty((N_sim, 6))
    RIGHT_FOOT_FORCE = np.nan*np.empty((N_sim, 6))
    LEFT_FOOT_FORCE_PIN = np.nan*np.empty((N_sim, 3))
    RIGHT_FOOT_FORCE_PIN = np.nan*np.empty((N_sim, 3))

    ############################################################################
    # logging
    ############################################################################
    
    k = 0                                               # current MPC index                          
    plan_idx = 1                                        # current index of the step within foot step plan
    t_step_elapsed = 0.0                                # elapsed time within current step (use to evaluate spline)
    t_publish = 0.0                                     # last publish time (last time we published something)

    for i in range(-N_pre, N_sim):
        t = simulator.simTime() #>>>>TODO: simulator time
        dt = simulator.stepTime() #>>>>TODO: simulator dt

        if i < 0:
            robot.tsid_wrapper.setPostureRef(conf.q_posture)
            robot.shiftCoM(robot.support_foot)

        ########################################################################
        # update the mpc very no_sim_per_mpc steps
        ########################################################################
        if i >= 0 and i % no_sim_per_mpc == 0:#>>>>TODO: when to update mpc
            #>>>>TODO: Implement MPC update
            # 1. Get the current LIP state x_k from the interpolator
            x_k = interpolator.x

            # 2. extract ZMP_ref_k
            idx_begin_k =  k 
            idx_terminal_k =  4  * no_samples_per_step + k 
            ZMP_ref_k = ZMP_ref[idx_begin_k:idx_terminal_k]

            # 3. Solve the mpc and get the first control u_k
            u_k = mpc.buildSolveOCP(x_k, ZMP_ref_k, idx_terminal_k)
            # increment mpc counter
            k += 1        

        ########################################################################
        # update the foot spline 
        ########################################################################

        if i >= 0 and i % conf.no_sim_per_step == 0: #>>>>TODO: when to update spline
            #>>>>TODO: Start next step
            # 1. Get the net step location for the swing foot from the plan
            T_next_swing = plan[plan_idx]

            # robot.setSupportFoot(other_foot_id(T_next_swing.side))

            # 2. Set the swing foot of the robot depending on the side of the next step
            robot.setSwingFoot(T_next_swing.side)
            # 4. Get the current location of the swing foot
            T_curr_swing_pose = robot.swingFootPose()
            T_curr_swing_pose.translation[2] = 0.0
            T_next_swing_pose = T_next_swing.pose

            # 5. Plan a foot trajectory between current and the next foot pose
            foot_trajectory.reset(T_curr_swing_pose, T_next_swing_pose)
            # 6. Increment the step counter        
            t_step_elapsed = 0.0
            plan_idx += 1
            
        ########################################################################
        # in every iteration when walking
        ########################################################################       
        if i >= 0:
            # 1. update the foot trajectory with current step time and set teh new pose,
            #  velocity and acceration reference to the swing foot
            T_swing, V_swing, A_swing = foot_trajectory.evaluate(t_step_elapsed)
            robot.updateSwingFootRef(T_swing, V_swing, A_swing)
            # 2. update the interpolator with the latest command u_k computed by the mpc
            interpolator.integrate(u_k)
            # 3.feed the com tasks with the new com reference pos, vel, acc
            c, c_dot, c_ddot = interpolator.comState()
            robot.setComRefState(c, c_dot, c_ddot)
            # 4. increment the elapsed footstep time
            t_step_elapsed += dt

        # ########################################################################
        # # update the simulation
        # ########################################################################

        #>>>>TODO: update the simulator and the robot
        simulator.step()
        robot.update()

        # publish to ros
        if t - t_publish > 1./30.:
            t_publish = t
            #>>>>TODO: publish
            T_b_w = robot.robot.baseCoMPosition()
            robot.publish(T_b_w)
            
        # store for visualizations
        # if i >= 0:
        #     TIME[i] = t
        #     LF_2d = robot.tsid_wrapper.get_LF_3d_pos_vel_acc()
        #     RF_2d = robot.tsid_wrapper.get_RF_3d_pos_vel_acc()

        #     #>>>>TODO: log information
        #     RIGHT_FOOT_POS[i] = RF_2d[0]
        #     RIGHT_FOOT_VEL[i] = RF_2d[1]
        #     # RIGHT_FOOT_ACC[i] = RF_3d[2]
        #     LEFT_FOOT_POS[i] = LF_2d[0]
        #     LEFT_FOOT_VEL[i] = LF_2d[1]
        #     # LEFT_FOOT_ACC[i] = LF_3d[2]
        #     COM_PIN_POS[i] = robot.tsid_wrapper.comState().value()
        #     COM_PIN_VEL[i] = robot.tsid_wrapper.comState().derivative()
        #     COM_PIN_ACC[i] = robot.tsid_wrapper.comState().second_derivative()

        #     COM_PB_POS[i] = robot.robot.baseWorldPosition()
        #     COM_PB_VEL[i] = robot.robot.baseWorldVelocity()[:3]

        #     COM_REF_POS[i] = c
        #     COM_REF_VEL[i] = c_dot
        #     COM_REF_ACC[i] = c_ddot
            
        #     ZMP_PB[i] = robot.zmp
        #     DCM_PB[i] = robot.dcm

        #     ZMP_REF[i] = ZMP_ref_k[0]


            # LEFT_FOOT_FORCE[i] = robot.left_foot_force
            # RIGHT_FOOT_FORCE[i] = robot.right_foot_force

    ########################################################################
    # enough with the simulation, lets plot
    ########################################################################
    
    # import matplotlib.pyplot as plt
    # plt.style.use('seaborn-dark')

    # plt.figure()
    # plt.subplot(3, 1, 1)
    # plt.title("COM")
    # plt.plot(TIME, COM_PIN_POS[:, 0], label='COM PIN 0')
    # plt.plot(TIME, COM_PB_POS[:, 0], label='COM PB 0')
    # plt.plot(TIME, COM_REF_POS[:, 0], label='COM REF 0', linestyle='dotted', color='black')
    # plt.legend()

    # plt.subplot(3, 1, 2)
    # plt.plot(TIME, COM_PIN_POS[:, 1], label='COM PIN 1')
    # plt.plot(TIME, COM_PB_POS[:, 1], label='COM PB 1')
    # plt.plot(TIME, COM_REF_POS[:, 1], label='COM REF 1', linestyle='dotted', color='black')
    # plt.legend()

    # plt.subplot(3, 1, 3)
    # plt.plot(TIME, COM_PIN_POS[:, 2], label='COM PIN 2')
    # plt.plot(TIME, COM_PB_POS[:, 2], label='COM PB 2')
    # plt.plot(TIME, COM_REF_POS[:, 2], label='COM REF 2', linestyle='dotted', color='black')
    # plt.legend()

    # plt.figure()
    # plt.subplot(3, 1, 1)
    # plt.title("COM VEL")
    # plt.plot(TIME, COM_PIN_VEL[:, 0], label='COM PIN 0')
    # plt.plot(TIME, COM_PB_VEL[:, 0], label='COM PB 0')
    # plt.plot(TIME, COM_REF_VEL[:, 0], label='COM REF 0', linestyle='dotted', color='black')
    # plt.legend()

    # plt.subplot(3, 1, 2)
    # plt.plot(TIME, COM_PIN_VEL[:, 1], label='COM PIN 1')
    # plt.plot(TIME, COM_PB_VEL[:, 1], label='COM PB 1')
    # plt.plot(TIME, COM_REF_VEL[:, 1], label='COM REF 1', linestyle='dotted', color='black')
    # plt.legend()

    # plt.subplot(3, 1, 3)
    # plt.plot(TIME, COM_PIN_VEL[:, 2], label='COM PIN 2')
    # plt.plot(TIME, COM_PB_VEL[:, 2], label='COM PB 2')
    # plt.plot(TIME, COM_REF_VEL[:, 2], label='COM REF 2', linestyle='dotted', color='black')
    # plt.legend()

    # plt.figure()

    # plt.subplot(3, 1, 1)
    # plt.title("COM ACC")
    # plt.plot(TIME, COM_PIN_ACC[:, 0], label='COM PIN 0')
    # plt.plot(TIME, COM_REF_ACC[:, 0], label='COM REF 0', linestyle='dotted', color='black')
    # plt.legend()
  
    # plt.subplot(3, 1, 2)
    # plt.plot(TIME, COM_PIN_ACC[:, 1], label='COM PIN 1')
    # plt.plot(TIME, COM_REF_ACC[:, 1], label='COM REF 1', linestyle='dotted', color='black')
    # plt.legend()

    # plt.subplot(3, 1, 3)
    # plt.plot(TIME, COM_PIN_ACC[:, 2], label='COM PIN 2')
    # plt.plot(TIME, COM_REF_ACC[:, 2], label='COM REF 2', linestyle='dotted', color='black')
    # plt.legend()

    # plt.figure()
    # plt.title("ZMP")
    # plt.subplot(2, 1, 1)
    # plt.plot(TIME, ZMP_PB[:, 0], label='ZMP PB 0')
    # plt.plot(TIME, ZMP_REF[:, 0], label='ZMP REF 0', linestyle='dotted', color='black')
    # plt.legend()

    # plt.subplot(2, 1, 2)
    # plt.plot(TIME, ZMP_PB[:, 1], label='ZMP PB 1')
    # plt.plot(TIME, ZMP_REF[:, 1], label='ZMP REF 1', linestyle='dotted', color='black')
    # plt.legend()

    # plt.show()

if __name__ == '__main__': 
    rospy.init_node('talos_walking')
    main()