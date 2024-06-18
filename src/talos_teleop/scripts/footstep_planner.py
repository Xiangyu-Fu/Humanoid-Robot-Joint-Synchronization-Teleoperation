import numpy as np
import pinocchio as pin
from enum import Enum
import pybullet as p
import talos_conf as conf

# simulator
from simulator.pybullet_wrapper import PybulletWrapper
from simulator.robot import Robot

import matplotlib.pyplot as plt


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
        
class FootStep:
    """FootStep
    Holds all information describing a single footstep
    """
    def __init__(self, pose, footprint, side=Side.LEFT):
        """inti FootStep

        Args:
            pose (pin.SE3): the pose of the footstep
            footprint (np.array): 3 by n matrix of foot vertices
            side (_type_, optional): Foot identifier. Defaults to Side.LEFT.
        """
        self.pose = pose
        self.footprint = footprint
        self.side = side
        
    def poseInWorld(self):
        return self.pose
        
    def plot(self, simulation):
        
        #>>>>TODO: plot in pybullet footprint, addGlobalDebugRectancle(...) 
        
        #>>>>TODO: display the side of the step, addUserDebugText(...)
        
        #>>>>TODO: plot step target position addSphereMarker(...)

        # calculate the corners of the footprint in world coordinates
        footprint_world = self.pose.rotation @ self.footprint + self.pose.translation.reshape(3,1)

        # assuming pybullet is used for visualization
        simulation.addGlobalDebugRectancle(self.pose.translation)
        simulation.addUserDebugText(0, 0, self.side.name, self.pose.translation)  
        simulation.addSphereMarker(self.pose.translation, radius=0.01)  # green sphere for the step position
        
        return None

class FootStepPlanner:
    """FootStepPlanner
    Creates footstep plans (list of right and left steps)
    """
    
    def __init__(self, conf):
        self.conf = conf
        self.steps = []
        
    def planLine(self, T_0_w, side, no_steps):
        """plan a sequence of steps in a strait line

        Args:
            T_0_w (pin.SE3): The inital starting position of the plan
            side (Side): The intial foot for starting the plan
            no_steps (_type_): The number of steps to take

        Returns:
            list: sequence of steps
        """
        
        # the displacement between steps in x and y direction
        dx = self.conf.step_size_x
        dy = 2 * self.conf.step_size_y
        
        # the footprint of the robot
        lfxp, lfxn = self.conf.lfxp, -self.conf.lfxn
        lfyp, lfyn = self.conf.lfyp, -self.conf.lfyn

        steps=[]
        
        #>>>>TODO: Plan a sequence of steps with T_0_w being the first step pose.
        # Step 0: start with the initial pose
        footprint = np.array([[lfxp, lfxp, lfxn, lfxn],
                              [lfyp, lfyn, lfyn, lfyp],
                              [   0,    0,    0,    0]])
        pose = pin.SE3(T_0_w.rotation, T_0_w.translation + np.array([0, 0, 0]))
        steps.append(FootStep(pose, footprint, side))


        # Step 1: plan the first step parallel to the initial step (robot starts standing on both feet)
        side = other_foot_id(side)
        footprint = np.array([[lfxp, lfxp, lfxn, lfxn],
                              [lfyp, lfyn, lfyn, lfyp],
                              [   0,    0,    0,    0]])
        pose = pin.SE3(T_0_w.rotation, T_0_w.translation + np.array([0, dy, 0]))
        steps.append(FootStep(pose, footprint, side))


        #>>>>Note: Plan the second step parallel to the first step (robot starts standing on both feet)
        for i in range(2, no_steps):
            # alternate steps between left and right
            side = other_foot_id(side)
            footprint = np.array([[lfxp, lfxp, lfxn, lfxn],
                                  [lfyp, lfyn, lfyn, lfyp],
                                  [   0,    0,    0,    0]])  # assuming foot is flat on the ground
            pose = pin.SE3(T_0_w.rotation, pose.translation + np.array([dx, dy*(-1)**(i-1), 0])) # type: ignore
            steps.append(FootStep(pose, footprint, side))

        #>>>>Note: Plan the final step parallel to the last-1 step (robot stops standing on both feet)            
        side = other_foot_id(side)
        footprint = np.array([[lfxp, lfxp, lfxn, lfxn],
                                [lfyp, lfyn, lfyn, lfyp],
                                [   0,    0,    0,    0]])
        pose = pin.SE3(pose.rotation, pose.translation + np.array([0, dy*(-1)**(i), 0]))  # type: ignore
        steps.append(FootStep(pose, footprint, side))
        
        self.steps = steps
        return steps

    
    def plot(self, simulation):
        # plt.figure()
        # plt.title("Footstep plan")
        # plt.xlabel("x [m]")
        # plt.ylabel("y [m]")
        # plt.grid(True)

        for step in self.steps:
            # plot the step
            step.plot(simulation)
            # plt.plot(step.pose.translation[0], step.pose.translation[1], 'x')
            # plot the step target position
            # plt.plot(step.pose.translation[0], step.pose.translation[1], 'o')
        # plt.show()

            
if __name__=='__main__':
    """ Test footstep planner
    """
    
    #>>>>TODO: Generate a plan and plot it in pybullet.
    #>>>>TODO: Check that the plan looks as expected

    simulator = PybulletWrapper(render=True)
    planner = FootStepPlanner(conf)
    T_0_w = pin.SE3(np.eye(3), np.array([0, 0, 0]))
    steps = planner.planLine(T_0_w, Side.RIGHT, 10)
    planner.plot(simulator)
    input("Press Enter to continue...")


    # p.disconnect()