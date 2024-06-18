import numpy as np
# import ndcurves as curves # Only if like to use this one, otherwise scipy, numpy, etc...
import pinocchio as pin
import matplotlib.pyplot as plt

class SwingFootTrajectory:
    """SwingFootTrajectory
    Interpolate Foot trajectory between SE3 T0 and T1
    """
    def __init__(self, T0, T1, duration, height=0.05):
        """initialize SwingFootTrajectory

        Args:
            T0 (pin.SE3): Inital foot pose
            T1 (pin.SE3): Final foot pose
            duration (float): step duration
            height (float, optional): setp height. Defaults to 0.05.
        """
        self._height = height
        self._t_elapsed = 0.0
        self._duration = duration
        self.reset(T0, T1)

    def reset(self, T0, T1):
        '''reset back to zero, update poses
        '''
        #>>>>TODO: plan the spline
        self._t_elapsed = 0.0
        try:
            self.T0 = T0.translation
        except:
            self.T0 = T0
        try:
            self.T1 = T1.translation
        except:
            self.T1 = T1

        T = self._duration
        half_T = T/2

        # Build the system of equations
        A_z = np.array([
            [0, 0, 0, 0, 0, 0, 1],
            [T**6, T**5, T**4, T**3, T**2, T, 1],
            [half_T**6, half_T**5, half_T**4, half_T**3, half_T**2, half_T, 1],
            [0, 0, 0, 0, 0, 1, 0],
            [6*T**5, 5*T**4, 4*T**3, 3*T**2, 2*T, 1, 0],
            [0, 0, 0, 0, 2, 0, 0],
            [30*T**4, 20*T**3, 12*T**2, 6*T, 2, 0, 0]
        ])
        B_z = np.array([self.T0[2], self.T1[2], self._height, 0, 0, 0, 0])
        
        # Solve the system of equations
        self.coeff_z = np.linalg.solve(A_z, B_z)

        A_x = np.array([
            [0, 0, 0, 0, 0, 1],
            [T**5, T**4, T**3, T**2, T, 1],
            [0, 0, 0, 0, 1, 0],
            [5*T**4, 4*T**3, 3*T**2, 2*T, 1, 0],
            [0, 0, 0, 2, 0, 0],
            [20*T**3, 12*T**2, 6*T, 2, 0, 0]
        ])

        B_x = np.array([self.T0[0], self.T1[0], 0, 0, 0, 0])

        self.coeff_x = np.linalg.solve(A_x, B_x)

        A_y = np.array([
            [0, 0, 0, 0, 0, 1],
            [T**5, T**4, T**3, T**2, T, 1],
            [0, 0, 0, 0, 1, 0],
            [5*T**4, 4*T**3, 3*T**2, 2*T, 1, 0],
            [0, 0, 0, 2, 0, 0],
            [20*T**3, 12*T**2, 6*T, 2, 0, 0]
        ])

        B_y = np.array([self.T0[1], self.T1[1], 0, 0, 0, 0])

        self.coeff_y = np.linalg.solve(A_y, B_y)

    def isDone(self):
        return self._t_elapsed >= self._duration 
    
    def evaluate(self, t):
        """evaluate at time t
        """
        #>>>>TODO: evaluate the spline at time t, return pose, velocity, acceleration
        # Clamp the input time to the duration
        t = np.clip(t, 0, self._duration)
        # Find position, velocity and acceleration at time t
        pos_z = np.polyval(self.coeff_z, t)
        vel_z = np.polyval(np.polyder(self.coeff_z, 1), t)
        acc_z = np.polyval(np.polyder(self.coeff_z, 2), t)

        pos_x = np.polyval(self.coeff_x, t)
        vel_x = np.polyval(np.polyder(self.coeff_x, 1), t)
        acc_x = np.polyval(np.polyder(self.coeff_x, 2), t)

        pos_y = np.polyval(self.coeff_y, t)
        vel_y = np.polyval(np.polyder(self.coeff_y, 1), t)
        acc_y = np.polyval(np.polyder(self.coeff_y, 2), t)

        pos = np.array([pos_x, pos_y, pos_z])
        vel = np.array([vel_x, vel_y, vel_z])
        acc = np.array([acc_x, acc_y, acc_z])
        # Update elapsed time
        self._t_elapsed = t
        return pos, vel, acc
    
    def plot(self):
        times = np.linspace(0, self._duration, num=500)
        pos, vel, acc = [], [], []
        for t in times:
            p, v, a = self.evaluate(t)
            pos.append(p)
            vel.append(v)
            acc.append(a)

        # plot
        plt.figure()
        plt.subplot(3, 1, 1)
        plt.plot(times, pos, label=['x', 'y', 'z']) # label="position
        plt.legend()
        plt.subplot(3, 1, 2)
        plt.plot(times, vel, label=['x', 'y', 'z'])
        plt.legend()
        plt.subplot(3, 1, 3)
        plt.plot(times, acc, label=['x', 'y', 'z'])
        plt.legend()
        plt.show()

if __name__=="__main__":
    T0 = pin.SE3(np.eye(3), np.array([0, 0, 0])) # type: ignore
    T1 = pin.SE3(np.eye(3), np.array([0.2, 0, 0])) # type: ignore

    #>>>>TODO: plot to make sure everything is correct
    traj = SwingFootTrajectory(T0, T1, 2.0, 0.05)
    times = np.linspace(0, 2.0, num=500)
    pos, vel, acc = [], [], []
    for t in times:
        p, v, a = traj.evaluate(t)
        pos.append(p)
        vel.append(v)
        acc.append(a)

    # plot
    plt.figure()
    plt.subplot(3, 1, 1)
    plt.plot(times, pos, label='position')
    plt.legend()
    plt.subplot(3, 1, 2)
    plt.plot(times, vel, label='velocity')
    plt.legend()
    plt.subplot(3, 1, 3)
    plt.plot(times, acc, label='acceleration')
    plt.legend()
    plt.show()