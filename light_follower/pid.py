import math
import numpy as np
from simple_pid import PID

class Config():
    def __init__(self):
        # robot parameter
        self.max_speed = 0.23  # [m/s]
        self.min_speed = 0.0  # [m/s]
        self.max_yawrate = 40.0 * math.pi / 180.0  # [rad/s]
        self.max_accel = 0.1  # [m/ss]
        self.max_dyawrate = 20.0 * math.pi / 180.0  # [rad/ss]
        self.v_reso = 0.01  # [m/s]
        self.yawrate_reso = 0.1 * math.pi / 180.0  # [rad/s]
        self.dt = 0.1  # [s]
        self.predict_time = 3.0  # [s]
        self.to_goal_cost_gain = 1.0 # 1.0
        self.speed_cost_gain = 1.0 # 1.0
        self.robot_radius = 0.1  # [m]

        # pid param
        self.angle_kp = 1.0
        self.angle_ki = 0.3
        self.angle_kd = 0.05
        self.angle_limit = 0.5# max 2.8

        self.distance_kp = 0.1
        self.distance_ki = 0.01
        self.distance_kd = 0.0
        self.distance_limit = 2.3

class pid_controller():
    def __init__(self, NUM_ROBOT=4, GOAL=np.array([0, 0]), X=np.array([0.0, 0.0, math.pi / 8.0, 0.0, 0.0])):
        self.num_robots = NUM_ROBOT
        self.config = Config()

        # initial state [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
        self.x = X
        # goal position [x(m), y(m)]
        self.goal = GOAL
        # obstacles [x(m) y(m), ....]
        # self.ob = np.matrix(np.arange(0., (self.num_robots-1)*2.).reshape((self.num_robots-1), 2))
        # control u [linear.x, angular.z]
        self.u = np.array([0.0, 0.0])

        self.pid_angle = PID(self.config.angle_kp, self.config.angle_ki, self.config.angle_kp, setpoint = 0)
        self.pid_angle.output_limits = (-self.config.angle_limit, self.config.angle_limit)
        self.pid_angle.set_auto_mode(enabled=True)

        self.pid_distance = PID(self.config.distance_kp, self.config.distance_ki, self.config.distance_kp, setpoint = 0)
        self.pid_distance.output_limits = (-self.config.distance_limit, self.config.distance_limit)
        self.pid_distance.set_auto_mode(enabled=True)

    def loop(self):
        # print(self.x)
        # self.u, ltraj = self.dwa_control(self.x, self.u, self.config, self.goal, self.ob)
        x_ = self.goal[0] - self.x[0]
        y_ = self.goal[1] - self.x[1]
        angle2goal = math.atan2(y_, x_)
        distance2goal = math.sqrt(y_*y_ + x_*x_)

        if abs(angle2goal - self.x[2]) > 0.2:
            self.u = np.array([0.0, self.pid_angle(self.x[2]-angle2goal)])
        else:
            if distance2goal > 0.08:
                self.u = np.array([self.pid_distance(0.1-distance2goal), 0.0])
            else:
                self.u = np.array([0.0, 0.0])
                print("goal")

        # check goal
        if math.sqrt((self.x[0] - self.goal[0])**2 + (self.x[1] - self.goal[1])**2) <= self.config.robot_radius:
            print("Goal!!")
            self.u = np.array([0.0, 0.0])
        # print(self.u)

        return self.u, distance2goal

def main():
    print(__file__ + " start!!")

if __name__ == '__main__':
    main()