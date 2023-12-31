#! /usr/bin/env python3
import math
import numpy as np
import matplotlib.pyplot as plt

show_animation = False


class Config():
    # simulation parameters

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
        self.robot_radius = 0.18  # [m]

class dwa_approach():
    def __init__(self, NUM_ROBOT=4, GOAL=np.array([0, 0]), X=np.array([0.0, 0.0, math.pi / 8.0, 0.0, 0.0])):
        self.num_robots = NUM_ROBOT
        self.config = Config()

        # initial state [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
        self.x = X
        # goal position [x(m), y(m)]
        self.goal = GOAL
        # obstacles [x(m) y(m), ....]
        self.ob = np.matrix(np.arange(0., (self.num_robots-1)*2.).reshape((self.num_robots-1), 2))
        # control u [linear.x, angular.z]
        self.u = np.array([0.0, 0.0])

    def motion(self, x, u, dt):
        # motion model

        x[2] += u[1] * dt
        x[0] += u[0] * math.cos(x[2]) * dt
        x[1] += u[0] * math.sin(x[2]) * dt
        x[3] = u[0]
        x[4] = u[1]

        return x


    def calc_dynamic_window(self, x, config):

        # Dynamic window from robot specification
        Vs = [config.min_speed, config.max_speed,
            -config.max_yawrate, config.max_yawrate]

        # Dynamic window from motion model
        Vd = [x[3] - config.max_accel * config.dt,
            x[3] + config.max_accel * config.dt,
            x[4] - config.max_dyawrate * config.dt,
            x[4] + config.max_dyawrate * config.dt]
        #  print(Vs, Vd)

        #  [vmin,vmax, yawrate min, yawrate max]
        dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
            max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]

        return dw


    def calc_trajectory(self, xinit, v, y, config):

        x = np.array(xinit)
        traj = np.array(x)
        time = 0
        while time <= config.predict_time:
            x = self.motion(x, [v, y], config.dt)
            traj = np.vstack((traj, x))
            time += config.dt

        return traj


    def calc_final_input(self, x, u, dw, config, goal, ob):

        xinit = x[:]
        min_cost = 10000.0
        min_u = u
        min_u[0] = 0.0
        best_traj = np.array([x])

        # evalucate all trajectory with sampled input in dynamic window
        for v in np.arange(dw[0], dw[1], config.v_reso):
            for y in np.arange(dw[2], dw[3], config.yawrate_reso):
                traj = self.calc_trajectory(xinit, v, y, config)

                # calc cost
                to_goal_cost = self.calc_to_goal_cost(traj, goal, config)
                speed_cost = config.speed_cost_gain * \
                    (config.max_speed - traj[-1, 3])
                ob_cost = self.calc_obstacle_cost(traj, ob, config)
                #print(ob_cost)

                final_cost = to_goal_cost + speed_cost + ob_cost

                # search minimum trajectory
                if min_cost >= final_cost:
                    min_cost = final_cost
                    min_u = [v, y]
                    best_traj = traj

        return min_u, best_traj


    def calc_obstacle_cost(self, traj, ob, config):
        # calc obstacle cost inf: collistion, 0:free

        skip_n = 2
        minr = float("inf")

        for ii in range(0, len(traj[:, 1]), skip_n):
            for i in range(len(ob[:, 0])):
                ox = ob[i, 0]
                oy = ob[i, 1]
                dx = traj[ii, 0] - ox
                dy = traj[ii, 1] - oy

                # r = math.sqrt(dx**2 + dy**2)
                r = math.sqrt(dx**2 + dy**2) * 9.0
                if r <= config.robot_radius:
                    return float("Inf")  # collision

                if minr >= r:
                    minr = r

                # print(r)

        return 1.0 / minr  # OK


    def calc_to_goal_cost(self, traj, goal, config):
        # calc to goal cost. It is 2D norm.

        goal_magnitude = math.sqrt(goal[0]**2 + goal[1]**2)
        traj_magnitude = math.sqrt(traj[-1, 0]**2 + traj[-1, 1]**2)
        dot_product = (goal[0]*traj[-1, 0]) + (goal[1]*traj[-1, 1])
        error = dot_product / (goal_magnitude*traj_magnitude)
        error_angle = math.acos(error)
        cost = config.to_goal_cost_gain * error_angle

        return cost


    def dwa_control(self, x, u, config, goal, ob):
        # Dynamic Window control

        dw = self.calc_dynamic_window(x, config)

        u, traj = self.calc_final_input(x, u, dw, config, goal, ob)

        return u, traj


    def plot_arrow(self, x, y, yaw, length=0.5, width=0.1):
        plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
                head_length=width, head_width=width)
        plt.plot(x, y)

    def loop(self):
        # print(self.x)
        self.u, ltraj = self.dwa_control(self.x, self.u, self.config, self.goal, self.ob)
        # check goal
        if math.sqrt((self.x[0] - self.goal[0])**2 + (self.x[1] - self.goal[1])**2) <= self.config.robot_radius:
            print("Goal!!")
            self.u = np.array([0.0, 0.0])
        # print(self.u)

        if show_animation:
            plt.cla()
            plt.plot(ltraj[:, 0], ltraj[:, 1], "-g")
            plt.plot(self.x[0], self.x[1], "xr")
            plt.plot(self.goal[0], self.goal[1], "xb")
            plt.plot(self.ob[:, 0], self.ob[:, 1], "ok")
            # plt.plot(10, 10,  "ok")
            self.plot_arrow(self.x[0], self.x[1], self.x[2])
            plt.axis("equal")
            plt.grid(True)
            plt.pause(0.0001)

        return self.u

def main():
    # print(__file__ + " start!!")
    # initial state [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
    x = np.array([0.0, 0.0, math.pi / 8.0, 0.0, 0.0])
    # goal position [x(m), y(m)]
    goal = np.array([10, 10])
    # obstacles [x(m) y(m), ....]
    ob = np.matrix([[-1, -1],
                    [0, 2],
                    [4.0, 2.0],
                    [5.0, 4.0],
                    [5.0, 5.0],
                    [5.0, 6.0],
                    [5.0, 9.0],
                    [8.0, 9.0],
                    [7.0, 9.0],
                    [12.0, 12.0]
                    ])

    u = np.array([0.0, 0.0])
    config = Config()
    traj = np.array(x)

    for i in range(1000):
        # ob[0] = []

        u, ltraj = dwa_control(x, u, config, goal, ob)
        # print(u)

        x = motion(x, u, config.dt)
        traj = np.vstack((traj, x))  # store state history
        # if i <= 5:
        #   print(traj)

        if show_animation:
            plt.cla()
            plt.plot(ltraj[:, 0], ltraj[:, 1], "-g")
            plt.plot(x[0], x[1], "xr")
            plt.plot(goal[0], goal[1], "xb")
            plt.plot(ob[:, 0], ob[:, 1], "ok")
            plot_arrow(x[0], x[1], x[2])
            plt.axis("equal")
            plt.grid(True)
            plt.pause(0.0001)

        # check goal
        if math.sqrt((x[0] - goal[0])**2 + (x[1] - goal[1])**2) <= config.robot_radius:
            print("Goal!!")
            break

    print("Done")
    if show_animation:
        plt.plot(traj[:, 0], traj[:, 1], "-r")
        plt.show()


if __name__ == '__main__':
    main()