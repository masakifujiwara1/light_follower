import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from tf2_msgs.msg import TFMessage
import copy
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
import tf2_geometry_msgs
import numpy as np
import math
from . import pid

TIME_PERIOD = 0.2
NUM_ROBOT = 4
ROBOTS_DIST = 0.6

class calc_relative_pos:
    def __init__(self):
        pass

    # def 
        
class robot_control:
    def __init__(self):
        pass
    
    # def 

class light_follower_node(Node):
    def __init__(self):
        super().__init__('light_follower_node')

        self.robot1_cmd_vel_pub =  self.create_publisher(Twist, 'robot1/cmd_vel', 1)
        self.robot2_cmd_vel_pub =  self.create_publisher(Twist, 'robot2/cmd_vel', 1)
        self.robot3_cmd_vel_pub =  self.create_publisher(Twist, 'robot3/cmd_vel', 1)
        self.robot4_cmd_vel_pub =  self.create_publisher(Twist, 'robot4/cmd_vel', 1)

        self.inital_pose_sub = self.create_subscription(PoseWithCovarianceStamped, 'initialpose', self.initial_pose_callback, rclpy.qos.qos_profile_sensor_data)

        self.get_pos_list = [0 for i in range(NUM_ROBOT)]
        self.get_yaw_list = [0 for i in range(NUM_ROBOT)]
        self.get_u_list = [0 for i in range(NUM_ROBOT)]

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.025, self.timer_callback)

        # self.time_period = TIME_PERIOD
        # self.tmr = self.create_timer(self.time_period, self.callback)

        self.calc_class = calc_relative_pos()

        # initialize
        self.init = False

        # goal pose 
        self.goal_pos = 0

        self.pid = []

        # instance pid
        for i in range(NUM_ROBOT):
            self.pid.append(pid.pid_controller())

        # robot_control
        self.u_list = [0 for i in range(NUM_ROBOT)]
        self.dist_list = [0 for i in range(NUM_ROBOT)]

        for i in range(NUM_ROBOT):
            self.u_list[i] = np.array([0.0, 0.0])

        self.ob = np.matrix(np.arange(0., NUM_ROBOT*2.).reshape(NUM_ROBOT, 2))
        self.ob *= 5

        self.store_robot_num = 10

        self.goal_flag = False

        self.cmd_vel = Twist()
    
    def initial_pose_callback(self, msg):
        self.goal_pos = copy.deepcopy(msg)
        self.goal_flag = True
        return self.goal_pos

    def euler_from_quaternion(self, quaternion):
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def timer_callback(self):
        try:
            for i in range(NUM_ROBOT):
            # for i in range(1):
                transform = self.tf_buffer.lookup_transform('map', 'robot' + str(i+1), rclpy.time.Time())
                position = transform.transform.translation
                rotation = transform.transform.rotation
                (roll, pitch, yaw) = self.euler_from_quaternion(rotation)
                # self.ob[i] = copy.deepcopy(np.array([position.x, position.y]))
                self.get_pos_list[i] = copy.deepcopy(position)
                self.get_yaw_list[i] = copy.deepcopy(yaw)
            self.init = True

            # print(self.ob)

            if self.init and self.goal_flag:
                # debug
                # print(self.get_yaw_list)
                # print(self.get_pos_list[0].x)

                for i in range(NUM_ROBOT):
                # for i in range(1):
                    self.pid[i].x = np.array([self.get_pos_list[i].x, self.get_pos_list[i].y, self.get_yaw_list[i], self.u_list[i][0], self.u_list[i][1]])
                    self.pid[i].goal = np.array([self.goal_pos.pose.pose.position.x, self.goal_pos.pose.pose.position.y])

                    self.u_list[i], self.dist_list[i] = copy.deepcopy(self.pid[i].loop())

                    # cmd_vel pub
                    self.cmd_vel.linear.x = copy.deepcopy(self.u_list[i][0])
                    self.cmd_vel.angular.z = copy.deepcopy(self.u_list[i][1])

                    # check between robots
                    robot_dist_min = 10000
                    self.store_robot_num = 100
                    # calc_between = np.delete(self.get_pos_list, i, 0)
                    for j in range(NUM_ROBOT):
                        # print(calc_between[j])
                        if not j == i:
                            x_ = self.get_pos_list[j].x - self.get_pos_list[i].x
                            y_ = self.get_pos_list[j].y - self.get_pos_list[i].y
                            calc_req = math.sqrt(x_*x_ + y_*y_)
                            if calc_req < robot_dist_min:
                                robot_dist_min = calc_req
                                self.store_robot_num = j
                    
                    if robot_dist_min <= ROBOTS_DIST:
                        if self.dist_list[i] > self.dist_list[self.store_robot_num]:
                            self.cmd_vel.linear.x = 0.0
                            # self.cmd_vel.angular.z = 0.0
                    
                    if i == 0:
                        self.robot1_cmd_vel_pub.publish(self.cmd_vel)
                    if i == 1:
                        self.robot2_cmd_vel_pub.publish(self.cmd_vel)
                    if i == 2:
                        self.robot3_cmd_vel_pub.publish(self.cmd_vel)
                    if i == 3:
                        self.robot4_cmd_vel_pub.publish(self.cmd_vel)

                # print(self.pid[0].u)
                # print(self.dist_list)
                # self.robot1_cmd_vel_pub.publish(self.cmd_vel)

        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().info(f'Exception: {e}')

def main(args=None):
    rclpy.init(args=args)
    class_node = light_follower_node()
    rclpy.spin(class_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
