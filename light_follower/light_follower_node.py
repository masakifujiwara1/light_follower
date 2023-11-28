import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from tf2_msgs.msg import TFMessage
import copy
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
import tf2_geometry_msgs
import numpy as np
from . import dwa

TIME_PERIOD = 0.1
NUM_ROBOT = 4

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
        self.timer = self.create_timer(0.1, self.timer_callback)

        # self.time_period = TIME_PERIOD
        # self.tmr = self.create_timer(self.time_period, self.callback)

        self.calc_class = calc_relative_pos()

        # initialize
        self.init = False

        # goal pose 
        self.goal_pos = 0

        # instance dwa
        self.dwa = dwa.dwa_approach()

        # robot_control
        self.u_list = [0 for i in range(NUM_ROBOT)]
        for i in range(NUM_ROBOT):
            self.u_list[i] = np.array([0.0, 0.0])

        self.ob = np.matrix(np.arange(0., NUM_ROBOT*2.).reshape(NUM_ROBOT, 2))

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
                transform = self.tf_buffer.lookup_transform('robot' + str(i+1), 'map', rclpy.time.Time())
                position = transform.transform.translation
                rotation = transform.transform.rotation
                (roll, pitch, yaw) = self.euler_from_quaternion(rotation)
                # self.ob[i] = copy.deepcopy(np.array[position.x, position.y])
                self.ob[i] = copy.deepcopy(np.array([position.x, position.y]))
                self.get_pos_list[i] = copy.deepcopy(position)
                self.get_yaw_list[i] = copy.deepcopy(yaw)
            self.init = True

            if self.init and self.goal_flag:
                # debug
                # print(self.get_yaw_list)
                # print(self.get_pos_list[0].x)

                for i in range(NUM_ROBOT):
                    self.dwa.x = np.array([self.get_pos_list[i].x, self.get_pos_list[i].y, self.get_yaw_list[i], self.u_list[i][0], self.u_list[i][1]])
                    self.dwa.goal_pos = np.array([self.goal_pos.pose.pose.position.x, self.goal_pos.pose.pose.position.y])
                    ob_ = copy.deepcopy(self.ob)
                    # print(ob_)
                    # print(np.delete(ob_, i, 0))
                    self.dwa.ob = np.delete(ob_, i, 0)
                    # print(ob_)
                    # self.dwa.ob = ob_
                    # print(self.u_list[i])
                    self.dwa.u = self.u_list[i]
                    self.u_list[i] = copy.deepcopy(self.dwa.loop())
                    # print("robot" + str(i+1) , self.u_list[i])

                    # cmd_vel pub
                    self.cmd_vel.linear.x = copy.deepcopy(self.u_list[i][0])
                    self.cmd_vel.angular.z = copy.deepcopy(self.u_list[i][1])
                    print(i, cmd_vel)

                    if i == 0:
                        self.robot1_cmd_vel_pub.publish(self.cmd_vel)
                    if i == 1:
                        self.robot2_cmd_vel_pub.publish(self.cmd_vel)
                    if i == 2:
                        self.robot3_cmd_vel_pub.publish(self.cmd_vel)
                    if i == 3:
                        self.robot4_cmd_vel_pub.publish(self.cmd_vel)

                # self.cmd_vel.linear.x = self.u_list[0][0]
                # self.cmd_vel.angular.z = self.u_list[0][1]
                # print(self.get_pos_list[0])
                # print(self.get_yaw_list[0])
                # print(self.cmd_vel)
                # print(self.dwa.ob)
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
