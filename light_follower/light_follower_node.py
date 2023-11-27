import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from tf2_msgs.msg import TFMessage
import copy
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
import tf2_geometry_msgs

TIME_PERIOD = 0.1
NUM_ROBOT = 4

class calc_relative_pos:
    def __init__(self):
        pass

    def 
        
class robot_control:
    pass

class light_follower_node(Node):
    def __init__(self):
        super().__init__('light_follower_node')

        self.inital_pose_sub = self.create_subscription(PoseWithCovarianceStamped, 'initialpose', self.initial_pose_callback, rclpy.qos.qos_profile_sensor_data)

        self.get_pos_list = [0 for i in range(NUM_ROBOT)]
        self.get_rot_list = [0 for i in range(NUM_ROBOT)]
        self.get_yaw_list = [0 for i in range(NUM_ROBOT)]

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.1, self.timer_callback)

        # self.time_period = TIME_PERIOD
        # self.tmr = self.create_timer(self.time_period, self.callback)

        self.calc_class = calc_relative_pos()

        # initialize
        self.init = False

        # goal pose 
        self.get_pos = 0
    
    def initial_pose_callback(self, msg):
        self.get_pos = copy.deepcopy(msg)
        return self.get_pos

    def timer_callback(self):
        try:
            for i in range(NUM_ROBOT):
                transform = self.tf_buffer.lookup_transform('robot' + str(i+1), 'map', rclpy.time.Time())
                position = transform.transform.translation
                rotation = transform.transform.rotation
                self.get_pos_list[i] = copy.deepcopy(position)
                self.get_rot_list[i] = copy.deepcopy(rotation)
            self.init = True

            # test
            if self.init:
                print(self.get_pos_list[0].x)

        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().info(f'Exception: {e}')

def main(args=None):
    rclpy.init(args=args)
    class_node = light_follower_node()
    rclpy.spin(class_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
