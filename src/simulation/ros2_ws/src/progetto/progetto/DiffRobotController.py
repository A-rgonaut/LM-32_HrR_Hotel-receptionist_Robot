import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, Quaternion, PoseStamped, TransformStamped, PoseWithCovarianceStamped
from std_msgs.msg import Float64
from tf2_ros import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from math import sin, cos, atan2, pi

class DiffRobotController(Node):
    def __init__(self):
        super().__init__("DiffRobotController")
        self.L          = 0.28  # m
        self.R          = 0.07  # m
        self.init_x     = 10.0
        self.init_y     = 11.0
        self.init_theta = pi

        self.odometry_publisher = self.create_publisher(Odometry,
            '/odom', 10)
        self.sensor_subscriber = self.create_subscription(JointState,
            '/wheels_state', self.compute_odometry, 10)
        self.ext_control_subscriber = self.create_subscription(Twist,
            '/cmd_vel', self.command_turn, 10)
        self.l_wheel_publisher = self.create_publisher(Float64,
            '/left_wheel_cmd', 10)
        self.r_wheel_publisher = self.create_publisher(Float64,
            '/right_wheel_cmd', 10)

        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped,
            '/initialpose', 10)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.publish_static_transforms()

        self.path_pub = self.create_publisher(Path, '/percorso_effettivo', 10)
        self.actual_path = Path()
        self.actual_path.header.frame_id = 'odom'

        self.odom_timer = self.create_timer(0.1, self.publish_odometry)

        self.x     = 0.0
        self.y     = 0.0
        self.theta = 0.0

        self.amcl_init_timer = self.create_timer(3.0, self.send_initial_pose_to_amcl)

        self.current_time = self.get_clock().now()

        self.get_logger().info('DiffRobotController avviato.')

    def send_initial_pose_to_amcl(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = self.init_x
        msg.pose.pose.position.y = self.init_y
        msg.pose.pose.position.z = 0.0
        q = self.euler_to_quaternion(0, 0, self.init_theta)
        msg.pose.pose.orientation = q
        msg.pose.covariance = [0.0] * 36
        msg.pose.covariance[0]  = 0.001
        msg.pose.covariance[7]  = 0.001
        msg.pose.covariance[35] = 0.001
        self.initial_pose_pub.publish(msg)
        self.get_logger().info(f'Inviata posa iniziale ad AMCL: x={self.init_x}, y={self.init_y}')
        self.amcl_init_timer.cancel()

    def publish_static_transforms(self):
        now = self.get_clock().now().to_msg()
        # 1. Transform: base_footprint -> base_link
        t_base = TransformStamped()
        t_base.header.stamp.sec = 0
        t_base.header.stamp.nanosec = 0
        t_base.header.frame_id = 'base_footprint'
        t_base.child_frame_id = 'base_link'
        t_base.transform.translation.x = 0.0
        t_base.transform.translation.y = 0.0
        t_base.transform.translation.z = 0.0
        t_base.transform.rotation.w = 1.0
        # 2. Transform: base_link -> lidar_link
        t_lidar = TransformStamped()
        t_lidar.header.stamp.sec = 0
        t_lidar.header.stamp.nanosec = 0
        t_lidar.header.frame_id = 'base_link'
        t_lidar.child_frame_id = 'lidar_link'
        t_lidar.transform.translation.x = 0.0
        t_lidar.transform.translation.y = 0.0
        t_lidar.transform.translation.z = 0.3
        t_lidar.transform.rotation.w = 1.0
        self.tf_static_broadcaster.sendTransform([t_base, t_lidar])

    def compute_odometry(self, encoder_msgs):
        msg_time_sec = encoder_msgs.header.stamp.sec
        msg_time_nanosec = encoder_msgs.header.stamp.nanosec
        current_msg_time = msg_time_sec + msg_time_nanosec * 1e-9
        if not hasattr(self, 'last_msg_time'):
            self.last_msg_time = current_msg_time
            return
        dt = current_msg_time - self.last_msg_time
        self.last_msg_time = current_msg_time
        if dt <= 0 or dt > 1.0:
            return

        w_R = encoder_msgs.velocity[0]
        w_L = encoder_msgs.velocity[1]

        v_R = w_R * self.R
        v_L = w_L * self.R

        # Cinematica diretta:
        if abs(v_R - v_L) > 1e-6:  # Moto circolare:
            R = self.L * 0.5 * (v_R + v_L) / (v_R - v_L)
            ICC = [self.x - R * sin(self.theta),
                   self.y + R * cos(self.theta)]
            w = (v_R - v_L) / self.L
            ICCx, ICCy = ICC[0], ICC[1]
            self.x, self.y = (
                (cos(w * dt) * (self.x - ICCx) -
                 sin(w * dt) * (self.y - ICCy) + ICCx),
                (sin(w * dt) * (self.x - ICCx) +
                 cos(w * dt) * (self.y - ICCy) + ICCy))
            self.theta = self.theta + w * dt
            self.theta = atan2(sin(self.theta), cos(self.theta))
        else:  # Moto rettilineo:
            self.x = self.x + (0.5 * (v_R + v_L) * cos(self.theta)) * dt
            self.y = self.y + (0.5 * (v_R + v_L) * sin(self.theta)) * dt

    def publish_odometry(self):
        now = self.get_clock().now().to_msg()
        quat = self.euler_to_quaternion(0, 0, self.theta)

        odometry_msg = Odometry()
        odometry_msg.header.stamp = now
        odometry_msg.header.frame_id = 'odom'
        odometry_msg.child_frame_id = 'base_footprint'
        odometry_msg.pose.pose.position.x = self.x
        odometry_msg.pose.pose.position.y = self.y
        odometry_msg.pose.pose.position.z = 0.0
        odometry_msg.pose.pose.orientation = quat
        self.odometry_publisher.publish(odometry_msg)

        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = quat
        self.tf_broadcaster.sendTransform(t)

        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = now
        pose_stamped.header.frame_id = 'odom'
        pose_stamped.pose = odometry_msg.pose.pose
        self.actual_path.header.stamp = now
        self.actual_path.poses.append(pose_stamped)
        if len(self.actual_path.poses) > 2000:
            self.actual_path.poses.pop(0)
        self.path_pub.publish(self.actual_path)

    def euler_to_quaternion(self, roll, pitch, yaw):
        cy = cos(yaw   * 0.5)
        sy = sin(yaw   * 0.5)
        cp = cos(pitch * 0.5)
        sp = sin(pitch * 0.5)
        cr = cos(roll  * 0.5)
        sr = sin(roll  * 0.5)
        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        return q

    def command_turn(self, cmd_msg):
        lin_vel = cmd_msg.linear.x
        ang_vel = cmd_msg.angular.z

        # Cinematica Inversa:
        # lin_vel = ang_vel * self.R
        v_R = (lin_vel + (ang_vel * self.L) / 2.0)
        v_L = (lin_vel - (ang_vel * self.L) / 2.0)
        w_R = v_R / self.R
        w_L = v_L / self.R

        L_msg = Float64()
        R_msg = Float64()
        L_msg.data = w_L
        R_msg.data = w_R
        self.l_wheel_publisher.publish(L_msg)
        self.r_wheel_publisher.publish(R_msg)

def main(args=None):
    rclpy.init()
    diff_robot_controller = DiffRobotController()
    rclpy.spin(diff_robot_controller)
    diff_robot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
