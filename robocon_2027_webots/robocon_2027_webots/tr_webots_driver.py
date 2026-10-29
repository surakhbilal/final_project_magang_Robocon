import math
import time
import rclpy

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, Range, PointCloud2
from tf2_ros import TransformBroadcaster
from std_msgs.msg import Float64


class TRWebotsDriver:
    """ROS 2 driver untuk TR mecanum robot (Versi Stabil + Gripper & Tip + Lidar)."""

    def __init__(self):
        self._node = None
        self._robot = None
        self._motors = {}
        self._sensors = {}
        self._target_velocity = {}
        self._cmd_vel_subscription = None
        self._wheel_radius = 0.10
        self._half_wheelbase = 0.18
        self._half_track_width = 0.285
        self._max_wheel_speed = 20.0
        self._motor_direction = {
            'fl': 1.0, 'fr': 1.0, 'rl': 1.0, 'rr': 1.0,
        }
        self._command_timeout = 0.5
        self._last_command_time = time.monotonic()

        self._wheel_joint_names = {
            'fl': 'front_left_wheel_joint',
            'fr': 'front_right_wheel_joint',
            'rl': 'rear_left_wheel_joint',
            'rr': 'rear_right_wheel_joint',
        }
        self._last_wheel_angle = {
            'fl': 0.0, 'fr': 0.0, 'rl': 0.0, 'rr': 0.0,
        }
        self._timestep_sec = 0.032
        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0
        self._joint_state_publisher = None
        self._odom_publisher = None
        self._tf_broadcaster = None

        # STATE INERTIAL UNIT
        self._inertial_unit = None
        self._use_inertial_unit = False

        # STATE LIDAR
        self._range_sensor = None
        self._lidar_pub = None
        self._range_pub = None

        # STATE GRIPPER & TIP
        self._grip_left_motor = None
        self._grip_right_motor = None
        self._grip_tip_left = None
        self._grip_tip_right = None
        
        self._gripper_sub = None
        self._tip_sub = None

    def init(self, webots_node, properties):
        print('[TR DRIVER] init() dimulai', flush=True)
        
        if not rclpy.ok():
            rclpy.init()

        self._robot = getattr(webots_node, 'robot', None)
        self._node = rclpy.create_node('tr_webots_driver')

        if self._robot is not None:
            # 1. Setup Roda
            self._motors = {
                'fl': self._robot.getDevice('front_left_wheel_motor'),
                'fr': self._robot.getDevice('front_right_wheel_motor'),
                'rl': self._robot.getDevice('rear_left_wheel_motor'),
                'rr': self._robot.getDevice('rear_right_wheel_motor'),
            }
            for motor in self._motors.values():
                motor.setPosition(float('inf'))
                motor.setVelocity(0.0)

            self._sensors = {
                'fl': self._robot.getDevice('front_left_wheel_sensor'),
                'fr': self._robot.getDevice('front_right_wheel_sensor'),
                'rl': self._robot.getDevice('rear_left_wheel_sensor'),
                'rr': self._robot.getDevice('rear_right_wheel_sensor'),
            }
            timestep_ms = int(self._robot.getBasicTimeStep())
            self._timestep_sec = timestep_ms / 1000.0
            for sensor in self._sensors.values():
                sensor.enable(timestep_ms)

            # 2. Setup Inertial Unit
            self._inertial_unit = self._robot.getDevice('inertial_unit')
            if self._inertial_unit is not None:
                self._inertial_unit.enable(timestep_ms)
                self._use_inertial_unit = True
                print('[TR DRIVER] ✅ InertialUnit AKTIF', flush=True)

            # 3. Setup LIDAR
            self._range_sensor = self._robot.getDevice('lidar') 
            if self._range_sensor is not None:
                self._range_sensor.enable(timestep_ms)
                print('[TR DRIVER] ✅ Lidar AKTIF', flush=True)
            else:
                print('[TR DRIVER] ⚠️ Lidar tidak ditemukan di Proto!', flush=True)

            # 4. Setup Gripper Utama
            self._grip_left_motor = self._robot.getDevice('gripper_left_motor')
            self._grip_right_motor = self._robot.getDevice('gripper_right_motor')
            if self._grip_left_motor and self._grip_right_motor:
                self._grip_left_motor.setPosition(0.0)
                self._grip_right_motor.setPosition(0.0)
                print('[TR DRIVER] ✅ Gripper AKTIF', flush=True)
            else:
                print('[TR DRIVER] ⚠️ Motor Gripper tidak ditemukan di Proto!', flush=True)

            # 5. Setup Ujung Jari (Tip)
            self._grip_tip_left = self._robot.getDevice('gripper_ujung_kiri_motor')
            self._grip_tip_right = self._robot.getDevice('gripper_ujung_kanan_motor')
            if self._grip_tip_left and self._grip_tip_right:
                self._grip_tip_left.setPosition(0.0)
                self._grip_tip_right.setPosition(0.0)
                print('[TR DRIVER] ✅ Ujung Jari Gripper AKTIF', flush=True)
            else:
                print('[TR DRIVER] ⚠️ Motor Ujung Jari tidak ditemukan di Proto!', flush=True)

        self._target_velocity = {'fl': 0.0, 'fr': 0.0, 'rl': 0.0, 'rr': 0.0}
        
        # 6. Subscribers
        self._cmd_vel_subscription = self._node.create_subscription(
            Twist, '/cmd_vel', self._cmd_vel_callback, 10)
            
        self._gripper_sub = self._node.create_subscription(
            Float64, '/gripper_cmd', self._gripper_callback, 10)
            
        self._tip_sub = self._node.create_subscription(
            Float64, '/gripper_tip_cmd', self._tip_callback, 10)

        # 7. Publishers
        self._joint_state_publisher = self._node.create_publisher(JointState, '/joint_states', 10)
        self._odom_publisher = self._node.create_publisher(Odometry, '/odom', 10)
        
        # Publisher untuk Lidar (PointCloud2)
        self._lidar_pub = self._node.create_publisher(PointCloud2, '/tr_robot/lidar/point_cloud', 10)
        
        # Publisher untuk Range (Buat di-plot 2D)
        self._range_pub = self._node.create_publisher(Range, '/range_sensor/range', 10)

        self._tf_broadcaster = TransformBroadcaster(self._node)
        self._node.get_logger().info('TR driver aktif (Siap menerima perintah Gripper & Tip)')

    # --- CALLBACK GRIPPER UTAMA ---
    def _gripper_callback(self, msg):
        if self._grip_left_motor and self._grip_right_motor:
            target_angle = msg.data * 0.6 
            self._grip_left_motor.setPosition(target_angle)
            self._grip_right_motor.setPosition(-target_angle)
            self._node.get_logger().info(f'Gripper Utama: {target_angle}')
        else:
            self._node.get_logger().error('Gagal gerakin Gripper: Motor gak ketemu!')

    # --- CALLBACK UJUNG JARI (TIP) ---
    def _tip_callback(self, msg):
        if self._grip_tip_left and self._grip_tip_right:
            target_angle = msg.data
            self._grip_tip_left.setPosition(target_angle)
            self._grip_tip_right.setPosition(-target_angle)
            self._node.get_logger().info(f'Tip Hook: {target_angle}')
        else:
            self._node.get_logger().error('Gagal gerakin Tip: Motor gak ketemu!')

    # ====================================================
    # LOGIKA RODA & ODOMETRI (TIDAK DIUBAH SAMA SEKALI)
    # ====================================================

    def _normalize_wheel_speeds(self, fl, fr, rl, rr):
        max_abs = max(abs(fl), abs(fr), abs(rl), abs(rr), 1.0)
        if max_abs > self._max_wheel_speed:
            scale = self._max_wheel_speed / max_abs
            fl *= scale; fr *= scale; rl *= scale; rr *= scale
        return fl, fr, rl, rr

    def _cmd_vel_callback(self, message):
        vx, vy, wz = message.linear.x, -message.linear.y, message.angular.z
        lever_arm = self._half_wheelbase + self._half_track_width
        fl = (vx - vy - lever_arm * wz) / self._wheel_radius
        fr = (vx + vy + lever_arm * wz) / self._wheel_radius
        rl = (vx + vy - lever_arm * wz) / self._wheel_radius
        rr = (vx - vy + lever_arm * wz) / self._wheel_radius
        fl, fr, rl, rr = self._normalize_wheel_speeds(fl, fr, rl, rr)
        self._target_velocity['fl'] = fl
        self._target_velocity['fr'] = fr
        self._target_velocity['rl'] = rl
        self._target_velocity['rr'] = rr
        self._last_command_time = time.monotonic()

    def _command_has_timed_out(self):
        return (time.monotonic() - self._last_command_time) > self._command_timeout

    def _stop_robot(self):
        for wheel_name in self._target_velocity:
            self._target_velocity[wheel_name] = 0.0

    def _read_wheel_velocities(self):
        dt = self._timestep_sec
        angles = {}; velocities = {}
        for wheel_name, sensor in self._sensors.items():
            angle = sensor.getValue()
            if math.isnan(angle): angle = self._last_wheel_angle[wheel_name]
            velocities[wheel_name] = (angle - self._last_wheel_angle[wheel_name]) / dt
            self._last_wheel_angle[wheel_name] = angle
            angles[wheel_name] = angle
        return angles, velocities

    def _publish_joint_states(self, stamp, angles, velocities):
        msg = JointState()
        msg.header.stamp = stamp
        msg.name = [self._wheel_joint_names[key] for key in ('fl', 'fr', 'rl', 'rr')]
        msg.position = [angles[key] for key in ('fl', 'fr', 'rl', 'rr')]
        msg.velocity = [velocities[key] for key in ('fl', 'fr', 'rl', 'rr')]
        self._joint_state_publisher.publish(msg)

    def _publish_rangefinder(self, stamp):
        if self._range_sensor is not None:
            range_image = self._range_sensor.getRangeImage()
            
            if range_image is not None and len(range_image) > 0:
                center_idx = len(range_image) // 2
                min_dist = 5.0
                
                for i in range(center_idx - 15, center_idx + 15):
                    if 0 <= i < len(range_image):
                        d = range_image[i]
                        if not math.isnan(d) and not math.isinf(d) and d > 0.1:
                            if d < min_dist:
                                min_dist = d
                
                msg = Range()
                msg.header.stamp = stamp
                msg.header.frame_id = 'lidar'
                msg.radiation_type = 0
                msg.field_of_view = 0.5
                msg.min_range = 0.1
                msg.max_range = 5.0 
                msg.range = min_dist
                self._range_pub.publish(msg)

    def _update_odometry(self, stamp, velocities):
        dt = self._timestep_sec
        r = self._wheel_radius
        lever_arm = self._half_wheelbase + self._half_track_width

        fl, fr, rl, rr = velocities['fl'], velocities['fr'], velocities['rl'], velocities['rr']
        vx = r * (fl + fr + rl + rr) / 4.0
        vy_internal = r * (fr + rl - fl - rr) / 4.0
        vy = -vy_internal

        if self._use_inertial_unit and self._inertial_unit is not None:
            rpy = self._inertial_unit.getRollPitchYaw()
            self._theta = -rpy[2]
            wz = 0.0
        else:
            wz = r * (fr + rr - fl - rl) / (4.0 * lever_arm)
            self._theta += wz * dt

        self._x += (vx * math.cos(self._theta) - vy * math.sin(self._theta)) * dt
        self._y += (vx * math.sin(self._theta) + vy * math.cos(self._theta)) * dt

        quat_z = math.sin(self._theta / 2.0)
        quat_w = math.cos(self._theta / 2.0)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.orientation.z = quat_z
        odom.pose.pose.orientation.w = quat_w
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
        self._odom_publisher.publish(odom)

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'base_footprint'
        tf.transform.translation.x = self._x
        tf.transform.translation.y = self._y
        tf.transform.rotation.z = quat_z
        tf.transform.rotation.w = quat_w
        self._tf_broadcaster.sendTransform(tf)

    def step(self):
        if self._node is None:
            return
        rclpy.spin_once(self._node, timeout_sec=0)
        if self._command_has_timed_out():
            self._stop_robot()
        for wheel_name, motor in self._motors.items():
            motor.setVelocity(self._target_velocity[wheel_name] * self._motor_direction[wheel_name])
        
        if self._sensors:
            stamp = self._node.get_clock().now().to_msg()
            angles, velocities = self._read_wheel_velocities()
            self._publish_joint_states(stamp, angles, velocities)
            self._update_odometry(stamp, velocities)
            self._publish_rangefinder(stamp)

            # TF Lidar
            tf_lidar = TransformStamped()
            tf_lidar.header.stamp = stamp
            tf_lidar.header.frame_id = 'base_footprint'
            tf_lidar.child_frame_id = 'lidar'
            
            tf_lidar.transform.translation.x = 0.21
            tf_lidar.transform.translation.y = 0.04
            tf_lidar.transform.translation.z = 0.14
            tf_lidar.transform.rotation.w = 1.0
            
            self._tf_broadcaster.sendTransform(tf_lidar)

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('tr_webots_driver')
    driver = TRWebotsDriver()
    driver._node = node
    driver._robot = None
    driver._cmd_vel_subscription = node.create_subscription(Twist, '/cmd_vel', driver._cmd_vel_callback, 10)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()