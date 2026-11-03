import math
import time
import struct
import rclpy

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, Range, PointCloud2, PointField
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformBroadcaster
from std_msgs.msg import Float64


class TRWebotsDriver:
    """ROS 2 driver untuk TR mecanum robot (Versi Stabil + Gripper & Tip + Lidar + Visual Marker)."""

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

        # STATE MARKER (Visualisasi 3D untuk Foxglove)
        self._marker_pub = None

        # STATE GRIPPER & TIP
        self._grip_left_motor = None
        self._grip_right_motor = None
        self._grip_tip_left = None
        self._grip_tip_right = None
        
        self._gripper_sub = None
        self._tip_sub = None

        # Status Gripper & Tip (0.0 tutup, 1.0 buka)
        self._grip_state = 0.0
        self._tip_state = 0.0

        # PARAMETER KALIBRASI ARAH LIDAR
        self._lidar_mirror = False
        self._lidar_yaw_offset = 0.0

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
            Twist, '/tr/cmd_vel', self._cmd_vel_callback, 10)
            
        self._gripper_sub = self._node.create_subscription(
            Float64, '/tr/gripper_cmd', self._gripper_callback, 10)
            
        self._tip_sub = self._node.create_subscription(
            Float64, '/tr/gripper_tip_cmd', self._tip_callback, 10)

        # 7. Publishers
        self._joint_state_publisher = self._node.create_publisher(JointState, '/tr/joint_states', 10)
        self._odom_publisher = self._node.create_publisher(Odometry, '/tr/odom', 10)
        
        # Publisher untuk Lidar
        self._lidar_pub = self._node.create_publisher(PointCloud2, '/tr/lidar/point_cloud', 10)
        
        # Publisher untuk Range
        self._range_pub = self._node.create_publisher(Range, '/tr/range_sensor/range', 10)

        # Publisher untuk Visualisasi 3D (MarkerArray)
        self._marker_pub = self._node.create_publisher(MarkerArray, '/tr/robot_markers', 10)

        self._tf_broadcaster = TransformBroadcaster(self._node)
        self._node.get_logger().info('TR driver aktif (Siap menerima perintah Gripper & Tip)')

    # --- CALLBACK GRIPPER UTAMA ---
    def _gripper_callback(self, msg):
        if self._grip_left_motor and self._grip_right_motor:
            target_angle = msg.data * 0.6 
            self._grip_left_motor.setPosition(target_angle)
            self._grip_right_motor.setPosition(-target_angle)
            self._grip_state = msg.data  # Simpan state untuk Marker
            self._node.get_logger().info(f'Gripper Utama: {target_angle}')
        else:
            self._node.get_logger().error('Gagal gerakin Gripper: Motor gak ketemu!')

    # --- CALLBACK UJUNG JARI (TIP) ---
    def _tip_callback(self, msg):
        if self._grip_tip_left and self._grip_tip_right:
            target_angle = msg.data
            self._grip_tip_left.setPosition(target_angle)
            self._grip_tip_right.setPosition(-target_angle)
            self._tip_state = msg.data    # Simpan state untuk Marker
            self._node.get_logger().info(f'Tip Hook: {target_angle}')
        else:
            self._node.get_logger().error('Gagal gerakin Tip: Motor gak ketemu!')

    # ====================================================
    # LOGIKA RODA & ODOMETRI
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
        
        msg.name = [
            self._wheel_joint_names['fl'], 
            self._wheel_joint_names['fr'], 
            self._wheel_joint_names['rl'], 
            self._wheel_joint_names['rr'],
            'gripper_left_joint',
            'gripper_right_joint'
        ]
        
        msg.position = [
            angles['fl'], angles['fr'], angles['rl'], angles['rr'],
            0.0, 0.0
        ]
        
        msg.velocity = [
            velocities['fl'], velocities['fr'], velocities['rl'], velocities['rr'],
            0.0, 0.0
        ]
        
        self._joint_state_publisher.publish(msg)

    # ====================================================
    # LOGIKA VISUAL MARKER UNTUK FOXGLOVE (DIRAPIKAN)
    # ====================================================
    def _make_marker(self, stamp, marker_id, marker_type, pos, scale, color,
                     quat=(0.0, 0.0, 0.0, 1.0)):
        """Helper pembuat marker supaya kode visual lebih ringkas & konsisten."""
        m = Marker()
        m.header.frame_id = 'tr/base_footprint'
        m.header.stamp = stamp
        m.ns = 'robot_visual'
        m.id = marker_id
        m.type = marker_type
        m.action = Marker.ADD
        m.pose.position.x = float(pos[0])
        m.pose.position.y = float(pos[1])
        m.pose.position.z = float(pos[2])
        m.pose.orientation.x = float(quat[0])
        m.pose.orientation.y = float(quat[1])
        m.pose.orientation.z = float(quat[2])
        m.pose.orientation.w = float(quat[3])
        m.scale.x = float(scale[0])
        m.scale.y = float(scale[1])
        m.scale.z = float(scale[2])
        m.color.r = float(color[0])
        m.color.g = float(color[1])
        m.color.b = float(color[2])
        m.color.a = float(color[3])
        return m

    def _publish_markers(self, stamp):
        if self._marker_pub is None:
            return

        markers = []

        # ---- Dimensi (mengikuti parameter kinematika driver agar konsisten) ----
        r = self._wheel_radius            # jari-jari roda = tinggi sumbu roda dari lantai
        wx = self._half_wheelbase         # jarak roda depan/belakang dari pusat (sumbu X)
        wy = self._half_track_width       # jarak roda kiri/kanan dari pusat (sumbu Y)
        wheel_width = 0.06

        chassis_len = 0.52
        chassis_wid = 0.50
        chassis_h = 0.10
        chassis_z = r + chassis_h / 2.0   # dasar chassis sejajar sumbu roda (z = 0.15)

        # Warna
        c_body = (0.10, 0.25, 0.80, 1.0)
        c_deck = (0.07, 0.17, 0.60, 1.0)
        c_tire = (0.05, 0.05, 0.05, 1.0)
        c_hub = (0.70, 0.70, 0.72, 1.0)
        c_front = (1.00, 0.80, 0.10, 1.0)
        c_grip = (0.95, 0.50, 0.10, 1.0)

        # 1. Chassis + plat atas
        markers.append(self._make_marker(
            stamp, 0, Marker.CUBE,
            (0.0, 0.0, chassis_z),
            (chassis_len, chassis_wid, chassis_h), c_body))

        markers.append(self._make_marker(
            stamp, 1, Marker.CUBE,
            (0.0, 0.0, r + chassis_h + 0.005),
            (chassis_len - 0.06, chassis_wid - 0.06, 0.01), c_deck))

        # 2. Penanda depan robot (strip kuning di bumper depan)
        markers.append(self._make_marker(
            stamp, 2, Marker.CUBE,
            (chassis_len / 2.0 + 0.005, 0.0, chassis_z + 0.02),
            (0.01, chassis_wid, 0.04), c_front))

        # 3. Roda: sumbu silinder harus sejajar sumbu Y (putar 90 derajat terhadap X)
        #    dan pusatnya setinggi jari-jari roda supaya menapak tepat di lantai.
        s45 = math.sqrt(0.5)
        wheel_quat = (s45, 0.0, 0.0, s45)  # quaternion ternormalisasi (90 derajat pada sumbu X)

        wheel_layout = [
            (10, 11, +wx, +wy),   # FL
            (12, 13, +wx, -wy),   # FR
            (14, 15, -wx, +wy),   # RL
            (16, 17, -wx, -wy),   # RR
        ]
        for tire_id, hub_id, px, py in wheel_layout:
            # Ban
            markers.append(self._make_marker(
                stamp, tire_id, Marker.CYLINDER,
                (px, py, r),
                (2.0 * r, 2.0 * r, wheel_width), c_tire, wheel_quat))
            # Hub abu-abu (sedikit lebih lebar supaya terlihat di kedua sisi ban)
            markers.append(self._make_marker(
                stamp, hub_id, Marker.CYLINDER,
                (px, py, r),
                (0.10, 0.10, wheel_width + 0.008), c_hub, wheel_quat))

        # 4. Gripper Kiri & Kanan (posisi sama seperti sebelumnya, warna dibedakan)
        markers.append(self._make_marker(
            stamp, 20, Marker.CUBE,
            (0.30, 0.19, 0.11),
            (0.25, 0.02, 0.02), c_grip))
        markers.append(self._make_marker(
            stamp, 21, Marker.CUBE,
            (0.30, -0.19, 0.11),
            (0.25, 0.02, 0.02), c_grip))

        marker_array = MarkerArray()
        # Hapus marker lama (id 0-6 dari versi sebelumnya) supaya tidak muncul ganda
        clear = Marker()
        clear.header.frame_id = 'tr/base_footprint'
        clear.header.stamp = stamp
        clear.ns = 'robot_visual'
        clear.action = Marker.DELETEALL
        marker_array.markers.append(clear)
        marker_array.markers.extend(markers)

        self._marker_pub.publish(marker_array)

    # ====================================================
    # LOGIKA LIDAR
    # ====================================================

    @staticmethod
    def _wrap_angle(a):
        return math.atan2(math.sin(a), math.cos(a))

    def _lidar_ray_angle(self, col, hres, fov):
        if fov < (2.0 * math.pi - 1e-3) and hres > 1:
            step = fov / (hres - 1)
        else:
            step = fov / hres
        angle = (fov / 2.0) - col * step
        if self._lidar_mirror:
            angle = -angle
        return self._wrap_angle(angle + self._lidar_yaw_offset)

    def _publish_lidar(self, stamp):
        if self._range_sensor is None:
            return

        range_image = self._range_sensor.getRangeImage()
        if range_image is None or len(range_image) == 0:
            return

        hres = self._range_sensor.getHorizontalResolution()
        fov = self._range_sensor.getFov()
        if hres <= 0:
            hres = len(range_image)
        if fov <= 0.0:
            fov = 2.0 * math.pi

        max_range = 5.0
        min_range = 0.1

        # 1. Publish sebagai Range
        front_half_angle = 0.25
        min_dist = max_range
        for i, d in enumerate(range_image):
            if math.isnan(d) or math.isinf(d) or d <= min_range:
                continue
            col = i % hres
            angle = self._lidar_ray_angle(col, hres, fov)
            if abs(angle) <= front_half_angle and d < min_dist:
                min_dist = d

        msg_range = Range()
        msg_range.header.stamp = stamp
        msg_range.header.frame_id = 'tr/lidar'
        msg_range.radiation_type = 0
        msg_range.field_of_view = 0.5
        msg_range.min_range = min_range
        msg_range.max_range = max_range
        msg_range.range = min_dist
        self._range_pub.publish(msg_range)

        # 2. Publish sebagai PointCloud2
        points = []
        for i, r in enumerate(range_image):
            if math.isnan(r) or math.isinf(r) or r <= min_range or r >= max_range:
                continue
            col = i % hres
            angle = self._lidar_ray_angle(col, hres, fov)
            x = r * math.cos(angle)
            y = r * math.sin(angle)
            z = 0.0
            points.append((x, y, z))

        if points:
            msg_pc = PointCloud2()
            msg_pc.header.stamp = stamp
            msg_pc.header.frame_id = 'tr/lidar'
            msg_pc.height = 1
            msg_pc.width = len(points)
            msg_pc.fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            ]
            msg_pc.is_bigendian = False
            msg_pc.point_step = 12
            msg_pc.row_step = msg_pc.point_step * msg_pc.width
            msg_pc.is_dense = True

            buffer = []
            for p in points:
                buffer.extend(p)
            msg_pc.data = struct.pack('f' * len(buffer), *buffer)

            self._lidar_pub.publish(msg_pc)

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
        odom.header.frame_id = 'tr/odom'
        odom.child_frame_id = 'tr/base_footprint'
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
        tf.header.frame_id = 'tr/odom'
        tf.child_frame_id = 'tr/base_footprint'
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
            self._publish_lidar(stamp)
            self._publish_markers(stamp) # <--- PUBLISH MARKER DI SINI

            # TF Lidar
            tf_lidar = TransformStamped()
            tf_lidar.header.stamp = stamp
            tf_lidar.header.frame_id = 'tr/base_footprint'
            tf_lidar.child_frame_id = 'tr/lidar'
            
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
    driver._cmd_vel_subscription = node.create_subscription(Twist, '/tr/cmd_vel', driver._cmd_vel_callback, 10)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()