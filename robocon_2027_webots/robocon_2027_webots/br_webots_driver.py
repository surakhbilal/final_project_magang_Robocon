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


class BRWebotsDriver:
    """ROS 2 driver untuk BR (Builder Robot) mecanum + Slider + Gripper + Lidar + Visual Marker.

    Kamera TIDAK dipublish di sini. Sama seperti TR, kamera dipublish otomatis oleh
    webots_ros2_driver di /br_robot/camera/image_color & /br_robot/camera/camera_info
    (frame_id: camera), sehingga image & camera_info selalu konsisten.
    """

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

        # STATE MARKER (Visualisasi 3D)
        self._marker_pub = None

        # STATE GRIPPER & SLIDER (Z-AXIS)
        self._grip_left_motor = None
        self._grip_right_motor = None
        self._z_axis_motor = None
        self._z_axis_sensor = None

        self._gripper_sub = None
        self._z_axis_sub = None

        # Posisi slider (linear), default di bawah
        self._z_position = 0.0

        # Status Gripper (0.0 tutup, 1.0 buka)
        self._grip_state = 0.0

        # PARAMETER KALIBRASI ARAH LIDAR
        self._lidar_mirror = False
        self._lidar_yaw_offset = 0.0

    def init(self, webots_node, properties):
        print('[BR DRIVER] init() dimulai', flush=True)

        if not rclpy.ok():
            rclpy.init()

        self._robot = getattr(webots_node, 'robot', None)
        self._node = rclpy.create_node('br_webots_driver')

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
                print('[BR DRIVER] ✅ InertialUnit AKTIF', flush=True)

            # 3. Setup LIDAR
            self._range_sensor = self._robot.getDevice('lidar')
            if self._range_sensor is not None:
                self._range_sensor.enable(timestep_ms)
                print('[BR DRIVER] ✅ Lidar AKTIF', flush=True)
            else:
                print('[BR DRIVER] ⚠️ Lidar tidak ditemukan di Proto!', flush=True)

            # 4. Setup Gripper Utama (Hanya Kiri & Kanan, tanpa Tip)
            self._grip_left_motor = self._robot.getDevice('gripper_left_motor')
            self._grip_right_motor = self._robot.getDevice('gripper_right_motor')
            if self._grip_left_motor and self._grip_right_motor:
                self._grip_left_motor.setPosition(0.0)
                self._grip_right_motor.setPosition(0.0)
                print('[BR DRIVER] ✅ Gripper Utama AKTIF', flush=True)
            else:
                print('[BR DRIVER] ⚠️ Motor Gripper tidak ditemukan di Proto!', flush=True)

            # 5. Setup Z-Axis (Slider / Lift)
            self._z_axis_motor = self._robot.getDevice('z_axis_motor')
            self._z_axis_sensor = self._robot.getDevice('z_axis_sensor')
            if self._z_axis_motor and self._z_axis_sensor:
                self._z_axis_motor.setPosition(0.0)  # 0.0 = posisi awal bawah
                self._z_axis_sensor.enable(timestep_ms)
                print('[BR DRIVER] ✅ Z-Axis Slider AKTIF', flush=True)
            else:
                print('[BR DRIVER] ⚠️ Motor/Sensor Z-Axis tidak ditemukan di Proto!', flush=True)

        self._target_velocity = {'fl': 0.0, 'fr': 0.0, 'rl': 0.0, 'rr': 0.0}

        # 6. Subscribers
        self._cmd_vel_subscription = self._node.create_subscription(
            Twist, '/br/cmd_vel', self._cmd_vel_callback, 10)

        self._gripper_sub = self._node.create_subscription(
            Float64, '/br/gripper_cmd', self._gripper_callback, 10)

        self._z_axis_sub = self._node.create_subscription(
            Float64, '/br/z_axis_cmd', self._z_axis_callback, 10)

        # 7. Publishers
        self._joint_state_publisher = self._node.create_publisher(JointState, '/br/joint_states', 10)
        self._odom_publisher = self._node.create_publisher(Odometry, '/br/odom', 10)

        # Publisher untuk Lidar
        self._lidar_pub = self._node.create_publisher(PointCloud2, '/br/lidar/point_cloud', 10)

        # Publisher untuk Range
        self._range_pub = self._node.create_publisher(Range, '/br/range_sensor/range', 10)

        # Publisher untuk Visualisasi 3D (MarkerArray)
        self._marker_pub = self._node.create_publisher(MarkerArray, '/br/robot_markers', 10)

        self._tf_broadcaster = TransformBroadcaster(self._node)
        self._node.get_logger().info('BR driver aktif (Siap menerima perintah Gripper & Z-Axis)')

    # --- CALLBACK GRIPPER UTAMA ---
    def _gripper_callback(self, msg):
        if self._grip_left_motor and self._grip_right_motor:
            target_angle = msg.data * 0.6
            self._grip_left_motor.setPosition(target_angle)
            self._grip_right_motor.setPosition(-target_angle)
            self._grip_state = msg.data
            self._node.get_logger().info(f'BR Gripper: {target_angle}')
        else:
            self._node.get_logger().error('Gagal gerakin Gripper BR: Motor gak ketemu!')

    # --- CALLBACK Z-AXIS (SLIDER) ---
    def _z_axis_callback(self, msg):
        if self._z_axis_motor:
            target_height = msg.data
            self._z_axis_motor.setPosition(target_height)
            self._node.get_logger().info(f'BR Z-Axis Target: {target_height}')
        else:
            self._node.get_logger().error('Gagal gerakin Z-Axis BR: Motor gak ketemu!')

    # ====================================================
    # LOGIKA RODA & ODOMETRI
    # ====================================================

    def _normalize_wheel_speeds(self, fl, fr, rl, rr):
        max_abs = max(abs(fl), abs(fr), abs(rl), abs(rr), 1.0)
        if max_abs > self._max_wheel_speed:
            scale = self._max_wheel_speed / max_abs
            fl *= scale
            fr *= scale
            rl *= scale
            rr *= scale
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
        angles = {}
        velocities = {}
        for wheel_name, sensor in self._sensors.items():
            angle = sensor.getValue()
            if math.isnan(angle):
                angle = self._last_wheel_angle[wheel_name]
            velocities[wheel_name] = (angle - self._last_wheel_angle[wheel_name]) / dt
            self._last_wheel_angle[wheel_name] = angle
            angles[wheel_name] = angle
        return angles, velocities

    def _publish_joint_states(self, stamp, angles, velocities):
        msg = JointState()
        msg.header.stamp = stamp

        # NAMA JOINT: 4 Roda + Gripper Kiri + Gripper Kanan + Z-Axis Slider
        msg.name = [
            self._wheel_joint_names['fl'], self._wheel_joint_names['fr'],
            self._wheel_joint_names['rl'], self._wheel_joint_names['rr'],
            'gripper_left_joint', 'gripper_right_joint', 'z_axis_joint'
        ]

        msg.position = [
            angles['fl'], angles['fr'], angles['rl'], angles['rr'],
            0.0, 0.0, self._z_position
        ]

        msg.velocity = [
            velocities['fl'], velocities['fr'], velocities['rl'], velocities['rr'],
            0.0, 0.0, 0.0
        ]

        self._joint_state_publisher.publish(msg)

    # ====================================================
    # LOGIKA VISUAL MARKER UNTUK FOXGLOVE (GAYA SAMA DENGAN TR)
    # ====================================================
    def _make_marker(self, stamp, marker_id, marker_type, pos, scale, color,
                     quat=(0.0, 0.0, 0.0, 1.0)):
        """Helper pembuat marker supaya kode visual lebih ringkas & konsisten."""
        m = Marker()
        m.header.frame_id = 'br/base_footprint'
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

        chassis_len = 0.60
        chassis_wid = 0.50
        chassis_h = 0.10
        chassis_z = r + chassis_h / 2.0   # dasar chassis sejajar sumbu roda (z = 0.15)
        chassis_top = r + chassis_h       # z = 0.20

        pillar_x = 0.01
        pillar_y = 0.20
        pillar_top = 0.91
        pillar_h = pillar_top - r         # tiang berdiri dari dasar chassis ke atas
        pillar_z = r + pillar_h / 2.0

        slider_z = 0.08 + self._z_position  # tinggi palang slider (ikut sensor Z-Axis)

        # Warna
        c_body = (0.10, 0.25, 0.80, 1.0)
        c_deck = (0.07, 0.17, 0.60, 1.0)
        c_tire = (0.05, 0.05, 0.05, 1.0)
        c_hub = (0.70, 0.70, 0.72, 1.0)
        c_front = (1.00, 0.80, 0.10, 1.0)
        c_grip = (0.95, 0.50, 0.10, 1.0)
        c_pillar = (0.62, 0.64, 0.68, 1.0)
        c_dark = (0.12, 0.12, 0.14, 1.0)
        c_slider = (0.20, 0.55, 0.95, 1.0)

        # 1. Chassis + plat atas
        markers.append(self._make_marker(
            stamp, 0, Marker.CUBE,
            (0.0, 0.0, chassis_z),
            (chassis_len, chassis_wid, chassis_h), c_body))

        markers.append(self._make_marker(
            stamp, 1, Marker.CUBE,
            (0.0, 0.0, chassis_top + 0.005),
            (chassis_len - 0.06, chassis_wid - 0.06, 0.01), c_deck))

        # 2. Penanda depan robot (strip kuning di bumper depan)
        markers.append(self._make_marker(
            stamp, 2, Marker.CUBE,
            (chassis_len / 2.0 + 0.005, 0.0, chassis_z + 0.02),
            (0.01, chassis_wid, 0.04), c_front))

        # 3. Roda: sumbu silinder sejajar sumbu Y (putar 90 derajat terhadap X)
        #    dan pusatnya setinggi jari-jari roda supaya menapak tepat di lantai.
        s45 = math.sqrt(0.5)
        wheel_quat = (s45, 0.0, 0.0, s45)

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

        # 4. Tiang slider kiri & kanan + dudukan bawah + tutup atas
        for pid, base_id, cap_id, py in ((30, 32, 34, +pillar_y),
                                         (31, 33, 35, -pillar_y)):
            markers.append(self._make_marker(
                stamp, pid, Marker.CYLINDER,
                (pillar_x, py, pillar_z),
                (0.05, 0.05, pillar_h), c_pillar))
            markers.append(self._make_marker(
                stamp, base_id, Marker.CYLINDER,
                (pillar_x, py, chassis_top + 0.01),
                (0.09, 0.09, 0.02), c_dark))
            markers.append(self._make_marker(
                stamp, cap_id, Marker.CYLINDER,
                (pillar_x, py, pillar_top + 0.01),
                (0.07, 0.07, 0.02), c_dark))

        # 5. Palang slider (Z-Axis) + carriage di tiang, ikut naik/turun
        markers.append(self._make_marker(
            stamp, 40, Marker.CUBE,
            (pillar_x, 0.0, slider_z),
            (0.05, 0.46, 0.06), c_slider))
        markers.append(self._make_marker(
            stamp, 41, Marker.CUBE,
            (pillar_x, +pillar_y, slider_z),
            (0.08, 0.08, 0.09), c_dark))
        markers.append(self._make_marker(
            stamp, 42, Marker.CUBE,
            (pillar_x, -pillar_y, slider_z),
            (0.08, 0.08, 0.09), c_dark))

        # 6. Lengan penghubung palang -> gripper, lalu jari gripper kiri & kanan
        markers.append(self._make_marker(
            stamp, 50, Marker.CUBE,
            (0.095, +0.22, slider_z),
            (0.12, 0.04, 0.06), c_slider))
        markers.append(self._make_marker(
            stamp, 51, Marker.CUBE,
            (0.095, -0.22, slider_z),
            (0.12, 0.04, 0.06), c_slider))
        markers.append(self._make_marker(
            stamp, 52, Marker.CUBE,
            (0.30, +0.22, slider_z),
            (0.30, 0.04, 0.13), c_grip))
        markers.append(self._make_marker(
            stamp, 53, Marker.CUBE,
            (0.30, -0.22, slider_z),
            (0.30, 0.04, 0.13), c_grip))

        marker_array = MarkerArray()
        # Hapus marker lama (id 0-9 dari versi sebelumnya) supaya tidak muncul ganda
        clear = Marker()
        clear.header.frame_id = 'br/base_footprint'
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
        msg_range.header.frame_id = 'br/lidar'
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
            msg_pc.header.frame_id = 'br/lidar'
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
        odom.header.frame_id = 'br/odom'
        odom.child_frame_id = 'br/base_footprint'
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
        tf.header.frame_id = 'br/odom'
        tf.child_frame_id = 'br/base_footprint'
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

            # Ambil posisi Z-Axis dari sensor (jika ada)
            if self._z_axis_sensor is not None:
                self._z_position = self._z_axis_sensor.getValue()
                if math.isnan(self._z_position):
                    self._z_position = 0.0

            self._publish_joint_states(stamp, angles, velocities)
            self._update_odometry(stamp, velocities)
            self._publish_lidar(stamp)
            self._publish_markers(stamp)

            # TF Lidar
            tf_lidar = TransformStamped()
            tf_lidar.header.stamp = stamp
            tf_lidar.header.frame_id = 'br/base_footprint'
            tf_lidar.child_frame_id = 'br/lidar'

            tf_lidar.transform.translation.x = 0.21
            tf_lidar.transform.translation.y = 0.04
            tf_lidar.transform.translation.z = 0.14
            tf_lidar.transform.rotation.w = 1.0

            self._tf_broadcaster.sendTransform(tf_lidar)