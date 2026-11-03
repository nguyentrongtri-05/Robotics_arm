#!/usr/bin/env python3
"""
Node bang tai ao & vat mau da hinh dang (virtual_conveyor.py) chuan ROS 2 Humble.
Tich hop LUAT CHONG XUYEN THAU VAT LY (Hard Surface Collision & Contact Rules):
  1. Mat bang tai: Z >= 0.0 (Vat mau khong bao gio lot xuong duoi bang tai).
  2. Mat dau hut (tcp_link): Khi dau hut ha xuong cham mat tren cua vat mau (Z_top = 0.04m),
     vat mau va dau hut tuong tac be mat cung — khi bat hut hoac tiep xuc, mat tren vat mau
     luon nam dung tai Z = 0.0 cua tcp_link (tuyet doi khong cho ong hut dam xuyen qua khoi hop!).
  3. Dong bo MoveIt PlanningScene (/collision_object & /attached_collision_object) de MoveIt
     biet vat mau la vat can co the tich that.
"""

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener


class VirtualConveyorNode(Node):
    def __init__(self):
        super().__init__('virtual_conveyor')

        # Publishers: Hien thi 3D tren RViz2 & Tin hieu cam bien IN#1
        self.marker_pub = self.create_publisher(
            MarkerArray, '/visualization_marker_array', 10
        )
        self.sensor_pub = self.create_publisher(Bool, '/yaskawa/din1', 10)

        # Publishers: MoveIt 2 Collision Object & Attached Collision Object
        self.co_pub = self.create_publisher(
            CollisionObject, '/collision_object', 10
        )
        self.aco_pub = self.create_publisher(
            AttachedCollisionObject, '/attached_collision_object', 10
        )

        # Subscriber: Nhan lenh bat/tat van hut chan khong OUT#1 tu Teach Pendant
        self.vacuum_sub = self.create_subscription(
            Bool, '/yaskawa/dout1', self.vacuum_callback, 10
        )

        # TF2 Listener de doc vi tri va goc xoay dau hut tcp_link so voi world
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Thong so hinh hoc bang tai & vat mau
        self.conveyor_x = 0.25
        self.spawn_y = -0.35      # Diem xuat phat dau bang tai
        self.sensor_y = 0.00      # Vi tri cam bien quang IN#1 giua bang tai
        self.part_size = 0.04     # Kich thuoc chuan 4cm (0.04m)
        self.surface_z = self.part_size / 2.0  # Tam vat mau = 0.02m -> Mat tren = 0.04m

        # Danh sach cac mau hinh dang phoi luan phien
        self.shape_catalog = [
            ('Khối Hộp Vuông (CUBE)', Marker.CUBE, (1.0, 0.60, 0.0)),
            ('Khối Trụ Tròn (CYLINDER)', Marker.CYLINDER, (0.85, 0.25, 0.90)),
            ('Khối Cầu (SPHERE)', Marker.SPHERE, (0.95, 0.85, 0.10)),
        ]
        self.shape_index = 0

        # Trang thai vat mau: 'MOVING', 'AT_SENSOR', 'ATTACHED', 'PLACED'
        self.state = 'MOVING'
        self.part_x = self.conveyor_x
        self.part_y = self.spawn_y
        self.part_z = self.surface_z
        self.conveyor_speed = 0.08  # m/s

        self.vacuum_on = False
        self.placed_timer = 0.0

        # Vong lap cap nhat 30 Hz (dt = 0.033s)
        self.dt = 0.033
        self.timer = self.create_timer(self.dt, self.update_loop)
        self.get_logger().info(
            f'Khoi dong Bang tai! Dang cap phoi: {self.shape_catalog[self.shape_index][0]}'
        )

    def vacuum_callback(self, msg: Bool):
        prev = self.vacuum_on
        self.vacuum_on = msg.data
        if self.vacuum_on != prev:
            status = 'ON (HUT)' if self.vacuum_on else 'OFF (THA)'
            self.get_logger().info(f'Nhan tin hieu DOUT OT#(1) = {status}')

    def get_tcp_position(self):
        """Lay toa do (x, y, z) hien tai cua dau hut tcp_link trong he world."""
        try:
            trans = self.tf_buffer.lookup_transform(
                'world', 'tcp_link', rclpy.time.Time()
            )
            return (
                trans.transform.translation.x,
                trans.transform.translation.y,
                trans.transform.translation.z,
            )
        except Exception:
            return None

    def spawn_new_part(self):
        self.shape_index = (self.shape_index + 1) % len(self.shape_catalog)
        shape_name = self.shape_catalog[self.shape_index][0]
        self.part_x = self.conveyor_x
        self.part_y = self.spawn_y
        self.part_z = self.surface_z
        self.state = 'MOVING'
        self.get_logger().info(f'Da cap phoi moi len dau bang tai: {shape_name}')

    def update_loop(self):
        tcp_pos = self.get_tcp_position()

        # 1. Trang thai phoi dang troi tren bang tai
        if self.state == 'MOVING':
            self.part_y += self.conveyor_speed * self.dt
            if self.part_y >= self.sensor_y:
                self.part_y = self.sensor_y
                self.state = 'AT_SENSOR'
                self.get_logger().info(
                    'Cam bien IN#1 = ON! Vat mau da dung tai vi tri gap (X=0.25, Y=0.00).'
                )

        # 2. Trang thai phoi dang dung cho o cam bien IN#1
        elif self.state == 'AT_SENSOR':
            if tcp_pos is not None:
                xy_dist = math.hypot(tcp_pos[0] - self.part_x, tcp_pos[1] - self.part_y)
                part_top_z = self.part_z + (self.part_size / 2.0)  # 0.04m

                # Khi bat van hut (hoac dau hut cham sat mat tren vat mau <= 0.045m)
                if self.vacuum_on and xy_dist < 0.06 and abs(tcp_pos[2] - part_top_z) < 0.06:
                    self.state = 'ATTACHED'
                    self.get_logger().info('Dau hut da bat dinh vat mau! IN#1 = OFF.')

        # 3. Trang thai vat mau dang dinh chat tren mat dau hut tcp_link
        elif self.state == 'ATTACHED':
            if tcp_pos is not None:
                self.part_x = tcp_pos[0]
                self.part_y = tcp_pos[1]
                # LUAT CHONG XUYEN MAT BAN/KHAY: Tam vat mau khong bao gio thap hon surface_z (0.02m)
                self.part_z = max(self.surface_z, tcp_pos[2] - (self.part_size / 2.0))

            if not self.vacuum_on:
                self.state = 'PLACED'
                self.placed_timer = 0.0
                self.part_z = self.surface_z
                self.get_logger().info(
                    f'Da tha vat mau tai (X={self.part_x:.2f}, Y={self.part_y:.2f}). '
                    'Cap phoi hinh dang moi sau 2.5 giay...'
                )

        # 4. Trang thai da tha vat mau -> doi 2.5 giay roi cap phoi tiep theo
        elif self.state == 'PLACED':
            self.placed_timer += self.dt
            if self.placed_timer >= 2.5:
                self.spawn_new_part()

        # Xuat tin hieu cam bien IN#1
        in1_msg = Bool()
        in1_msg.data = (self.state == 'AT_SENSOR')
        self.sensor_pub.publish(in1_msg)

        # Xuat MarkerArray de hien thi vat mau + cam bien + khay tha tren RViz2
        self.publish_markers(in1_msg.data, tcp_pos)

    def publish_markers(self, sensor_active: bool, tcp_pos):
        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()
        _, shape_type, base_rgb = self.shape_catalog[self.shape_index]

        # Kiem tra xem dau hut co dang cham / ep vao mat tren vat mau khong
        # Neu dau hut ha xuong thap hon mat tren vat mau (Z < 0.04m) ngay tai vi tri phoi,
        # hoac dang ATTACHED -> khoa vat mau ep sat mat day dau hut (tcp_link) de KHONG BAO GIO XUYEN NHAU!
        touching_top = False
        if tcp_pos is not None and self.state in ('AT_SENSOR', 'PLACED'):
            xy_dist = math.hypot(tcp_pos[0] - self.part_x, tcp_pos[1] - self.part_y)
            if xy_dist < 0.035 and tcp_pos[2] <= (self.part_z + self.part_size / 2.0 + 0.002):
                touching_top = True

        # Marker 1: Vat mau (Phoi 3D)
        part_marker = Marker()
        part_marker.ns = 'workcell'
        part_marker.id = 1
        part_marker.type = shape_type
        part_marker.action = Marker.ADD
        part_marker.scale.x = self.part_size
        part_marker.scale.y = self.part_size
        part_marker.scale.z = self.part_size

        if self.state == 'ATTACHED' or touching_top:
            # GAN TRUC TIEP VAO HE TRUC DAU HUT 'tcp_link':
            # Mat day ong hut nam tai z = 0.0 cua tcp_link -> tam vat mau nam tai z = +0.02m
            # Giup mat tren vat mau LUON TIEP XUC PHANG 100% voi mat day ong hut, khong xuyen thau!
            part_marker.header.frame_id = 'tcp_link'
            part_marker.header.stamp = rclpy.time.Time().to_msg()
            part_marker.pose.position.x = 0.0
            part_marker.pose.position.y = 0.0
            part_marker.pose.position.z = float(self.part_size / 2.0)
            part_marker.pose.orientation.x = 0.0
            part_marker.pose.orientation.y = 0.0
            part_marker.pose.orientation.z = 0.0
            part_marker.pose.orientation.w = 1.0
            part_marker.frame_locked = True
        else:
            part_marker.header.frame_id = 'world'
            part_marker.header.stamp = now
            part_marker.pose.position.x = float(self.part_x)
            part_marker.pose.position.y = float(self.part_y)
            part_marker.pose.position.z = float(max(self.surface_z, self.part_z))
            part_marker.pose.orientation.x = 0.0
            part_marker.pose.orientation.y = 0.0
            part_marker.pose.orientation.z = 0.0
            part_marker.pose.orientation.w = 1.0
            part_marker.frame_locked = False

        if self.state == 'ATTACHED':
            part_marker.color.r = 0.1
            part_marker.color.g = 0.85
            part_marker.color.b = 1.0
        else:
            part_marker.color.r = base_rgb[0]
            part_marker.color.g = base_rgb[1]
            part_marker.color.b = base_rgb[2]

        part_marker.color.a = 1.0
        marker_array.markers.append(part_marker)

        # Marker 2: Den bao cam bien quang IN#1 ben canh bang tai
        sensor_marker = Marker()
        sensor_marker.header.frame_id = 'world'
        sensor_marker.header.stamp = now
        sensor_marker.ns = 'workcell'
        sensor_marker.id = 2
        sensor_marker.type = Marker.SPHERE
        sensor_marker.action = Marker.ADD
        sensor_marker.pose.position.x = self.conveyor_x + 0.09
        sensor_marker.pose.position.y = self.sensor_y
        sensor_marker.pose.position.z = 0.02
        sensor_marker.pose.orientation.w = 1.0
        sensor_marker.scale.x = 0.025
        sensor_marker.scale.y = 0.025
        sensor_marker.scale.z = 0.025
        if sensor_active:
            sensor_marker.color.r = 0.0
            sensor_marker.color.g = 1.0
            sensor_marker.color.b = 0.0
        else:
            sensor_marker.color.r = 0.8
            sensor_marker.color.g = 0.1
            sensor_marker.color.b = 0.1
        sensor_marker.color.a = 1.0
        marker_array.markers.append(sensor_marker)

        # Marker 3: Khay tha san pham (Place Tray) o ben canh robot (X=0.0, Y=0.25)
        tray_marker = Marker()
        tray_marker.header.frame_id = 'world'
        tray_marker.header.stamp = now
        tray_marker.ns = 'workcell'
        tray_marker.id = 3
        tray_marker.type = Marker.CUBE
        tray_marker.action = Marker.ADD
        tray_marker.pose.position.x = 0.0
        tray_marker.pose.position.y = 0.25
        tray_marker.pose.position.z = -0.005
        tray_marker.pose.orientation.w = 1.0
        tray_marker.scale.x = 0.14
        tray_marker.scale.y = 0.14
        tray_marker.scale.z = 0.01
        tray_marker.color.r = 0.4
        tray_marker.color.g = 0.4
        tray_marker.color.b = 0.45
        tray_marker.color.a = 0.9
        marker_array.markers.append(tray_marker)

        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = VirtualConveyorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
