#!/usr/bin/env python3
"""
Mock HW Interface (expanded)

Purpose (short)
- Emulates minimal robot hardware for point_recording.
- Provides services used by point_recording, and stores TFs to disk so "move to tf" (option 5) works.
- Use for manual testing and thesis demos without a real robot.

What it provides
- Services:
  - /ur_hardware_interface/set_freedrive       (leitungssatz_interfaces/srv/SetFreedrive)
  - /ur_hardware_interface/set_cart_target     (leitungssatz_interfaces/srv/SetCartTarget)
  - /ur_hardware_interface/set_force_mode      (leitungssatz_interfaces/srv/SetForceTarget)
  - /store_tf                                   (leitungssatz/srv/AddTf2)  <-- saves TF to JSON and broadcasts it
- Topics:
  - /ur_hardware_interface/tcp_pose (TransformStamped) published at 10 Hz
  - publishes saved frames to /tf_static so tf2 lookup works for "move to tf"

How to use (very short)
1) In a terminal:
   source /opt/ros/humble/setup.bash
   source ~/Desktop/ls_ros2/install/setup.bash
   python3 ~/Desktop/ls_ros2/mock_hw_interface.py
2) In another terminal run point_recording and use options:
   - Option 4 (remote control) will call set_cart_target and change /tcp_pose.
   - Option 1 (save point) will call /store_tf and the mock will save and broadcast the frame.
   - Option 5 (move to tf) in point_recording uses tf2 lookup; saved frames are available via /tf_static.

Saved data
- Saved frames are written to ./saved_frames.json (in the directory where you run the mock).
- File structure: list of frames with parent, child, translation, rotation, relative.

Limitations
- No physics, no real timing or collision. Mock always returns success.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Transform, Vector3, Quaternion
import tf2_ros
import json
import os

from leitungssatz_interfaces.srv import SetCartTarget, SetForceTarget, SetFreedrive
from leitungssatz.srv import AddTf2

SAVED_FILE = os.path.join(os.getcwd(), "saved_frames.json")


class MockHW(Node):
    def __init__(self):
        super().__init__('mock_hw_interface')
        self.get_logger().info('Starting mock_hw_interface (expanded)')

        # current tcp pose (TransformStamped)
        self.tcp = TransformStamped()
        self.tcp.header.frame_id = 'base'
        self.tcp.child_frame_id = 'tcp'
        self.tcp.transform = Transform()
        self.tcp.transform.translation = Vector3(x=0.5, y=0.0, z=0.5)
        self.tcp.transform.rotation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        # publishers and tf broadcasters
        self.pub_tcp = self.create_publisher(TransformStamped, '/ur_hardware_interface/tcp_pose', 10)
        self.timer = self.create_timer(0.1, self.publish_tcp)
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        # services
        self.srv_freedrive = self.create_service(SetFreedrive, '/ur_hardware_interface/set_freedrive', self.handle_freedrive)
        self.srv_cart = self.create_service(SetCartTarget, '/ur_hardware_interface/set_cart_target', self.handle_cart)
        self.srv_force = self.create_service(SetForceTarget, '/ur_hardware_interface/set_force_mode', self.handle_force)
        self.srv_storetf = self.create_service(AddTf2, '/store_tf', self.handle_store_tf)

        # in-memory store of saved frames (child_frame_id -> TransformStamped)
        self.saved_frames = {}
        self._load_saved_frames()

        self.get_logger().info('Mock services ready: set_freedrive, set_cart_target, set_force_mode, store_tf')

    def publish_tcp(self):
        now = self.get_clock().now().to_msg()
        self.tcp.header.stamp = now
        self.pub_tcp.publish(self.tcp)

    def handle_freedrive(self, request, response):
        # request.io (bool), request.free_axes (int64[6]), request.feature (int8), request.custom_frame (Transform)
        self.get_logger().info(f"SetFreedrive called: io={request.io}, feature={int(request.feature)}, free_axes={list(request.free_axes)}")
        response.freedrive_status = 0
        response.success = True
        response.message = 'mock: freedrive ' + ('started' if request.io else 'stopped')
        return response

    def handle_cart(self, request, response):
        # request.mode, request.cartesian_goal (Transform), request.speed, acceleration, asynchronous
        self.get_logger().info(f"SetCartTarget called: mode={int(request.mode)}, speed={float(request.speed)}, async={bool(request.asynchronous)}")
        g = request.cartesian_goal
        # update current tcp pose to requested goal (instant)
        self.tcp.transform = g
        response.success = True
        return response

    def handle_force(self, request, response):
        # request.io, request.frame, request.selection_vector, request.wrench, request.type, request.limits
        self.get_logger().info(f"SetForceTarget called: io={request.io}, type={int(request.type)}, selection={list(request.selection_vector)}, wrench={list(request.wrench)}")
        response.success = True
        return response

    def handle_store_tf(self, request, response):
        # request.pose is TransformStamped, request.relative is bool
        pose_ts = request.pose
        parent = pose_ts.header.frame_id
        child = pose_ts.child_frame_id if pose_ts.child_frame_id else pose_ts.child_frame_id
        if not child:
            # fallback name if client sent empty child_frame_id
            child = f"saved_tf_{len(self.saved_frames)+1}"
            pose_ts.child_frame_id = child

        self.get_logger().info(f"AddTf2 called: parent={parent}, child={child}, relative={bool(request.relative)}")

        # store in memory and on disk
        self.saved_frames[child] = pose_ts
        self._save_frames_file()

        # broadcast as static transform so tfBuffer can find it
        # ensure timestamp is now
        pose_ts.header.stamp = self.get_clock().now().to_msg()
        self.static_broadcaster.sendTransform(pose_ts)
        response.success = True
        return response

    def _save_frames_file(self):
        out = []
        for child, ts in self.saved_frames.items():
            t = ts.transform
            out.append({
                "parent": ts.header.frame_id,
                "child": ts.child_frame_id,
                "translation": {"x": t.translation.x, "y": t.translation.y, "z": t.translation.z},
                "rotation": {"x": t.rotation.x, "y": t.rotation.y, "z": t.rotation.z, "w": t.rotation.w},
                "relative": False
            })
        try:
            with open(SAVED_FILE, 'w') as f:
                json.dump(out, f, indent=2)
            self.get_logger().info(f"Saved {len(out)} frames to {SAVED_FILE}")
        except Exception as e:
            self.get_logger().error(f"Failed to write saved frames: {e}")

    def _load_saved_frames(self):
        if not os.path.exists(SAVED_FILE):
            return
        try:
            with open(SAVED_FILE, 'r') as f:
                data = json.load(f)
            for item in data:
                ts = TransformStamped()
                ts.header.frame_id = item.get("parent", "base")
                ts.child_frame_id = item.get("child", "unknown")
                tr = item.get("translation", {})
                ro = item.get("rotation", {})
                ts.transform.translation.x = float(tr.get("x", 0.0))
                ts.transform.translation.y = float(tr.get("y", 0.0))
                ts.transform.translation.z = float(tr.get("z", 0.0))
                ts.transform.rotation.x = float(ro.get("x", 0.0))
                ts.transform.rotation.y = float(ro.get("y", 0.0))
                ts.transform.rotation.z = float(ro.get("z", 0.0))
                ts.transform.rotation.w = float(ro.get("w", 1.0))
                # stamp with now so tf2 considers it valid
                ts.header.stamp = self.get_clock().now().to_msg()
                self.saved_frames[ts.child_frame_id] = ts
            # broadcast all loaded frames
            if self.saved_frames:
                self.static_broadcaster.sendTransform(list(self.saved_frames.values()))
                self.get_logger().info(f"Loaded and broadcast {len(self.saved_frames)} saved frames from {SAVED_FILE}")
        except Exception as e:
            self.get_logger().error(f"Failed to load saved frames: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = MockHW()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.get_logger().info('Shutting down mock_hw_interface')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()