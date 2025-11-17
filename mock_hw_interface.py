#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Transform, Vector3, Quaternion
from std_msgs.msg import Header
import time

from leitungssatz_interfaces.srv import SetCartTarget, SetForceTarget, SetFreedrive

class MockHW(Node):
    def __init__(self):
        super().__init__('mock_hw_interface')
        self.get_logger().info('Starting mock_hw_interface')

        # current tcp pose (TransformStamped)
        self.tcp = TransformStamped()
        self.tcp.header.frame_id = 'base'
        self.tcp.child_frame_id = 'tcp'
        self.tcp.transform = Transform()
        self.tcp.transform.translation = Vector3(x=0.5, y=0.0, z=0.5)
        self.tcp.transform.rotation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        # publishers
        self.pub_tcp = self.create_publisher(TransformStamped, '/ur_hardware_interface/tcp_pose', 10)
        self.timer = self.create_timer(0.1, self.publish_tcp)

        # services
        self.srv_freedrive = self.create_service(SetFreedrive, '/ur_hardware_interface/set_freedrive', self.handle_freedrive)
        self.srv_cart = self.create_service(SetCartTarget, '/ur_hardware_interface/set_cart_target', self.handle_cart)
        self.srv_force = self.create_service(SetForceTarget, '/ur_hardware_interface/set_force_mode', self.handle_force)

        self.get_logger().info('Mock services ready: set_freedrive, set_cart_target, set_force_mode')

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
        # update current tcp pose to request
        self.tcp.transform = g
        response.success = True
        return response

    def handle_force(self, request, response):
        # request.io, request.frame, request.selection_vector, request.wrench, request.type, request.limits
        self.get_logger().info(f"SetForceTarget called: io={request.io}, type={int(request.type)}, selection={list(request.selection_vector)}, wrench={list(request.wrench)}")
        response.success = True
        return response

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

'''
# Mock HW Interface — README

Purpose
- This small mock node simulates the robot hardware API used by the point_recording program.
- It lets you test point_recording and hw_interface service calls without a real UR5e robot.

What it does (simple)
- Provides three ROS2 service servers:
  - `/ur_hardware_interface/set_freedrive` — start/stop freedrive (logs request, returns success).
  - `/ur_hardware_interface/set_cart_target` — move command in cartesian space (logs request, updates tcp pose, returns success).
  - `/ur_hardware_interface/set_force_mode` — start/stop force mode (logs request, returns success).
- Publishes `/ur_hardware_interface/tcp_pose` (TransformStamped) at 10 Hz so point_recording reads a pose.
- Logs every incoming request to the terminal for easy observation.

How it works (brief)
- When a service request arrives, the mock prints the request fields and returns a success response.
- For cartesian moves, it updates the internal `tcp` transform to match the requested goal. That makes `/tcp_pose` reflect the new pose immediately.
- The node is stateless beyond the current `tcp` pose; it does not simulate physics, timing delays, collisions, or real sensor noise.

How to use (quick)
1. Build and source the workspace:
   source /opt/ros/humble/setup.bash
   source ~/Desktop/ls_ros2/install/setup.bash
2. Run the mock node:
   python3 ~/Desktop/ls_ros2/mock_hw_interface.py
3. In another terminal run point_recording and choose menu option 4 (move remotely).
4. Watch the mock terminal: each key press triggers a `SetCartTarget` log line. `/tcp_pose` updates too.

What to expect
- Console logs like:
  "SetCartTarget called: mode=1, speed=0.25, async=False"
  "SetFreedrive called: io=True, feature=1, free_axes=[1,1,1,1,1,1]"
- `/ur_hardware_interface/tcp_pose` will publish the latest transform you sent with `SetCartTarget`.
- Successful responses are always returned (mock returns `success = True`).

How to evaluate results (short)
- Confirm point_recording sends expected service calls:
  - Observe logs for SetCartTarget, SetFreedrive, SetForceTarget.
- Confirm tcp_pose changes when moves are sent:
  - ros2 topic echo /ur_hardware_interface/tcp_pose
- For each user action in the point_recording UI, there should be a corresponding log entry in the mock terminal.

Limitations (important)
- No real robot dynamics, no collision detection, no real force feedback.
- Force-mode behavior is synthetic. Do not assume real robot will behave the same under contact.

Suggested note for your thesis
- "A mock hardware interface was used to emulate the UR RTDE API. This allowed safe verification of point_recording service calls and pose handling without hardware. The mock logged all calls and updated TCP pose on SetCartTarget requests."


'''