#!/usr/bin/env python3
"""
Task 2.1 + 2.2 - WebSocket Waypoint Navigation + Sphere Color Detection
"""

import math
import threading
import queue
import json
import asyncio

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

import websockets


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class WaypointNavigator(Node):

    DIST_TOLERANCE = 0.10
    YAW_TOLERANCE = 0.05
    MAX_LINEAR_SPEED = 0.20
    MAX_ANGULAR_SPEED = 0.8
    HEADING_LOCK_ANGLE = 0.3

    WS_URI = "ws://localhost:8765"
    CAMERA_TOPIC = "/oakd/rgb/preview/image_raw"

    SCAN_ANGULAR_SPEED = 0.3
    SCAN_MAX_ANGLE = 2.0 * math.pi

    COLOR_RANGES = {
        'RED': [
            (np.array([0, 100, 80]), np.array([10, 255, 255])),
            (np.array([170, 100, 80]), np.array([180, 255, 255])),
        ],
        'GREEN': [
            (np.array([40, 80, 60]), np.array([85, 255, 255])),
        ],
        'YELLOW': [
            (np.array([20, 100, 100]), np.array([35, 255, 255])),
        ],
    }
    MIN_CONTOUR_AREA = 50

    STATE_ROTATE_TO_HEADING = "ROTATE_TO_HEADING"
    STATE_DRIVE_TO_POSITION = "DRIVE_TO_POSITION"
    STATE_ROTATE_TO_FINAL_YAW = "ROTATE_TO_FINAL_YAW"
    STATE_SCANNING = "SCANNING"
    STATE_DONE = "DONE"

    def __init__(self):
        super().__init__('waypoint_navigator')

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.camera_sub = self.create_subscription(
            Image, self.CAMERA_TOPIC, self.camera_callback, 10)

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.odom_received = False

        self.waypoint_queue = queue.Queue()
        self.waypoints_loaded = False

        self.waypoints = []
        self.current_wp_index = 0
        self.state = None

        self.cv_bridge = CvBridge()
        self.scan_prev_yaw = None
        self.scan_accum_angle = 0.0
        self.scan_best_areas = {c: 0.0 for c in self.COLOR_RANGES}
        self.detection_finalized = False

        self.ws_thread = threading.Thread(target=self._run_ws_client, daemon=True)
        self.ws_thread.start()

        self.control_timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Waypoint Navigator started. Waiting for odom + waypoints...')

    def _run_ws_client(self):
        asyncio.run(self._ws_client_coro())

    async def _ws_client_coro(self):
        while rclpy.ok():
            try:
                async with websockets.connect(self.WS_URI) as ws:
                    self.get_logger().info(f'Connected to {self.WS_URI}')
                    async for message in ws:
                        if self.waypoints_loaded:
                            continue
                        try:
                            data = json.loads(message)
                            wps = data.get('waypoints', [])
                            if wps:
                                self.waypoint_queue.put(wps)
                                self.waypoints_loaded = True
                                self.get_logger().info(
                                    f'Received {len(wps)} waypoints from server.')
                        except json.JSONDecodeError:
                            self.get_logger().warn('Received malformed JSON, skipping.')
            except Exception as e:
                self.get_logger().warn(f'WebSocket connection failed ({e}); retrying in 2s...')
                await asyncio.sleep(2.0)

    def odom_callback(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.odom_received = True

    def camera_callback(self, msg: Image):
        if self.state != self.STATE_SCANNING:
            return

        frame = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        for color_name, ranges in self.COLOR_RANGES.items():
            mask = None
            for lower, upper in ranges:
                m = cv2.inRange(hsv, lower, upper)
                mask = m if mask is None else cv2.bitwise_or(mask, m)
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)

            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                area = cv2.contourArea(max(contours, key=cv2.contourArea))
                if area > self.scan_best_areas[color_name]:
                    self.scan_best_areas[color_name] = area

    def finalize_detection(self):
        self.get_logger().info(f'Scan complete. Best areas seen (px^2): {self.scan_best_areas}')
        best_color = max(self.scan_best_areas, key=self.scan_best_areas.get)
        best_area = self.scan_best_areas[best_color]
        if best_area >= self.MIN_CONTOUR_AREA:
            self.get_logger().info(
                f'>>> LARGEST SPHERE COLOR: {best_color} (area={best_area:.1f} px^2) <<<')
        else:
            self.get_logger().warn(
                'No sphere detected above minimum area threshold during scan - '
                'check camera framing or HSV color ranges.')
        self.detection_finalized = True

    def control_loop(self):
        while not self.waypoint_queue.empty():
            wps = self.waypoint_queue.get()
            self.waypoints = wps
            self.current_wp_index = 0
            self.state = self.STATE_ROTATE_TO_HEADING
            self.get_logger().info('Waypoint list loaded. Beginning navigation.')

        if not self.odom_received or self.state is None:
            return

        if self.state == self.STATE_DONE:
            self.stop_robot()
            return

        if self.state == self.STATE_SCANNING:
            self._do_scan_step()
            return

        goal = self.waypoints[self.current_wp_index]
        goal_x, goal_y, goal_yaw = goal['x'], goal['y'], goal['yaw']

        dx = goal_x - self.current_x
        dy = goal_y - self.current_y
        distance_error = math.hypot(dx, dy)
        heading_to_goal = math.atan2(dy, dx)
        heading_error = normalize_angle(heading_to_goal - self.current_yaw)
        final_yaw_error = normalize_angle(goal_yaw - self.current_yaw)

        twist = Twist()

        if self.state == self.STATE_ROTATE_TO_HEADING:
            if abs(heading_error) > self.HEADING_LOCK_ANGLE:
                twist.angular.z = self._clamp(
                    2.0 * heading_error, -self.MAX_ANGULAR_SPEED, self.MAX_ANGULAR_SPEED)
            else:
                self.state = self.STATE_DRIVE_TO_POSITION

        elif self.state == self.STATE_DRIVE_TO_POSITION:
            if distance_error > self.DIST_TOLERANCE:
                twist.linear.x = self._clamp(
                    0.5 * distance_error, 0.0, self.MAX_LINEAR_SPEED)
                twist.angular.z = self._clamp(
                    1.0 * heading_error, -self.MAX_ANGULAR_SPEED, self.MAX_ANGULAR_SPEED)
                if abs(heading_error) > self.HEADING_LOCK_ANGLE * 2:
                    self.state = self.STATE_ROTATE_TO_HEADING
            else:
                self.state = self.STATE_ROTATE_TO_FINAL_YAW

        elif self.state == self.STATE_ROTATE_TO_FINAL_YAW:
            if abs(final_yaw_error) > self.YAW_TOLERANCE:
                twist.angular.z = self._clamp(
                    2.0 * final_yaw_error, -self.MAX_ANGULAR_SPEED, self.MAX_ANGULAR_SPEED)
            else:
                self.get_logger().info(
                    f'Reached waypoint {self.current_wp_index + 1}/{len(self.waypoints)} '
                    f'(x={goal_x:.2f}, y={goal_y:.2f}, yaw={goal_yaw:.2f})')
                self.current_wp_index += 1
                if self.current_wp_index >= len(self.waypoints):
                    self.get_logger().info(
                        'Final waypoint reached. Starting scan for spheres...')
                    self.state = self.STATE_SCANNING
                    self.scan_prev_yaw = self.current_yaw
                    self.scan_accum_angle = 0.0
                    self.scan_best_areas = {c: 0.0 for c in self.COLOR_RANGES}
                else:
                    self.state = self.STATE_ROTATE_TO_HEADING

        self.cmd_vel_pub.publish(twist)

    def _do_scan_step(self):
        delta = normalize_angle(self.current_yaw - self.scan_prev_yaw)
        self.scan_accum_angle += abs(delta)
        self.scan_prev_yaw = self.current_yaw

        if self.scan_accum_angle >= self.SCAN_MAX_ANGLE:
            self.stop_robot()
            self.state = self.STATE_DONE
            if not self.detection_finalized:
                self.finalize_detection()
            return

        twist = Twist()
        twist.angular.z = self.SCAN_ANGULAR_SPEED
        self.cmd_vel_pub.publish(twist)

    def stop_robot(self):
        try:
            self.cmd_vel_pub.publish(Twist())
        except Exception:
            pass

    @staticmethod
    def _clamp(value, low, high):
        return max(low, min(high, value))


def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()