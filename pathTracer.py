import argparse
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from tf_transformations import quaternion_from_euler, euler_from_quaternion
import numpy as np
import time
import os
from math import sqrt, pi
from cv_bridge import CvBridge
import cv2
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


class GoalPosePublisher(Node):

    def __init__(self, load_filename, position_threshold, yaw_threshold, image_save_directory, odometry_topic, image_topic):
        super().__init__('goal_pose_publisher')
        self.load_filename = load_filename
        self.position_threshold = position_threshold
        self.yaw_threshold = yaw_threshold
        self.image_save_directory = image_save_directory
        self.current_position = None
        self.current_yaw = None
        self.positions = self.load_from_text_file(load_filename)

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.publisher = self.create_publisher(PoseStamped, '/goal_pose', qos_profile)
        self.pose_subscription = self.create_subscription(
            Odometry,
            odometry_topic,
            self.odometry_callback,
            qos_profile
        )
        self.image_subscription = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            qos_profile
        )

        self.bridge = CvBridge()
        self.current_image = None
        self.image_count = 0

        if not os.path.exists(self.image_save_directory):
            os.makedirs(self.image_save_directory)

    def load_from_text_file(self, filename):
        if not os.path.exists(filename):
            raise RuntimeError(f"Failed to open file for reading: {filename}")

        try:
            with open(filename, 'r') as file:
                pos_with_yaw = []
                for line in file:
                    data = line.split()
                    position = np.array([float(data[0]), float(data[1]), float(data[2])])
                    yaw = float(data[3])
                    pos_with_yaw.append((position, yaw))
                return pos_with_yaw
        except FileNotFoundError:
            raise RuntimeError(f"Failed to open file for reading: {filename}")

    def odometry_callback(self, msg):
        self.current_position = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])
        orientation_q = msg.pose.pose.orientation
        _, _, self.current_yaw = euler_from_quaternion([
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        ])


    def image_callback(self, msg):
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')

    def publish_goal_poses(self):
        for position, yaw in self.positions:
            goal_pose_msg = PoseStamped()
            goal_pose_msg.header.frame_id = "map"

            goal_pose_msg.pose.position.x = position[0]
            goal_pose_msg.pose.position.y = position[1]
            goal_pose_msg.pose.position.z = position[2]

            quat = quaternion_from_euler(0, 0, yaw)
            goal_pose_msg.pose.orientation.x = quat[0]
            goal_pose_msg.pose.orientation.y = quat[1]
            goal_pose_msg.pose.orientation.z = quat[2]
            goal_pose_msg.pose.orientation.w = quat[3]

            self.publisher.publish(goal_pose_msg)
            self.get_logger().info(f'Publishing goal pose: {position}, yaw: {yaw}')

            while not self.is_goal_reached(position, yaw):
                rclpy.spin_once(self, timeout_sec=1)
                self.publisher.publish(goal_pose_msg)
                time.sleep(1)

            time.sleep(1)
            self.save_image()

    def is_goal_reached(self, goal_position, goal_yaw):
        if self.current_position is None or self.current_yaw is None:
            return False

        position_distance = sqrt(np.sum((goal_position - self.current_position) ** 2))

        self.get_logger().info(f'Current yaw distance to goal: {goal_yaw} , {self.current_yaw}')
        yaw_distance = abs(goal_yaw - self.current_yaw)
        yaw_distance = min(yaw_distance, 2 * pi - yaw_distance)  # Normalize yaw difference

        #self.get_logger().info(f'Current position distance to goal: {position_distance}')
        #self.get_logger().info(f'Current yaw distance to goal: {yaw_distance}')

        return (position_distance <= self.position_threshold) and (yaw_distance <= self.yaw_threshold)

    def save_image(self):
        if self.current_image is not None:
            image_path = os.path.join(self.image_save_directory, f'image_{self.image_count:04d}.png')
            cv2.imwrite(image_path, self.current_image)
            self.get_logger().info(f'Saved image {self.image_count} to {image_path}')
            self.image_count += 1
        else:
            self.get_logger().warning('No image available to save')


def main():
    parser = argparse.ArgumentParser(description='Publish goal poses from a text file and save images when goal is reached.')
    parser.add_argument('load_filename', type=str, help='The path to the text file containing the goal poses to load.')
    parser.add_argument('image_save_directory', type=str, help='The directory to save images.')
    parser.add_argument('--position_threshold', type=float, default=0.3, help='The position distance threshold to consider the goal reached.')
    parser.add_argument('--yaw_threshold', type=float, default=3, help='The yaw distance threshold to consider the goal reached.')
    parser.add_argument('--odometry_topic', type=str, default='/lio_sam/mapping/odometry', help='The topic to subscribe for odometry data.')
    parser.add_argument('--image_topic', type=str, default='/kinect_camera/image_raw/compressed', help='The topic to subscribe for image data.')
    args = parser.parse_args()

    rclpy.init()
    goal_publisher = GoalPosePublisher(args.load_filename, args.position_threshold, args.yaw_threshold, args.image_save_directory, args.odometry_topic, args.image_topic)
    goal_publisher.publish_goal_poses()
    goal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()