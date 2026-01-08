#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node 
from aruco_opencv_msgs.msg import ArucoDetection 
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from enum import Enum
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2

class RobotControl(Node):
    def __init__(self):
        super().__init__('robotControl')

        self.bridge = CvBridge()

        self.delta = 0.03
        self.error_px_delta = 5
        self.dist_treshold = 0.25
        self.dist_home_treshold = 0.05
        self.initial_marker = None
        self.rotation_goal = None
        self.marker_id_goal = None
        self.goal_marker_error = float("inf")
        self.distance_from_marker = None
        self.image_published = False
        self.detecting_first_marker = False
        self.first_marker_id = None

        self.robot_state = RobotState.STARTING
        self.markers_detected = dict()
        self.unvisited_markers = []

        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.image_publisher = self.create_publisher(CompressedImage, 'camera/image_with_circle', 10)

        self.detections_subscriber = self.create_subscription(
            ArucoDetection,
            '/aruco_detections',
            self.detections_callback,
            10)
        self.odometry_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odometry_callback,
            10)
        self.image_subscriber = self.create_subscription(
            CompressedImage, 
            'camera/image/compressed', 
            self.image_callback, 
            10)

        self.timer = self.create_timer(0.007, self.control_robot)

        self.rotating_velocity = Twist()
        self.stop_movement = Twist()
        self.go_to_marker_velocity = Twist()
        self.go_home_velocity = Twist()
        self.robot_position = Point()
        self.rotating_velocity.angular.z = 0.3

        self.go_to_marker_velocity.linear.x = 0.3
        self.go_home_velocity.linear.x = -0.3
    
    def oriented_to_marker(self):
        return self.goal_marker_error < self.error_px_delta
    
    def control_robot(self):
        
        if self.robot_state == RobotState.STARTING:            
            self.velocity_publisher.publish(self.rotating_velocity)
            if self.first_marker_id is not None and not self.detecting_first_marker:
                self.robot_state = RobotState.SEARCH_FOR_MARKERS
                self.get_logger().info("started")

        elif self.robot_state == RobotState.SEARCH_FOR_MARKERS:
            if self.detecting_first_marker:
                self.velocity_publisher.publish(self.stop_movement)
                self.get_logger().info('stopped')

                self.unvisited_markers = list(self.markers_detected.keys())
                self.unvisited_markers.sort(reverse = True)
                self.robot_state = RobotState.SET_MARKER_GOAL
            else:
                self.velocity_publisher.publish(self.rotating_velocity)


        elif self.robot_state == RobotState.SET_MARKER_GOAL:
            if len(self.unvisited_markers) != 0:
                self.marker_id_goal = self.unvisited_markers.pop()
                self.get_logger().info(f'current marker id goal: {self.marker_id_goal}')
                
                self.robot_state = RobotState.ROTATE_TO_MARKER
            else:
                self.get_logger().info('visited all markers')
                exit()
        
        elif self.robot_state == RobotState.ROTATE_TO_MARKER:
            if not self.oriented_to_marker():
                self.velocity_publisher.publish(self.rotating_velocity) 
            else:
                self.velocity_publisher.publish(self.stop_movement)
                self.robot_state = RobotState.GO_TO_MARKER
        
        elif self.robot_state == RobotState.GO_TO_MARKER:
            self.velocity_publisher.publish(self.go_to_marker_velocity)

            if self.distance_from_marker and abs(self.distance_from_marker) < self.dist_treshold :
                self.velocity_publisher.publish(self.stop_movement)
                self.get_logger().info('marker reached')
                self.robot_state = RobotState.ACQUIRE_IMAGE

        elif self.robot_state == RobotState.ACQUIRE_IMAGE:
            if self.image_published:
                self.get_logger().info('image published')
                self.image_published = False  
                self.robot_state = RobotState.COMING_BACK              
            else:
                pass

        elif self.robot_state == RobotState.COMING_BACK:
            self.velocity_publisher.publish(self.go_home_velocity)
            if math.sqrt(self.robot_position.x**2 + self.robot_position.y**2) < self.dist_home_treshold:
                self.velocity_publisher.publish(self.stop_movement)
                self.get_logger().info('home reached')
                self.robot_state = RobotState.SET_MARKER_GOAL

    def image_callback(self, msg):
        detecting_first_marker = False
        cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')

        if self.robot_state != RobotState.SEARCH_FOR_MARKERS and self.robot_state != RobotState.ACQUIRE_IMAGE \
            and self.robot_state!= RobotState.ROTATE_TO_MARKER and self.robot_state != RobotState.STARTING:
            self.detecting_first_marker = False
            return
        
        img_gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
        corners, ids, _ = cv2.aruco.detectMarkers(img_gray, aruco_dict)
        if ids is None:
            self.detecting_first_marker = False
            return
    
        if len(self.markers_detected) == 0:
            self.first_marker_id = ids[0][0]
            self.get_logger().info("detected first marker")

        for i in range(len(ids)):
            id = ids[i][0]

            if id == self.first_marker_id:
                detecting_first_marker = True

            x_center = 0
            y_center = 0

            for corner in corners[i][0]:
                x_center += corner[0]
                y_center += corner[1]
            x_center /= 4.0
            y_center /= 4.0

            if self.robot_state == RobotState.ACQUIRE_IMAGE and not self.image_published:
                radius = 0
                for corner in corners[i][0]:
                    act_dist = math.dist(corner,[x_center,y_center])
                    if radius < act_dist:
                        radius = act_dist

                cv2.circle(cv_image, (int(x_center), int(y_center)), int(radius), (0, 0, 255), 3)
                cv2.imshow("view", cv_image)
                cv2.waitKey(15000) 
                cv2.destroyAllWindows()   
                out_msg = self.bridge.cv2_to_compressed_imgmsg(cv_image, dst_format='jpeg')
                out_msg.header = msg.header  
                self.image_publisher.publish(out_msg)
                self.image_published = True

            x_error = abs(640/2 - x_center)
            if self.robot_state == RobotState.ROTATE_TO_MARKER and id == self.marker_id_goal:
                self.goal_marker_error = x_error
            else:
                self.goal_marker_error = float("inf")

            if id not in self.markers_detected or \
                x_error < self.markers_detected[id]:
                self.markers_detected[id] = x_error
        self.detecting_first_marker = detecting_first_marker

    
    def detections_callback(self, msg):
        if self.robot_state == RobotState.GO_TO_MARKER or self.robot_state == RobotState.COMING_BACK:
            for marker in msg.markers:
                if marker.marker_id == self.marker_id_goal:
                    self.distance_from_marker = marker.pose.position.z
                    break

    def odometry_callback(self, msg):
        self.robot_position = msg.pose.pose.position
class RobotState(Enum):
    STARTING  = 1
    SEARCH_FOR_MARKERS = 2 
    SET_MARKER_GOAL = 3
    ROTATE_TO_MARKER = 4
    GO_TO_MARKER = 5
    ACQUIRE_IMAGE = 6
    COMING_BACK = 7

def main(args=None):
    rclpy.init(args=args)
    robot_controller = RobotControl()
    rclpy.spin(robot_controller)
    robot_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()