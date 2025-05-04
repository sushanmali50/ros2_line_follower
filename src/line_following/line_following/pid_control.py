import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class LineFollowingMotor(Node):
    def __init__(self):
        super().__init__('line_following_motor')
        self.bridge = CvBridge()

        # Subscribers and publishers
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.image_pub = self.create_publisher(Image, '/processed_image', 10)  # For visualization

        # Motion parameters
        self.forward_speed = 0.5  # Speed when aligned
        self.turn_speed = 0.35  # Fixed turn speed for correction
        self.error_threshold = 60  # Only turn when error exceeds this

    def image_callback(self, msg):
        # Convert ROS2 image to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        height, width, _ = cv_image.shape

        # Crop to the lower half of the image
        lower_half = cv_image[height // 2 :, :]

        # Convert to HSV and create a mask for black color
        hsv = cv2.cvtColor(lower_half, cv2.COLOR_BGR2HSV)
        lower_black = (0, 0, 0)
        upper_black = (180, 255, 50)
        mask = cv2.inRange(hsv, lower_black, upper_black)

        # Find contours in the lower half
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        twist = Twist()
        twist.linear.x = 0.0  # Default to stopping

        if contours:
            # Find the largest contour (assuming it's the track)
            largest_contour = max(contours, key=cv2.contourArea)
            cv2.drawContours(lower_half, [largest_contour], -1, (0, 255, 0), 2)

            # Get bounding box and center
            x, y, w, h = cv2.boundingRect(largest_contour)
            center_x = x + w // 2
            center_y = y + h // 2  # Just for visualization

            # Adjust center_y to match the full image coordinates
            adjusted_center_y = center_y + (height // 2)

            # Draw center point on the full image
            cv2.circle(cv_image, (center_x, adjusted_center_y), 5, (255, 0, 0), -1)

            # Compute error (how far from center)
            error = center_x - (width // 2)

            if abs(error) < self.error_threshold:
                # If the error is small, move forward
                twist.linear.x = self.forward_speed
                twist.angular.z = 0.0
            else:
                # If the error is large, stop forward motion and turn
                twist.linear.x = 0.1
                twist.angular.z = -np.sign(error) * self.turn_speed  # Normalize for Twist message

            # Debugging output
            self.get_logger().info(f"Error: {error}, Angular: {twist.angular.z:.2f}")

        # Publish velocity command
        self.cmd_vel_pub.publish(twist)

        # Publish processed image with contours for RQT
        processed_img_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
        self.image_pub.publish(processed_img_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LineFollowingMotor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
