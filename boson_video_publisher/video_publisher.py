import rclpy  # ROS 2 Python client library
from rclpy.node import Node  # Base class for creating ROS 2 nodes
import cv2  # OpenCV library for image processing
from cv_bridge import CvBridge  # Library to convert between ROS and OpenCV images
from sensor_msgs.msg import Image  # ROS 2 message type for images

# Define a class for the video publisher node
class VideoPublisher(Node):
    def __init__(self):
        # Initialize the node with the name 'video_publisher'
        super().__init__('video_publisher')
        
        # Create a publisher for the 'boson_video' topic with a queue size of 10
        # This will publish Image messages
        self.publisher_ = self.create_publisher(Image, 'boson_video', 10)
        
        # Initialize CvBridge to convert between OpenCV images and ROS Image messages
        self.bridge = CvBridge()
        
        # Create a timer to call the timer_callback function at 20 Hz (0.05 seconds interval)
        self.timer = self.create_timer(0.05, self.timer_callback)
        
        # Open the video capture device (FLIR Boson camera in this case) at '/dev/video2'
        # Adjust the device path if necessary
        self.cap = cv2.VideoCapture('/dev/video2')

        # Check if the video capture device was successfully opened
        if not self.cap.isOpened():
            # Log an error message if the video device cannot be opened
            self.get_logger().error('Unable to open video device')
            
            # Shut down the ROS node if the video device is not accessible
            rclpy.shutdown()

    def timer_callback(self):
        # Capture a frame from the video device
        ret, frame = self.cap.read()
        
        # If a frame was successfully captured
        if ret:
            # Determine the image encoding type based on the number of color channels
            # 'bgr8' for color images (3 channels), 'mono8' for grayscale images (1 channel)
            encoding = "bgr8" if len(frame.shape) == 3 else "mono8"
            
            # Convert the OpenCV image to a ROS Image message
            msg = self.bridge.cv2_to_imgmsg(frame, encoding=encoding)
            
            # Publish the Image message on the 'boson_video' topic
            self.publisher_.publish(msg)
            
            # Log an info message indicating that a frame has been published
            self.get_logger().info('Publishing frame')
        else:
            # Log an error message if the frame capture failed
            self.get_logger().error('Failed to capture frame')

    def destroy_node(self):
        # Release the video capture device before shutting down the node
        super().destroy_node()
        self.cap.release()

# Main function to initialize the node and start the ROS spinning loop
def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)
    
    # Create an instance of the VideoPublisher node
    video_publisher = VideoPublisher()
    
    # Keep the node running and processing events
    rclpy.spin(video_publisher)
    
    # When the node is stopped, release resources and shut down rclpy
    video_publisher.destroy_node()
    rclpy.shutdown()

# Entry point of the script
if __name__ == '__main__':
    main()
