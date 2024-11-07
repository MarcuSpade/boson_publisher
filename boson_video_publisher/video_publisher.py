import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class VideoPublisher(Node):
    def __init__(self):
        super().__init__('video_publisher')
        self.publisher_ = self.create_publisher(Image, 'boson_video', 10)
        self.bridge = CvBridge()
        self.timer = self.create_timer(0.05, self.timer_callback)  # Publica a cada 0.1 segundos (10 Hz)
        self.cap = cv2.VideoCapture('/dev/video2')  # Altere o dispositivo, se necessário

        if not self.cap.isOpened():
            self.get_logger().error('Não foi possível abrir o dispositivo de vídeo')
            rclpy.shutdown()

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            # Verifica o número de canais da imagem
            encoding = "bgr8" if len(frame.shape) == 3 else "mono8"
            
            # Converte a imagem OpenCV para uma mensagem ROS e publica
            msg = self.bridge.cv2_to_imgmsg(frame, encoding=encoding)
            self.publisher_.publish(msg)
            self.get_logger().info('Publicando frame')
        else:
            self.get_logger().error('Não foi possível capturar o frame')

    def destroy_node(self):
        super().destroy_node()
        self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    video_publisher = VideoPublisher()
    rclpy.spin(video_publisher)
    video_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


