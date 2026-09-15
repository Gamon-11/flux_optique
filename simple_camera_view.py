import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

# Class SimpleCameraViewer qui herite de noeud ROS 2
class SimpleCameraViewer(Node):
    def __init__(self):
        super().__init__('camera_viewer')
        
        # Création d'une fenêtre OpenCV unique et fixe
        self.window_name = "Camera DownFacing"
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, 640, 480)

        # Abonnement au topic de la caméra (/camera)
        self.subscription = self.create_subscription(
            Image,
            '/camera',
            self.image_callback,
            10)
            
        self.bridge = CvBridge()
        self.get_logger().info("Lecteur caméra prêt. En attente d'images sur /camera...")

    def image_callback(self, msg):
        try:
            # Conversion de l'image ROS 2 vers OpenCV
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Affichage dans l'unique fenêtre réservée
            cv2.imshow(self.window_name, frame)
            
            # Quitter si la touche 'q' ou 'Echap' est pressée
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:
                self.get_logger().info("Fermeture demandée par l'utilisateur.")
                rclpy.shutdown()

        except Exception as e:
            self.get_logger().error(f"Erreur de lecture d'image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SimpleCameraViewer()
    try:
        rclpy.spin(node)
    except Exception:
        pass
    finally:
        cv2.destroyAllWindows()
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
