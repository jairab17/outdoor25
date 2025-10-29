#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import pygame
import threading
import time


class PygameMoveDrone(Node):
    def __init__(self):
        super().__init__('keyboard_move_drone')
        self.get_logger().info('Keyboard control node (pygame) started')

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/px4_driver/cmd_vel', 10)
        self.height_pub = self.create_publisher(Bool, '/px4_driver/do_height_control', 10)

        # Parameters
        self.declare_parameter('scale_linear', 7.0)
        self.declare_parameter('scale_angular', 2.0)
        self.declare_parameter('do_height_control', False)

        self.do_height_control = self.get_parameter('do_height_control').value
        self.running = True
        self.twist = Twist()

        # Start pygame in another thread (non-blocking)
        self.key_thread = threading.Thread(target=self.keyboard_loop, daemon=True)
        self.key_thread.start()

        # Publish rate
        self.timer = self.create_timer(0.1, self.publish_cmd)

    def keyboard_loop(self):
        pygame.init()
        pygame.display.set_mode((200, 100))
        pygame.display.set_caption("Drone Keyboard Control (Press ESC to quit)")

        linear_speed = self.get_parameter('scale_linear').value
        angular_speed = self.get_parameter('scale_angular').value

        self.get_logger().info(
            "Controls:\n"
            "W/S: forward/back\n"
            "A/D: left/right\n"
            "↑/↓: ascend/descend\n"
            "Q/E: yaw left/right\n"
            "ESC: quit"
        )

        while self.running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False
                elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                    self.running = False

            # Reset twist
            twist = Twist()
            keys = pygame.key.get_pressed()

            # --- Linear X (forward/back) ---
            if keys[pygame.K_w]:
                twist.linear.x = linear_speed
            elif keys[pygame.K_s]:
                twist.linear.x = -linear_speed

            # --- Linear Y (left/right) ---
            if keys[pygame.K_a]:
                twist.linear.y = linear_speed
            elif keys[pygame.K_d]:
                twist.linear.y = -linear_speed

            # --- Linear Z (up/down) ---
            if keys[pygame.K_UP]:
                twist.linear.z = linear_speed
            elif keys[pygame.K_DOWN]:
                twist.linear.z = -linear_speed

            # --- Angular Z (yaw) ---
            if keys[pygame.K_q]:
                twist.angular.z = -angular_speed
            elif keys[pygame.K_e]:
                twist.angular.z = angular_speed

            self.twist = twist
            time.sleep(0.05)

        pygame.quit()
        self.get_logger().info("Keyboard control stopped")

    def publish_cmd(self):
        if not self.running:
            rclpy.shutdown()
            return

        # Publish height control state
        msg = Bool()
        msg.data = self.do_height_control
        self.height_pub.publish(msg)

        # Publish twist command
        self.cmd_pub.publish(self.twist)


def main(args=None):
    rclpy.init(args=args)
    node = PygameMoveDrone()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


"""
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class MoveDroneNode(Node):
    def __init__(self):
        super().__init__('move_drone')
        self.get_logger().info('move_drone node started')

        # Publicador a /px4_driver/cmd_vel
        self.cmd_pub = self.create_publisher(Twist, '/px4_driver/cmd_vel', 10)
        self.do_height_control_pub = self.create_publisher(Bool, "/px4_driver/do_height_control", 10)

        # Suscripción a /joy
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        self.declare_parameter("do_height_control", False)
        self.do_height_control = self.get_parameter("do_height_control").get_parameter_value().bool_value

        # Parámetros de escalado
        self.declare_parameter('scale_linear', 3.0)
        self.declare_parameter('scale_angular', 3.0)
        self.declare_parameter('enable_button', 4)  # L1 por defecto

    def joy_callback(self, msg: Joy):
        enable_button = self.get_parameter('enable_button').value
        twist = Twist()
        do_height_control_msg = Bool()
        do_height_control_msg.data = self.do_height_control
        self.do_height_control_pub.publish(do_height_control_msg)

        # Solo se mueve si se mantiene presionado el botón de habilitación
        if enable_button < len(msg.buttons) and msg.buttons[enable_button] == 1:
            roll  = msg.axes[3]  # roll
            pitch  = msg.axes[2]  # pitch
            yaw = msg.axes[0]  # yaw
            z = msg.axes[1]  # z

            
            twist.linear.x = roll * self.get_parameter('scale_linear').value
            twist.linear.y = pitch * self.get_parameter('scale_linear').value
            twist.linear.z = z * self.get_parameter('scale_linear').value
            twist.angular.z = -yaw * self.get_parameter('scale_angular').value

            self.cmd_pub.publish(twist)

        
        else:
            # Si no está presionado el botón, publica Twist en 0 para que el dron se mantenga quieto
            twist = Twist()
            self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = MoveDroneNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
"""