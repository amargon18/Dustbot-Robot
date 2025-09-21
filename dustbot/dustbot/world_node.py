import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from dustbot_interfaces.srv import SetDirection, LoadGarbage
import random

class WorldNode(Node):
    def __init__(self):
        super().__init__('world_node')
        self.robot_position = Point()
        self.robot_position.x = 0.0
        self.robot_position.y = 0.0

        # Parámetros
        self.declare_parameter('N', 5)
        self.declare_parameter('P', 3)

        self.N = self.get_parameter('N').get_parameter_value().integer_value
        self.P = self.get_parameter('P').get_parameter_value().integer_value

        # PUBLISHERS
        self.position_publisher = self.create_publisher(Point, '/dustbot/global_position', 10)
        self.garbage_position_publisher = self.create_publisher(Point, '/dustbot/garbage_position', 10)

        # SERVICIOS
        self.load_garbage_service = self.create_service(LoadGarbage, '/dustbot/load_garbage', self.load_garbage_callback)
        self.set_direction_service = self.create_service(SetDirection, '/dustbot/set_direction', self.set_direction_callback)

        # Generación de basura
        self.garbage_count = 0
        self.garbage_position = None
        self.is_garbage_collected = True  # Controla si se puede generar nueva basura

        # Temporizador para publicar la posición del robot
        self.timer = self.create_timer(1.0, self.timer_callback)

    def random_position(self):
        x = float(random.randint(0, self.N - 1))
        y = float(random.randint(0, self.N - 1))
        return Point(x=x, y=y, z=0.0)

    def create_garbage(self):
        if self.garbage_count < self.P and self.is_garbage_collected:
            self.garbage_position = self.random_position()
            self.garbage_position_publisher.publish(self.garbage_position)
            self.get_logger().info(f"New garbage created at postiion: ({self.garbage_position.x}, {self.garbage_position.y})")
            self.is_garbage_collected = False  # Bloquea nueva generación hasta que se recoja
        elif self.garbage_count == self.P:
            self.get_logger().info(f"All garbage positions generated and collected. Shutting down.")
            rclpy.shutdown()

        
    def load_garbage_callback(self, request, response):
        if self.garbage_position and self.robot_position:
            self.garbage_count += 1
            self.get_logger().info(f"Garbage number {self.garbage_count} collected")
            self.is_garbage_collected = True  # Permitir nueva generación de basura
            response.success = True
        else:
            response.success = False
            response.message = "Failed to collect garbage"
        return response

    def set_direction_callback(self, request, response):
        direction = request.direction
        self.update_robot_position(direction)
        self.position_publisher.publish(self.robot_position)

        if self.robot_position == self.garbage_position:
            response.success = True
        else:
            response.success = False

        return response

 
    def update_robot_position(self, direction):
        if direction == "Up" and self.robot_position.y < self.N - 1:
            self.robot_position.y += 1
        elif direction == "Down" and self.robot_position.y > 0:
            self.robot_position.y -= 1
        elif direction == "Right" and self.robot_position.x < self.N - 1:
            self.robot_position.x += 1
        elif direction == "Left" and self.robot_position.x > 0:
            self.robot_position.x -= 1
    

    def timer_callback(self):
        self.position_publisher.publish(self.robot_position)
        self.create_garbage()

def main(args=None):
    rclpy.init(args=args)
    node = WorldNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
