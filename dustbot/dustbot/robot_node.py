import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from dustbot_interfaces.srv import SetDirection, LoadGarbage
import time

class RobotNode(Node):
    def __init__(self):
        super().__init__('robot_node')
        self.garbage_subscriber = self.create_subscription(Point, '/dustbot/garbage_position', self.garbage_callback, 10)
        self.set_direction_client = self.create_client(SetDirection, '/dustbot/set_direction')
        self.load_garbage_client = self.create_client(LoadGarbage, '/dustbot/load_garbage')

        self.declare_parameter('N', 5) 
        self.declare_parameter('P', 3) 

        self.N = self.get_parameter('N').get_parameter_value().integer_value
        self.P = self.get_parameter('P').get_parameter_value().integer_value

        self.i = 0
        self.garbage_position = None
        self.robot_position = Point()
        self.robot_position.x = 0.0
        self.robot_position.y = 0.0
        self.get_logger().info(f"Robot initial position: ({self.robot_position.x}, {self.robot_position.y})")
        self.is_busy = False  # Flag to track if robot is moving

        # Publicador de la posición del robot
        self.position_publisher = self.create_publisher(Point, '/dustbot/global_position', 10)

    def garbage_callback(self, msg):
        self.garbage_position = msg
        self.is_busy = True  # Mark robot as busy

        self.move_to_garbage()

    def move_to_garbage(self): 
        while not self.has_reached_garbage():
            # Llamar al servicio de dirección
            self.call_set_direction_service()
            # Publicar la nueva posición
            self.position_publisher.publish(self.robot_position)
            # Una vez que ha llegado, intentar recoger la basura
        self.get_logger().info(f"Reached garbage at: ({self.robot_position.x}, {self.robot_position.y}). Attempting to collect...")
        self.call_load_garbage_service()
        self.is_busy = False

    def calculate_direction(self, robot_pos, garbage_pos):  
        if robot_pos.x < garbage_pos.x:
            return "Right"
        elif robot_pos.x > garbage_pos.x:
            return "Left"
        elif robot_pos.y < garbage_pos.y:
            return "Up"
        elif robot_pos.y > garbage_pos.y:
            return "Down"
        
        
    def call_set_direction_service(self):
        # Llamar al servicio de dirección de forma asincrónica
        while not self.set_direction_client.wait_for_service(timeout_sec=1.0):
           self.get_logger().info('Service /dustbot/set_direction not available, waiting...')

        set_dir_req = SetDirection.Request()
        set_dir_req.direction = self.calculate_direction(self.robot_position, self.garbage_position)

        self.get_logger().info(f'Moving {set_dir_req.direction}')
        self.update_position(set_dir_req.direction)
        self.get_logger().info(f'New robot position: ({self.robot_position.x}, {self.robot_position.y})')



        # Definir el callback para manejar la respuesta del servicio
        def _set_direction_done_clbk(future):
            # Aquí el future está hecho y la respuesta está disponible
            response = future.result()

            if response.success:
                self.get_logger().info(f"Direction set to: {set_dir_req.direction}")
            
        # Llamar al servicio de forma asincrónica y agregar el callback
        future = self.set_direction_client.call_async(set_dir_req)
        future.add_done_callback(_set_direction_done_clbk)


    def update_position(self, direction):
        # Actualizar la posición del robot después de moverse
        if direction == "Up" and self.robot_position.y < self.N - 1:
            self.robot_position.y += 1
            time.sleep(1)
        elif direction == "Down" and self.robot_position.y > 0:
            self.robot_position.y -= 1
            time.sleep(1)
        elif direction == "Right" and self.robot_position.x < self.N - 1:
            self.robot_position.x += 1
            time.sleep(1)
        elif direction == "Left" and self.robot_position.x > 0:
            self.robot_position.x -= 1
            time.sleep(1)


    def has_reached_garbage(self):
        # Comparar las posiciones exactas
        return self.robot_position.x == self.garbage_position.x and self.robot_position.y == self.garbage_position.y

    def call_load_garbage_service(self):
        # Llamar al servicio para cargar la basura
        if not self.load_garbage_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /dustbot/load_garbage not available, waiting...')
            return

        request = LoadGarbage.Request()  # Crear un Request vacío
        
        def _set_load_garbage_done_clbk(future):
            # Aquí el future está hecho y la respuesta está disponible
            response = future.result()
            if response.success:
                self.get_logger().info(f"Garbage collected succesfully")
            else:
                self.get_logger().error(f"Garbage not collected")

        future = self.load_garbage_client.call_async(request)
        future.add_done_callback(_set_load_garbage_done_clbk)

def main(args=None):
    rclpy.init(args=args)
    node = RobotNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


