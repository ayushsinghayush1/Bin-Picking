# door_handle_node.py
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from std_msgs.msg import Bool
# Import the custom service definition for toggling the door state.
# This service is defined in the 'api_handler' package's 'srv' directory.
from api_handler.srv import ToggleDoorState 

class DoorHandleNode(Node):
    """
    ROS 2 Node that simulates a physical door handle sensor and actuator.
    It publishes the current state of the door (open/closed) and
    provides a service to toggle its state.
    """
    def __init__(self):
        super().__init__('door_handle_node')
        self.get_logger().info('Door Handle Node starting up...')

        # Define QoS (Quality of Service) profile for communication.
        # This profile ensures reliable message delivery and that the last
        # published message is available to new subscribers (TRANSIENT_LOCAL).
        # It's crucial for this to match the QoS used by subscribers in other nodes
        # (e.g., api_handler_node) to ensure proper data exchange.
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL # IMPORTANT: Match this with api_handler_node's subscriber
        )

        # Internal state variable for the door.
        # True signifies the door is CLOSED, False signifies it is OPEN.
        self.current_door_state = True # Initialize door as closed

        # Publisher for the door state.
        # It publishes Bool messages to the '/door_state' topic.
        self.door_state_publisher = self.create_publisher(Bool, '/door_state', qos_profile)
        self.get_logger().info('Publisher for /door_state created.')

        # Timer to periodically publish the current door state.
        # This ensures consistent updates even if the state doesn't change from service calls.
        self.publisher_timer = self.create_timer(1.0, self.publish_door_state) # Publish every 1.0 second
        self.get_logger().info('Door state publisher timer started.')

        # Service server to allow other nodes (e.g., HMI through api_handler)
        # to request a toggle of the door's state.
        self.toggle_door_service = self.create_service(
            ToggleDoorState, 'toggle_door_state', self.toggle_door_state_callback
        )
        self.get_logger().info('Service server for /toggle_door_state created.')

    def publish_door_state(self):
        """
        Publishes the current state of the door to the '/door_state' topic.
        """
        msg = Bool()
        msg.data = self.current_door_state # True for CLOSED, False for OPEN
        self.door_state_publisher.publish(msg)
        status_str = "CLOSED" if self.current_door_state else "OPEN"
        self.get_logger().info(f'Publishing door state: {status_str} (Bool: {msg.data})')

    def toggle_door_state_callback(self, request, response):
        """
        Callback function for the 'toggle_door_state' service.
        When this service is called, it flips the current door state.
        """
        self.current_door_state = not self.current_door_state # Toggle the boolean state
        status_str = "CLOSED" if self.current_door_state else "OPEN"
        self.get_logger().info(f'Received toggle_door_state request. Door is now {status_str}.')
        # The service response can include a message indicating the new state.
        response.message = f'Door state toggled to {status_str}.'
        return response

def main(args=None):
    """
    Main function to initialize and run the Door Handle Node.
    """
    rclpy.init(args=args) # Initialize the ROS 2 client library
    node = DoorHandleNode() # Create an instance of the node
    try:
        # Spin the node to process callbacks (timer, service requests)
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Door Handle Node stopped cleanly.')
    finally:
        # Ensure the node is destroyed and ROS 2 is shut down cleanly on exit
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()