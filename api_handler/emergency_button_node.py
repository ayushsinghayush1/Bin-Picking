import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import Bool
# Import the custom service definition for toggling the emergency button state.
# This service is expected to be defined within the 'api_handler' package.
from api_handler.srv import ToggleEmergencyButtonState 


class EmergencyButtonNode(Node):
    """
    ROS 2 Node that simulates a physical emergency stop button.
    It publishes the current state of the button (active/inactive) and
    provides a service to toggle its state.
    """
    def __init__(self):
        super().__init__('emergency_button_node')
        self.get_logger().info('Emergency Button Node starting up...')

        # --- Initialize Node State ---
        # This variable holds the current state of the emergency button.
        # 'True' means the emergency button is active (pressed), 'False' means it's inactive (released).
        self.is_active = False # Initial state: Emergency button is NOT active (released)

        # --- QoS Profile ---
        # Define QoS (Quality of Service) profile for communication.
        # This profile ensures reliable message delivery and that the last
        # published message is available to new subscribers (TRANSIENT_LOCAL).
        # It's crucial for this to match the QoS used by subscribers in other nodes
        # (e.g., api_handler_node) to ensure proper data exchange.
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # --- Publishers ---
        # Create a publisher to send Bool messages to the '/emergency_button_state' topic.
        self.publisher_ = self.create_publisher(Bool, '/emergency_button_state', qos_profile)
        self.get_logger().info('Publisher for /emergency_button_state created.')

        # --- Service Server ---
        # Create a service server to listen for requests to 'toggle_emergency_button_state'.
        # This service will be called by other nodes (e.g., the API handler from the HMI).
        self.srv = self.create_service(ToggleEmergencyButtonState, 'toggle_emergency_button_state', self.toggle_emergency_button_callback)
        self.get_logger().info('ROS 2 service "toggle_emergency_button_state" created.')

        # --- Timer to periodically publish state (optional, but good for initial status) ---
        # This timer ensures that the current state is periodically published,
        # providing consistent updates even if the state doesn't change due to service calls.
        self.timer = self.create_timer(1.0, self.publish_emergency_state) # Publish every 1.0 second

        self.get_logger().info('Initial emergency button state: False (INACTIVE)') # Log initial state


    # --- Callback to publish current state ---
    def publish_emergency_state(self):
        """
        Publishes the current state of the emergency button to its topic.
        """
        msg = Bool()
        msg.data = self.is_active # True if active, False if inactive
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing emergency button state: {msg.data} ({"ACTIVE" if msg.data else "INACTIVE"})')


    # --- Service Server Callback ---
    def toggle_emergency_button_callback(self, request, response):
        """
        Callback function for the 'toggle_emergency_button_state' service.
        This function is executed when another node calls this service.
        It flips the current emergency button state (active/inactive).
        """
        # Toggle the internal 'is_active' state.
        self.is_active = not self.is_active

        # Immediately publish the new state after toggling.
        self.publish_emergency_state() 
        status_str = "ACTIVE" if self.is_active else "INACTIVE"
        self.get_logger().info(f'ROS 2 Service: Toggling emergency button state. Now: {status_str}.')

        # Populate the response fields as defined in the ToggleEmergencyButtonState.srv file.
        response.new_state = self.is_active # Return the new boolean state
        response.message = f"Emergency button toggled to {status_str}." # Return a descriptive message
        return response


def main(args=None):
    """
    Main function to initialize and run the Emergency Button Node.
    """
    rclpy.init(args=args) # Initialize the ROS 2 client library.
    node = EmergencyButtonNode() # Create an instance of the node.
    try:
        # Spin the node to process its callbacks (publisher timer, service requests).
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Handle graceful shutdown if Ctrl+C is pressed.
        node.get_logger().info('Node stopped cleanly.')
    finally:
        # Ensure the node is properly destroyed and ROS 2 client library is shut down.
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()