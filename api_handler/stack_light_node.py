import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from std_msgs.msg import Int8
# Import the custom service definition for setting the stack light state.
# This service is expected to be defined within the 'api_handler' package.
from api_handler.srv import SetStackLightState 

class StackLightNode(Node):
    """
    ROS 2 Node that simulates an industrial stack light.
    It provides a service to directly set its color (RED, YELLOW, GREEN)
    and publishes its current state to a topic.
    """
    # Define integer constants for stack light states to improve readability.
    # These values correspond to the 'Int8' message type.
    STATE_RED = 0     # Corresponds to value 0 in Int8
    STATE_YELLOW = 1  # Corresponds to value 1 in Int8
    STATE_GREEN = 2   # Corresponds to value 2 in Int8

    def __init__(self):
        super().__init__('stack_light_node')
        self.get_logger().info('Stack Light Node starting up...')

        # --- Initialize Stack Light State ---
        # The internal variable holding the current active color of the stack light.
        self.current_light_state = self.STATE_RED # Default to RED on node startup

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

        # --- Publisher for stack light status ---
        # This publisher sends the 'current_light_state' (an Int8 value)
        # to the '/stack_light_status' topic. Other nodes can subscribe to this
        # topic to get the stack light's visual status.
        self.stack_light_publisher = self.create_publisher(Int8, '/stack_light_status', qos_profile)
        self.get_logger().info('Publisher for /stack_light_status created.')

        # --- Service Server for setting stack light state ---
        # This service allows other ROS 2 nodes (e.g., the API handler from the HMI)
        # to directly command the stack light's color by calling 'set_stack_light_state'.
        self.srv = self.create_service(SetStackLightState, 'set_stack_light_state', self.set_stack_light_callback)
        self.get_logger().info('ROS 2 service "set_stack_light_state" created.')

        # --- Timer to periodically publish the current state ---
        # This timer ensures that the current state is always advertised on the topic
        # at regular intervals (every 0.5 seconds), providing continuous updates
        # and ensuring new subscribers receive a recent message.
        self.publisher_timer = self.create_timer(0.5, self.publish_stack_light_state)
        self.get_logger().info('Stack light status publisher timer started.')

        self.get_logger().info(f'Publishing initial stack light state: {self.current_light_state} (RED)')


    # --- Service Server Callback ---
    def set_stack_light_callback(self, request, response):
        """
        Callback function for the 'set_stack_light_state' service.
        This function is executed when another node calls this service.
        It updates the stack light's internal state to the requested color.
        """
        # Validate the requested state to ensure it's one of the defined colors.
        if request.state in [self.STATE_RED, self.STATE_YELLOW, self.STATE_GREEN]:
            self.current_light_state = request.state # Update the internal state
            # Immediately publish the new state after a successful service call
            self.publish_stack_light_state()

            # Prepare a human-readable message for the response
            status_map = {
                self.STATE_RED: "RED",
                self.STATE_YELLOW: "YELLOW",
                self.STATE_GREEN: "GREEN"
            }
            response.success = True
            response.message = f"Stack light set to {status_map[request.state]}."
            self.get_logger().info(f'ROS 2 Service: Stack light set to {status_map[request.state]}.')
        else:
            # Handle invalid state requests
            response.success = False
            response.message = f"Invalid stack light state requested: {request.state}. Must be 0 (RED), 1 (YELLOW), or 2 (GREEN)."
            self.get_logger().warn(response.message)
        return response

    # --- Publisher Callback ---
    def publish_stack_light_state(self):
        """
        Publishes the current stack light state (held in `self.current_light_state`)
        to the '/stack_light_status' topic.
        """
        msg = Int8() # Create an Int8 message
        msg.data = self.current_light_state # Set the data to the current light state
        self.stack_light_publisher.publish(msg) # Publish the message

        # Prepare a human-readable string for logging purposes
        status_map = {
            self.STATE_RED: "RED",
            self.STATE_YELLOW: "YELLOW",
            self.STATE_GREEN: "GREEN"
        }
        # Log the published state. This helps in debugging and monitoring.
        self.get_logger().info(f'Stack Light: {status_map.get(self.current_light_state, "UNKNOWN")} (Value: {msg.data})')


def main(args=None):
    """
    Main function to initialize and run the Stack Light Node.
    """
    rclpy.init(args=args) # Initialize the ROS 2 client library.
    node = StackLightNode() # Create an instance of the node.
    try:
        # Spin the node to process its callbacks (publisher timer, service requests).
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Handle graceful shutdown if Ctrl+C is pressed.
        node.get_logger().info('Stack Light Node stopped cleanly.')
    finally:
        # Ensure the node is properly destroyed and ROS 2 client library is shut down.
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()