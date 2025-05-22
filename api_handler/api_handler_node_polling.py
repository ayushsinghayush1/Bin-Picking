import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import Bool, Int8
from api_handler.srv import ToggleEmergencyButtonState, ToggleDoorState 

from flask import Flask, render_template, jsonify, send_from_directory, request
import threading
import json
import os
import time

# --- Flask App Initialization ---
app = Flask(__name__, static_folder='hmi_static')

# --- Global variable to hold the ROS 2 Node instance ---
# This instance will be used by Flask routes to interact with ROS 2
ros_node_instance = None

# --- Define QoS Profile ---
# QoS (Quality of Service) profile for reliable communication over topics.
# This ensures messages are delivered and history is kept, important for state updates.
qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    durability=DurabilityPolicy.TRANSIENT_LOCAL
)

# --- ROS 2 Node Class ---
class ApiHandlerNode(Node):
    """
    ROS 2 Node that acts as an API handler.
    It subscribes to various ROS 2 topics for system status,
    provides service clients to send commands to other ROS 2 nodes,
    and runs a Flask web server for HMI interaction.
    """
    def __init__(self):
        super().__init__('api_handler_node')
        self.get_logger().info('API Handler Node starting up...')

        # --- Initialize last known states ---
        # These variables store the latest state received from ROS 2 topics,
        # which will be exposed via the Flask API.
        self.last_emergency_button_state = False
        self.last_door_state = "UNKNOWN"
        self.last_system_busy_state = False # This is a simulated state within this node
        self.last_stack_light_state = 0 # 0:Red, 1:Yellow, 2:Green

        # --- Subscribers ---
        # Subscribers listen to topics published by other ROS 2 nodes
        # to keep the HMI updated on system status.
        self.create_subscription(
            Bool,
            '/emergency_button_state',
            self.emergency_button_state_callback,
            qos_profile
        )
        self.create_subscription(
            Bool,
            '/door_state',
            self.door_state_callback,
            qos_profile
        )
        self.create_subscription(
            Int8,
            '/stack_light_status',
            self.stack_light_status_callback,
            qos_profile
        )
        self.get_logger().info('Subscribers for /emergency_button_state, /door_state, /stack_light_status created.')

        # --- Service Clients ---
        # Service clients are used to send commands to other ROS 2 nodes.
        # The 'while not wait_for_service' loop ensures the service is available
        # before attempting to call it.
        self.emergency_button_client = self.create_client(ToggleEmergencyButtonState, 'toggle_emergency_button_state')
        while not self.emergency_button_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Emergency button service not available, waiting...')

        self.door_handle_client = self.create_client(ToggleDoorState, 'toggle_door_state')
        while not self.door_handle_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Door handle service not available, waiting...')

        self.get_logger().info('Service clients for toggle_emergency_button_state, toggle_door_state created.')


    # --- Subscriber Callbacks ---
    # These methods are called automatically when a new message arrives on their respective topics.
    def emergency_button_state_callback(self, msg):
        """Callback for /emergency_button_state topic."""
        self.last_emergency_button_state = msg.data
        if msg.data:
            self.get_logger().warn(f'Received emergency button state update: ACTIVE (Denying requests!) (Bool: {msg.data})')
        else:
            self.get_logger().info(f'Received emergency button state update: INACTIVE (Allowing requests!) (Bool: {msg.data})')

    def door_state_callback(self, msg):
        """Callback for /door_state topic."""
        # Assuming msg.data=True means door is CLOSED, False means OPEN
        self.last_door_state = "CLOSED" if msg.data else "OPEN"
        self.get_logger().info(f'Received door state update: {self.last_door_state} (Bool: {msg.data})')

    def stack_light_status_callback(self, msg):
        """Callback for /stack_light_status topic."""
        # Assuming 0=RED, 1=YELLOW, 2=GREEN for stack light status
        self.last_stack_light_state = msg.data
        status_map = {0: "RED", 1: "YELLOW", 2: "GREEN"}
        self.get_logger().info(f'Received stack light state update: {self.last_stack_light_state} ({status_map.get(self.last_stack_light_state, "UNKNOWN")})')

    # --- Service Call Methods (for Flask to call) ---
    # These methods are called by Flask routes to trigger ROS 2 service calls.
    def call_toggle_emergency_button_service(self):
        """Calls the 'toggle_emergency_button_state' ROS 2 service."""
        request = ToggleEmergencyButtonState.Request()
        future = self.emergency_button_client.call_async(request)
        # Spin until the service call completes to get the result
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def call_toggle_door_service(self):
        """Calls the 'toggle_door_state' ROS 2 service."""
        request = ToggleDoorState.Request()
        future = self.door_handle_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def toggle_system_busy_state(self):
        """Simulates toggling the system busy state internally for the HMI."""
        self.last_system_busy_state = not self.last_system_busy_state
        self.get_logger().info(f"Simulating system busy state: {'BUSY' if self.last_system_busy_state else 'READY'}")


# --- Flask Routes ---
# These define the HTTP endpoints for the web HMI to interact with.

@app.route('/hmi')
def hmi():
    """Serves the main HMI HTML page."""
    return send_from_directory(app.static_folder, 'index.html')

@app.route('/get_status')
def get_status():
    """
    API endpoint for the HMI to poll system status.
    Returns the latest known states from ROS 2 subscriptions as JSON.
    """
    global ros_node_instance
    if ros_node_instance:
        status = {
            'emergency_button_active': ros_node_instance.last_emergency_button_state,
            'door_state': ros_node_instance.last_door_state,
            'system_busy': ros_node_instance.last_system_busy_state,
            'stack_light_status': ros_node_instance.last_stack_light_state
        }
        return jsonify(status)
    # Return an error if the ROS 2 node hasn't been initialized yet
    return jsonify({"error": "ROS 2 node not initialized"}), 500


# --- HMI Button Command Routes ---
# These routes handle POST requests from HMI buttons to trigger actions.

@app.route('/toggle_emergency', methods=['POST'])
def toggle_emergency():
    """Handles the 'Toggle Emergency Button' command from the HMI."""
    global ros_node_instance
    if ros_node_instance:
        try:
            response = ros_node_instance.call_toggle_emergency_button_service()
            return jsonify({
                "success": True,
                "message": f"Emergency button toggled. New state: {'ACTIVE' if ros_node_instance.last_emergency_button_state else 'INACTIVE'}",
                "new_state": ros_node_instance.last_emergency_button_state
            })
        except Exception as e:
            ros_node_instance.get_logger().error(f"Error calling emergency button service: {e}")
            return jsonify({"success": False, "message": f"Failed to toggle emergency button: {e}"}), 500
    return jsonify({"success": False, "message": "ROS 2 node not initialized"}), 500

@app.route('/toggle_door', methods=['POST'])
def toggle_door():
    """Handles the 'Toggle Door' command from the HMI."""
    global ros_node_instance
    if ros_node_instance:
        try:
            response = ros_node_instance.call_toggle_door_service()
            return jsonify({
                "success": True,
                "message": f"Door toggled. New state: {ros_node_instance.last_door_state}",
                "new_state": ros_node_instance.last_door_state
            })
        except Exception as e:
            ros_node_instance.get_logger().error(f"Error calling door service: {e}")
            return jsonify({"success": False, "message": f"Failed to toggle door: {e}"}), 500
    return jsonify({"success": False, "message": "ROS 2 node not initialized"}), 500

@app.route('/toggle_busy', methods=['POST'])
def toggle_busy():
    """Handles the 'Toggle System Busy' command from the HMI (simulated internally)."""
    global ros_node_instance
    if ros_node_instance:
        ros_node_instance.toggle_system_busy_state()
        status_str = "BUSY" if ros_node_instance.last_system_busy_state else "READY"
        return jsonify({"success": True, "message": f"System busy state toggled to {status_str}.", "new_state": ros_node_instance.last_system_busy_state})
    return jsonify({"success": False, "message": "ROS 2 node not initialized"}), 500


# --- Flask Thread ---
def run_flask():
    """Starts the Flask web server in a separate thread."""
    app.run(host='127.0.0.1', port=5000)

# --- Main function for ROS 2 node ---
def main(args=None):
    """
    Main function to initialize the ROS 2 node and start the Flask app.
    It runs the Flask app in a separate thread to not block the ROS 2 event loop.
    """
    global ros_node_instance
    rclpy.init(args=args)
    ros_node_instance = ApiHandlerNode()

    # Create and start Flask app in a separate thread
    flask_thread = threading.Thread(target=run_flask)
    flask_thread.daemon = True # Daemon threads exit when the main program exits
    flask_thread.start()
    ros_node_instance.get_logger().info("Flask app started in a separate thread.")

    try:
        # Spin the ROS 2 node to process callbacks (subscriptions, service calls)
        rclpy.spin(ros_node_instance)
    except KeyboardInterrupt:
        ros_node_instance.get_logger().info('API Handler Node stopped cleanly.')
    finally:
        # Cleanly shut down the ROS 2 node and client library
        ros_node_instance.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()