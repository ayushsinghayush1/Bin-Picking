import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import Bool, Int8

# Import all necessary custom service types.
# Ensure these .srv files exist in your api_handler/srv/ directory.
from api_handler.srv import ToggleEmergencyButtonState, ToggleDoorState, SetStackLightState, ScanBarcode, GetLastBarcode

from flask import Flask, render_template, jsonify, send_from_directory, request
# Import SocketIO for real-time communication between Flask and HMI
from flask_socketio import SocketIO, emit
import threading
import json
import os
import time
from datetime import datetime # Import datetime for timestamps

# --- Flask App Initialization ---
# Initialize Flask app, specifying the folder for static HMI files.
app = Flask(__name__, static_folder='hmi_static')
# Initialize Flask-SocketIO for real-time communication.
# Using 'eventlet' for async_mode is recommended for concurrent operations, especially with ROS 2.
socketio = SocketIO(app, async_mode='eventlet', cors_allowed_origins="*")

# --- Global variable to hold the ROS 2 Node instance ---
# This allows Flask routes to access the ROS 2 node's methods and data.
ros_node_instance = None

# --- Define QoS Profile ---
# Standard QoS profile for reliable communication and keeping the last message.
qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    durability=DurabilityPolicy.TRANSIENT_LOCAL # Important for state updates
)

# --- ROS 2 Node Class ---
class ApiHandlerNode(Node):
    """
    This node acts as the central API handler, bridging ROS 2 communications
    (subscriptions to sensor states, client calls to control services,
    and server for barcode scanning) with a web-based Flask HMI.
    """
    def __init__(self):
        super().__init__('api_handler_node')
        self.get_logger().info('API Handler Node starting up...')

        # --- Initialize last known states ---
        # These variables store the latest information received from other nodes
        # or managed internally, to be sent to the HMI.
        self.last_emergency_button_state = False # True if active, False if inactive
        self.last_door_state = "UNKNOWN"         # "OPEN" or "CLOSED"
        self.last_system_busy_state = False      # True if busy, False if idle
        self.last_stack_light_state = 0          # 0:Red, 1:Yellow, 2:Green

        # --- Barcode and Request/Response Tracking ---
        self.last_scanned_barcode_data = "N/A"
        self.last_scanned_barcode_timestamp = "N/A"
        self.last_response_success = False
        self.last_response_message = "No barcode scanned yet."

        # --- Publishers ---
        # Publisher for the system busy status.
        self.system_busy_publisher = self.create_publisher(Bool, '/system_busy_status', qos_profile)
        self.get_logger().info('Publisher for /system_busy_status created.')

        # --- Subscribers ---
        # Subscribes to various sensor/status topics from other ROS 2 nodes.
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
        # Clients to call services provided by other ROS 2 nodes (e.g., to toggle their states).
        self.emergency_button_client = self.create_client(ToggleEmergencyButtonState, 'toggle_emergency_button_state')
        while not self.emergency_button_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Emergency button service not available, waiting...')

        self.door_handle_client = self.create_client(ToggleDoorState, 'toggle_door_state')
        while not self.door_handle_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Door handle service not available, waiting...')

        self.stack_light_client = self.create_client(SetStackLightState, 'set_stack_light_state')
        while not self.stack_light_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Stack light service not available, waiting...')

        self.get_logger().info('Service clients for toggle_emergency_button_state, toggle_door_state, set_stack_light_state created.')

        # --- Service Servers ---
        # These services are provided by the ApiHandlerNode itself,
        # typically for other ROS 2 nodes (or internal use) to request actions.
        # This is where your barcode services are defined.
        self.scan_barcode_service = self.create_service(ScanBarcode, 'scan_barcode_service', self.scan_barcode_callback)
        self.get_last_barcode_service = self.create_service(GetLastBarcode, 'get_last_barcode_service', self.get_last_barcode_callback)
        self.get_logger().info('ROS 2 service servers for "scan_barcode_service" and "get_last_barcode_service" created.')


    # --- Subscriber Callbacks ---
    def emergency_button_state_callback(self, msg):
        """Callback for /emergency_button_state topic."""
        self.last_emergency_button_state = msg.data
        if msg.data:
            self.get_logger().warn(f'Received emergency button state update: ACTIVE (Denying requests!) (Bool: {msg.data})')
        else:
            self.get_logger().info(f'Received emergency button state update: INACTIVE (Allowing requests!) (Bool: {msg.data})')
        # Emit update to HMI
        socketio.emit('emergency_state_update', {'active': self.last_emergency_button_state})

    def door_state_callback(self, msg):
        """Callback for /door_state topic."""
        self.last_door_state = "CLOSED" if msg.data else "OPEN"
        self.get_logger().info(f'Received door state update: {self.last_door_state} (Bool: {msg.data})')
        # Emit update to HMI
        socketio.emit('door_state_update', {'closed': msg.data})

    def stack_light_status_callback(self, msg):
        """Callback for /stack_light_status topic."""
        self.last_stack_light_state = msg.data
        status_map = {0: "RED", 1: "YELLOW", 2: "GREEN"}
        self.get_logger().info(f'Received stack light state update: {self.last_stack_light_state} ({status_map.get(self.last_stack_light_state, "UNKNOWN")})')
        # Emit update to HMI
        socketio.emit('stack_light_update', {'value': self.last_stack_light_state})

    # --- Service Server Callbacks (provided by this node) ---
    def scan_barcode_callback(self, request, response):
        """
        Callback for the 'scan_barcode_service'.
        Simulates a barcode scan and applies safety interlocks.
        """
        self.get_logger().info(f'Received scan_barcode request for: {request.barcode_data} at {request.timestamp}')
        self.set_system_busy(True) # Set system to busy during scan

        # Safety Interlocks
        if self.last_emergency_button_state:
            response.success = False
            response.message = "Pick request denied: EMERGENCY BUTTON ACTIVE. Please deactivate to proceed."
            self.get_logger().error(f'Pick request for "{request.barcode_data}" DENIED: EMERGENCY BUTTON ACTIVE.')
        elif self.last_door_state == "OPEN":
            response.success = False
            response.message = "Pick request denied: Door is OPEN. Please close the door to proceed."
            self.get_logger().error(f'Pick request for "{request.barcode_data}" denied due to open door.')
        else:
            # Simulate processing time
            self.get_logger().info(f'Processing pick request for barcode: {request.barcode_data}...')
            time.sleep(1.0) # Simulate work being done

            # Successful scan
            response.success = True
            response.message = f"Barcode scanned and pick request processed successfully for: {request.barcode_data}"
            self.last_scanned_barcode_data = request.barcode_data
            self.last_scanned_barcode_timestamp = request.timestamp
            self.get_logger().info(f'Barcode scan successful: {request.barcode_data}')

        # Store the response for HMI update
        self.last_response_success = response.success
        self.last_response_message = response.message

        # Emit update to HMI
        self.emit_request_response_update()
        self.set_system_busy(False) # Set system back to idle
        return response

    def get_last_barcode_callback(self, request, response):
        """
        Callback for the 'get_last_barcode_service'.
        Returns the data of the last successfully scanned barcode.
        """
        response.barcode_data = self.last_scanned_barcode_data
        response.timestamp = self.last_scanned_barcode_timestamp
        response.success = self.last_response_success
        response.message = self.last_response_message
        self.get_logger().info(f'Returning last barcode data: {self.last_scanned_barcode_data}')
        return response

    # --- Internal Methods ---
    def set_system_busy(self, busy_state):
        """Sets the system busy state and publishes it."""
        self.last_system_busy_state = busy_state
        msg = Bool()
        msg.data = self.last_system_busy_state
        self.system_busy_publisher.publish(msg)
        self.get_logger().info(f"System busy state set to: {'BUSY' if busy_state else 'IDLE'}")
        socketio.emit('system_busy_update', {'busy': self.last_system_busy_state})

    def emit_request_response_update(self):
        """Emits the latest barcode request and response to the HMI."""
        socketio.emit('request_response_update', {
            'request': {
                'barcode_data': self.last_scanned_barcode_data,
                'timestamp': self.last_scanned_barcode_timestamp
            },
            'response': {
                'success': self.last_response_success,
                'message': self.last_response_message
            }
        })

    # --- Service Call Methods (for Flask to call) ---
    # These methods encapsulate the ROS 2 service client calls.
    def call_toggle_emergency_button_service(self):
        request = ToggleEmergencyButtonState.Request()
        future = self.emergency_button_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def call_toggle_door_service(self):
        request = ToggleDoorState.Request()
        future = self.door_handle_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def call_set_stack_light_service(self, color_value):
        request = SetStackLightState.Request()
        request.state = color_value
        future = self.stack_light_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def toggle_system_busy_state(self):
        """Toggles the internal system busy state (used by HMI button, not barcode scan)."""
        self.set_system_busy(not self.last_system_busy_state)


# --- Flask Routes ---
# These routes handle HTTP requests from the web browser.

@app.route('/hmi')
def hmi():
    """Serves the main HMI HTML page."""
    return send_from_directory(app.static_folder, 'index.html')

@app.route('/get_status')
def get_status():
    """Returns the current status of all monitored components as JSON."""
    global ros_node_instance
    if ros_node_instance:
        status = {
            'emergency_button_active': ros_node_instance.last_emergency_button_state,
            'door_state': ros_node_instance.last_door_state,
            'system_busy': ros_node_instance.last_system_busy_state,
            'stack_light_status': ros_node_instance.last_stack_light_state,
            'last_request_response': {
                'request': {
                    'barcode_data': ros_node_instance.last_scanned_barcode_data,
                    'timestamp': ros_node_instance.last_scanned_barcode_timestamp
                },
                'response': {
                    'success': ros_node_instance.last_response_success,
                    'message': ros_node_instance.last_response_message
                }
            }
        }
        return jsonify(status)
    return jsonify({"error": "ROS 2 node not initialized"}), 500

@app.route('/toggle_emergency', methods=['POST'])
def toggle_emergency():
    """Flask endpoint to toggle the emergency button via its ROS 2 service."""
    global ros_node_instance
    if ros_node_instance:
        try:
            response = ros_node_instance.call_toggle_emergency_button_service()
            return jsonify({
                "success": True,
                "message": f"Emergency button toggled. New state: {'ACTIVE' if response.new_state else 'INACTIVE'}",
                "new_state": response.new_state # Use response.new_state from ROS 2 service
            })
        except Exception as e:
            ros_node_instance.get_logger().error(f"Error calling emergency button service: {e}")
            return jsonify({"success": False, "message": f"Failed to toggle emergency button: {e}"}), 500
    return jsonify({"success": False, "message": "ROS 2 node not initialized"}), 500

@app.route('/toggle_door', methods=['POST'])
def toggle_door():
    """Flask endpoint to toggle the door state via its ROS 2 service."""
    global ros_node_instance
    if ros_node_instance:
        try:
            response = ros_node_instance.call_toggle_door_service()
            return jsonify({
                "success": True,
                "message": f"Door toggled. New state: {ros_node_instance.last_door_state}", # Using cached state from sub
                "new_state": ros_node_instance.last_door_state
            })
        except Exception as e:
            ros_node_instance.get_logger().error(f"Error calling door service: {e}")
            return jsonify({"success": False, "message": f"Failed to toggle door: {e}"}), 500
    return jsonify({"success": False, "message": "ROS 2 node not initialized"}), 500

@app.route('/toggle_busy', methods=['POST'])
def toggle_busy():
    """Flask endpoint to toggle the system busy state (manual simulation)."""
    global ros_node_instance
    if ros_node_instance:
        ros_node_instance.toggle_system_busy_state()
        status_str = "BUSY" if ros_node_instance.last_system_busy_state else "IDLE"
        return jsonify({"success": True, "message": f"System busy state toggled to {status_str}.", "new_state": ros_node_instance.last_system_busy_state})
    return jsonify({"success": False, "message": "ROS 2 node not initialized"}), 500

@app.route('/set_stack_light_green', methods=['POST'])
def set_stack_light_green():
    """Flask endpoint to set stack light to GREEN."""
    global ros_node_instance
    if ros_node_instance:
        try:
            response = ros_node_instance.call_set_stack_light_service(2) # 2 is GREEN
            return jsonify({"success": response.success, "message": response.message})
        except Exception as e:
            ros_node_instance.get_logger().error(f"Error setting stack light green: {e}")
            return jsonify({"success": False, "message": f"Failed to set stack light green: {e}"}), 500
    return jsonify({"success": False, "message": "ROS 2 node not initialized"}), 500

@app.route('/set_stack_light_yellow', methods=['POST'])
def set_stack_light_yellow():
    """Flask endpoint to set stack light to YELLOW."""
    global ros_node_instance
    if ros_node_instance:
        try:
            response = ros_node_instance.call_set_stack_light_service(1) # 1 is YELLOW
            return jsonify({"success": response.success, "message": response.message})
        except Exception as e:
            ros_node_instance.get_logger().error(f"Error setting stack light yellow: {e}")
            return jsonify({"success": False, "message": f"Failed to set stack light yellow: {e}"}), 500
    return jsonify({"success": False, "message": "ROS 2 node not initialized"}), 500

@app.route('/set_stack_light_red', methods=['POST'])
def set_stack_light_red():
    """Flask endpoint to set stack light to RED."""
    global ros_node_instance
    if ros_node_instance:
        try:
            response = ros_node_instance.call_set_stack_light_service(0) # 0 is RED
            return jsonify({"success": response.success, "message": response.message})
        except Exception as e:
            ros_node_instance.get_logger().error(f"Error setting stack light red: {e}")
            return jsonify({"success": False, "message": f"Failed to set stack light red: {e}"}), 500
    return jsonify({"success": False, "message": "ROS 2 node not initialized"}), 500

@app.route('/scan_barcode', methods=['POST'])
def scan_barcode():
    """
    Flask endpoint to trigger a barcode scan.
    This calls the ROS 2 scan_barcode_service.
    """
    global ros_node_instance
    if not ros_node_instance:
        return jsonify({"success": False, "message": "ROS 2 node not initialized"}), 500

    data = request.get_json()
    barcode_data = data.get('barcode_data', '')

    if not barcode_data:
        return jsonify({"success": False, "message": "Barcode data is required."}), 400

    current_timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

    # Call the ROS 2 scan_barcode_service
    request_ros = ScanBarcode.Request()
    request_ros.barcode_data = barcode_data
    request_ros.timestamp = current_timestamp

    try:
        ros_node_instance.get_logger().info(f"HMI requesting barcode scan for: {barcode_data}")
        future = ros_node_instance.scan_barcode_service.call_async(request_ros)
        rclpy.spin_until_future_complete(ros_node_instance, future)
        response_ros = future.result()

        # The scan_barcode_callback already updates self.last_scanned_barcode_data,
        # self.last_scanned_barcode_timestamp, self.last_response_success, and
        # self.last_response_message, and emits to SocketIO.
        # So we just return the result from the ROS 2 service call here.
        return jsonify({
            "success": response_ros.success,
            "message": response_ros.message,
            "barcode_data": barcode_data,
            "timestamp": current_timestamp
        })

    except Exception as e:
        ros_node_instance.get_logger().error(f"Error calling scan_barcode_service: {e}")
        ros_node_instance.last_response_success = False
        ros_node_instance.last_response_message = f"Failed to call barcode service: {e}"
        ros_node_instance.emit_request_response_update() # Ensure HMI is updated on error
        return jsonify({"success": False, "message": f"Failed to communicate with barcode service: {e}"}), 500

@app.route('/get_last_barcode', methods=['GET'])
def get_last_barcode():
    """
    Flask endpoint to get the last scanned barcode data.
    This calls the ROS 2 get_last_barcode_service.
    """
    global ros_node_instance
    if not ros_node_instance:
        return jsonify({"success": False, "message": "ROS 2 node not initialized"}), 500

    # Call the ROS 2 get_last_barcode_service
    request_ros = GetLastBarcode.Request()

    try:
        future = ros_node_instance.get_last_barcode_service.call_async(request_ros)
        rclpy.spin_until_future_complete(ros_node_instance, future)
        response_ros = future.result()

        return jsonify({
            "success": response_ros.success,
            "barcode_data": response_ros.barcode_data,
            "timestamp": response_ros.timestamp,
            "message": response_ros.message
        })

    except Exception as e:
        ros_node_instance.get_logger().error(f"Error calling get_last_barcode_service: {e}")
        return jsonify({"success": False, "message": f"Failed to communicate with get_last_barcode service: {e}"}), 500


# --- Flask Thread ---
def run_flask():
    """Starts the Flask-SocketIO server."""
    # Use socketio.run() instead of app.run() when using Flask-SocketIO
    socketio.run(app, host='127.0.0.1', port=5000, debug=False, allow_unsafe_werkzeug=True) # debug=True is for development, set to False in production

# --- Main function for ROS 2 node ---
def main(args=None):
    global ros_node_instance
    rclpy.init(args=args)
    ros_node_instance = ApiHandlerNode()

    # Start Flask app in a separate thread.
    flask_thread = threading.Thread(target=run_flask)
    flask_thread.daemon = True # Daemon threads exit when the main program exits
    flask_thread.start()
    ros_node_instance.get_logger().info("Flask app started in a separate thread.")

    try:
        # Keep the ROS 2 node spinning to process callbacks from subscriptions and service calls.
        rclpy.spin(ros_node_instance)
    except KeyboardInterrupt:
        ros_node_instance.get_logger().info('API Handler Node stopped cleanly.')
    finally:
        # Clean up ROS 2 resources on shutdown.
        ros_node_instance.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()