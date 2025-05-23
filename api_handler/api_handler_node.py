# This MUST be the first thing executed in the script to ensure eventlet monkey-patches correctly.
# eventlet is used by Flask-SocketIO for asynchronous operations,
# and monkey-patching ensures standard library functions become non-blocking.
import eventlet
eventlet.monkey_patch()

# Now, all other imports can follow.
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from flask import Flask, jsonify, request, send_from_directory
from flask_socketio import SocketIO, emit

import threading
import time
import os

# Import custom ROS 2 service definitions. These are generated from .srv files.
from api_handler.srv import ScanBarcode, GetLastBarcode, ToggleEmergencyButtonState, ToggleDoorState
# Import standard ROS 2 message types.
from std_msgs.msg import Bool, Int8 # Int8 for stack light status

# --- Flask App Setup (GLOBAL) ---
# Initializes the Flask web application.
# `static_folder` and `template_folder` point to the directory containing HMI files (HTML, CSS, JS).
app = Flask(__name__, static_folder='hmi_static', template_folder='hmi_static')
# Initializes Flask-SocketIO for real-time, bidirectional communication with the HMI.
# `cors_allowed_origins="*"` allows connections from any origin (useful for development).
# `async_mode='eventlet'` specifies the asynchronous framework to use.
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='eventlet')

# --- ROS2 Node Class ---
class ApiHandlerNode(Node):
    """
    ROS 2 Node that serves as the central API handler for the Bin Picking Cell.
    It bridges ROS 2 communication (topics, services) with a web-based Human-Machine Interface (HMI)
    using Flask and SocketIO for real-time updates and command handling.
    """

    def __init__(self):
        super().__init__('api_handler_node')
        self.get_logger().info('API Handler Node starting up...')

        # Quality of Service (QoS) profile for reliable communication over topics.
        # - RELIABLE: Guarantees delivery of all messages.
        # - KEEP_LAST: Stores a finite number of messages.
        # - DEPTH: Number of messages to store (10 in this case).
        # - TRANSIENT_LOCAL: Ensures late-joining subscribers receive the last published message.
        #   IMPORTANT: This must match publishers in other nodes for reliable latching.
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # Internal State Variables (to store the latest status from ROS 2 topics)
        # These variables are updated by ROS 2 subscribers and then emitted to the HMI.
        self.is_emergency_active = False
        self.is_door_closed = True # Default assumption for the door state
        self.last_scanned_barcode = "N/A"
        self.last_scanned_timestamp = "N/A"
        self.is_system_busy = False # Tracks if the system is 'busy' (e.g., during a pick operation)
        self.current_stack_light_value = 0 # Stores the current stack light status (0:RED, 1:YELLOW, 2:GREEN)


        # --- ROS 2 Publishers ---
        # Publishers send messages to ROS 2 topics.
        self.system_busy_publisher = self.create_publisher(Bool, '/system_busy_status', qos_profile)
        self.get_logger().info('Publisher for /system_busy_status created.')
        # Immediately publish initial system busy status after publisher creation.
        # This allows late-joining subscribers (like the stack light node) to get the current state.
        self.publish_system_busy_status(self.is_system_busy, "initial state")


        # --- ROS 2 Subscribers ---
        # Subscribers receive messages from ROS 2 topics.
        self.emergency_button_subscriber = self.create_subscription(
            Bool,
            '/emergency_button_state',
            self.emergency_button_callback,
            qos_profile
        )
        self.get_logger().info('Subscriber for /emergency_button_state created.')

        self.door_state_subscriber = self.create_subscription(
            Bool,
            '/door_state',
            self.door_state_callback,
            qos_profile
        )
        self.get_logger().info('Subscriber for /door_state created.')

        self.stack_light_subscriber = self.create_subscription(
            Int8, # Expects an Int8 message for stack light status
            '/stack_light_status',
            self.stack_light_callback,
            qos_profile
        )
        self.get_logger().info('Subscriber for /stack_light_status created.')

        # --- ROS 2 Service Servers ---
        # Service servers implement the logic to respond to incoming service requests.
        self.scan_barcode_service = self.create_service(ScanBarcode, 'scan_barcode_service', self.scan_barcode_callback)
        self.get_logger().info('ROS 2 service servers for "scan_barcode_service" and "get_last_barcode_service" created.')

        self.get_last_barcode_service = self.create_service(GetLastBarcode, 'get_last_barcode_service', self.get_last_barcode_callback)


    # --- ROS 2 Subscriber Callback Methods ---
    # These methods are executed when new data arrives on the subscribed topics.
    # They update the node's internal state and then trigger an HMI update.
    def emergency_button_callback(self, msg):
        """Updates internal state based on /emergency_button_state and emits to HMI."""
        self.is_emergency_active = msg.data
        if self.is_emergency_active:
            self.get_logger().warn(f'Received emergency button state update: ACTIVE (Denying requests!) (Bool: {msg.data})')
        else:
            self.get_logger().info(f'Received emergency button state update: INACTIVE (Bool: {msg.data})')
        self.emit_current_hmi_state() # Emit full state to all connected HMI clients

    def door_state_callback(self, msg):
        """Updates internal state based on /door_state and emits to HMI."""
        self.is_door_closed = msg.data
        door_status_str = "CLOSED" if self.is_door_closed else "OPEN"
        self.get_logger().info(f'Received door state update: {door_status_str} (Bool: {msg.data})')
        self.emit_current_hmi_state() # Emit full state to all connected HMI clients

    def stack_light_callback(self, msg):
        """Updates internal state based on /stack_light_status and emits to HMI."""
        self.current_stack_light_value = msg.data
        self.get_logger().info(f'Received stack light state update: {msg.data}')
        self.emit_current_hmi_state() # Emit full state to all connected HMI clients


    # --- ROS 2 Service Server Callback Methods ---
    # These methods handle requests from other ROS 2 nodes to this node's services.
    def scan_barcode_callback(self, request, response):
        """
        Handles the 'scan_barcode_service' request from other ROS 2 nodes.
        Simulates a barcode scan and pick operation, applying safety interlocks.
        """
        self.get_logger().info(f'ROS 2 Service: Received scan_barcode request: barcode_data="{request.barcode_data}", timestamp="{request.timestamp}"')
        self.publish_system_busy_status(True, "scan request received") # Indicate system is busy during processing

        # Decision logic for denying pick requests based on system safety/state
        if self.is_emergency_active:
            response.success = False
            response.message = 'Pick request denied: EMERGENCY BUTTON ACTIVE. Please deactivate to proceed.'
            self.get_logger().error(f'ROS 2 Service: Pick request for "{request.barcode_data}" DENIED: EMERGENCY ACTIVE.')
            self.publish_system_busy_status(False, "emergency denial") # Clear busy status on denial
        elif not self.is_door_closed: # Door is OPEN
            response.success = False
            response.message = 'Pick request denied: Door is OPEN. Please close the door to proceed.'
            self.get_logger().warn(f'ROS 2 Service: Pick request for "{request.barcode_data}" denied due to open door.')
            self.publish_system_busy_status(False, "door denial") # Clear busy status on denial
        else:
            # Simulate processing time for the pick operation
            self.get_logger().info(f'ROS 2 Service: Processing pick request for "{request.barcode_data}"...')
            time.sleep(1) # Blocking sleep to simulate work (eventlet handles this non-blocking for web server)

            # Update internal state with scanned barcode data
            self.last_scanned_barcode = request.barcode_data
            self.last_scanned_timestamp = request.timestamp
            response.success = True
            response.message = 'Barcode scanned and pick request processed successfully.'
            self.get_logger().info(f'ROS 2 Service: Sending scan_barcode response (success: {response.success}).')
            self.publish_system_busy_status(False, "scan complete") # Clear system busy status

        # Emit request/response info to HMI for immediate feedback on barcode actions
        socketio.emit('request_response_update', {
            'request': {'barcode_data': request.barcode_data, 'timestamp': request.timestamp},
            'response': {'success': response.success, 'message': response.message}
        })
        return response

    def get_last_barcode_callback(self, request, response):
        """
        Handles the 'get_last_barcode_service' request from other ROS 2 nodes.
        Returns the data of the last successfully scanned barcode.
        """
        self.get_logger().info('ROS 2 Service: Received get_last_barcode request.')
        response.barcode_data = self.last_scanned_barcode
        response.timestamp = self.last_scanned_timestamp
        response.success = True
        self.get_logger().info(f'ROS 2 Service: Sending get_last_barcode response (barcode: {response.barcode_data}, timestamp: {response.timestamp}).')
        # This service typically doesn't change core system status, so no need for emit_current_hmi_state here.
        return response

    # --- Utility Methods ---
    def publish_system_busy_status(self, is_busy, reason=""):
        """
        Publishes the current system busy status to the /system_busy_status topic.
        Also updates the internal state and triggers an HMI update.
        """
        msg = Bool()
        msg.data = is_busy
        self.system_busy_publisher.publish(msg)
        self.is_system_busy = is_busy # Update internal state
        self.get_logger().info(f'Published system busy status: {is_busy}' + (f' ({reason})' if reason else ''))
        self.emit_current_hmi_state() # Emit full state to ensure HMI reflects busy status change


    # --- Flask Endpoint Methods ---
    # These methods are wrapped by Flask's `add_url_rule` and handle HTTP requests from the HMI.
    # They call internal ROS 2 service callbacks or update internal states.
    def flask_scan_barcode_endpoint(self):
        """
        Flask endpoint to handle barcode scan requests from the HMI (via HTTP POST).
        It calls the internal logic that mirrors the ROS 2 'scan_barcode_service'.
        """
        barcode_data = request.json.get('barcode_data')
        timestamp = request.json.get('timestamp')

        self.get_logger().info(f'Flask API: Received scan_barcode request via HTTP: barcode_data="{barcode_data}", timestamp="{timestamp}"')
        self.publish_system_busy_status(True, "flask scan request received") # Indicate system is busy

        # Re-using the same safety interlock logic as the ROS 2 service callback
        if self.is_emergency_active:
            message = 'Pick request denied: EMERGENCY BUTTON ACTIVE. Please deactivate to proceed.'
            success = False
            self.get_logger().error(f'Flask API: Pick request for "{barcode_data}" DENIED: EMERGENCY ACTIVE.')
            self.publish_system_busy_status(False, "emergency denial")
        elif not self.is_door_closed:
            message = 'Pick request denied: Door is OPEN. Please close the door to proceed.'
            success = False
            self.get_logger().warn(f'Flask API: Pick request for "{barcode_data}" denied due to open door.')
            self.publish_system_busy_status(False, "door denial")
        else:
            self.get_logger().info(f'Flask API: Processing pick request for "{barcode_data}"...')
            time.sleep(1) # Simulate work (non-blocking due to eventlet)

            self.last_scanned_barcode = barcode_data
            self.last_scanned_timestamp = timestamp
            message = 'Barcode scanned and pick request processed successfully.'
            success = True
            self.publish_system_busy_status(False, "flask scan complete")

        # Emit request/response info to HMI via SocketIO for real-time feedback
        socketio.emit('request_response_update', {
            'request': {'barcode_data': barcode_data, 'timestamp': timestamp},
            'response': {'success': success, 'message': message}
        })
        return jsonify(success=success, message=message)

    def flask_get_last_barcode_endpoint(self):
        """
        Flask endpoint to handle requests for the last scanned barcode data (via HTTP GET).
        """
        self.get_logger().info('Flask API: Received get_last_barcode request via HTTP.')
        return jsonify(barcode_data=self.last_scanned_barcode, timestamp=self.last_scanned_timestamp, success=True)

    # --- Centralized method to emit all current HMI states ---
    def emit_current_hmi_state(self):
        """
        Emits the complete current state of the system to all connected HMI clients
        via SocketIO. This ensures the HMI is always synchronized with the backend.
        """
        self.get_logger().debug('Emitting full HMI state to connected clients.')
        socketio.emit('emergency_state_update', {'active': self.is_emergency_active})
        socketio.emit('door_state_update', {'closed': self.is_door_closed})
        socketio.emit('stack_light_update', {'value': self.current_stack_light_value})
        socketio.emit('system_busy_update', {'busy': self.is_system_busy})
        # Also send the last barcode request/response for display on HMI
        socketio.emit('request_response_update', {
            'request': {'barcode_data': self.last_scanned_barcode, 'timestamp': self.last_scanned_timestamp},
            'response': {'success': True if self.last_scanned_barcode != "N/A" else False, 'message': 'Last barcode data.'}
        })


# --- Main Execution Block ---

def main(args=None):
    """
    Initializes the ROS 2 context, creates the API Handler node,
    and starts the Flask-SocketIO web server in a separate thread.
    The main thread is then used to spin the ROS 2 node.
    """
    rclpy.init(args=args) # Initialize ROS 2 client library
    node = ApiHandlerNode() # Instantiate the ROS 2 node

    # --- DEFINE FLASK ROUTES AND SOCKETIO HANDLERS HERE (GLOBAL SCOPE) ---
    # These lines associate URL paths with the corresponding methods of the ApiHandlerNode instance.
    app.add_url_rule('/scan_barcode', 'scan_barcode', node.flask_scan_barcode_endpoint, methods=['POST'])
    app.add_url_rule('/get_last_barcode', 'get_last_barcode', node.flask_get_last_barcode_endpoint, methods=['GET'])

    @app.route('/hmi')
    def serve_hmi():
        """Route to serve the main HMI HTML file when '/hmi' is accessed."""
        return send_from_directory(app.template_folder, 'index.html')

    @socketio.on('connect')
    def handle_connect():
        """
        SocketIO event handler for new client connections.
        When a client connects, it immediately emits the current system state
        to synchronize the HMI.
        """
        node.get_logger().info('HMI client connected via SocketIO.')
        # Emit all current states immediately upon client connection
        node.emit_current_hmi_state()


    # Run Flask-SocketIO app in a separate thread
    def run_flask_app():
        """Function to start the Flask-SocketIO server."""
        host = '0.0.0.0' # Listen on all available network interfaces
        port = 5000      # Default port for Flask
        node.get_logger().info(f'Flask API server listening on http://{host}:{port}')
        # Use eventlet's WSGI server to run the Flask app, enabling non-blocking operations.
        eventlet.wsgi.server(eventlet.listen((host, port)), app, debug=False, log_output=False)


    # Create and start a new thread for the Flask web server.
    flask_thread = threading.Thread(target=run_flask_app)
    flask_thread.daemon = True # Daemonize thread so it exits automatically when the main program exits
    flask_thread.start()

    # Add a small delay here to allow Flask server to fully initialize and
    # for QoS-latching messages to be received by ROS 2 subscribers upon startup.
    node.get_logger().info("Waiting a moment for Flask/ROS2 initial sync...")
    time.sleep(1) # Adjust this value if needed, but 1 second is usually sufficient

    node.get_logger().info("ROS 2 node spinning...")
    # Spin the ROS 2 node in the main thread to continuously process
    # incoming ROS 2 messages and service requests/responses.
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped cleanly.')
    finally:
        # Cleanly shut down the ROS 2 node and client library upon exit.
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()