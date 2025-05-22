# Bin Picking Cell Control System (ROS 2 & Web HMI)

## 1. Overview

This project implements a control system for a simulated Bin Picking Cell, integrating ROS 2 with a real-time web-based Human-Machine Interface (HMI). The system demonstrates core robotics concepts such as state management, inter-node communication (publishers, subscribers, services), and interaction with an external user interface.

The primary goal is to provide a comprehensive, modular solution for monitoring and controlling key aspects of a bin picking operation, including safety mechanisms (emergency stop, door interlock) and operational status (system busy, stack light indication), all accessible and controllable via a modern web browser.

## 2. Features

* **Real-time Status Monitoring:**
    * Emergency Button state (Active/Inactive)
    * Door Handle state (Open/Closed)
    * System Busy status (Busy/Idle)
    * Stack Light indication (Red, Yellow, Green)
    * Last Barcode Scan Request/Response details
* **Web-based HMI:** Intuitive graphical interface for displaying current system states and sending commands.
* **ROS 2 Integration:** Seamless communication between web HMI and ROS 2 nodes using Flask-SocketIO.
* **Safety Interlocks:** Barcode scan/pick requests are denied if the Emergency Button is active or the Door is open.
* **Modular Node Design:** Separate ROS 2 nodes for distinct functionalities (Emergency Button, Door Handle, Stack Light, API Handler).
* **Custom ROS 2 Services:** Defined for commanding actions (e.g., toggling button/door states, setting stack light, handling barcode scans).

## 3. System Architecture

The system is structured as a combination of ROS 2 nodes and a web application, communicating through a central API handler.

+----------------+       +-------------------+       +--------------------+
|                |       |                   |       |                    |
| Emergency      |       | Door Handle Node  |       | Stack Light Node   |
| Button Node    |       | (Publishes /door_ |       | (Publishes /stack_ |
| (Publishes /em |-----> | _state, Provides  |-----> | _light_status,     |
| button_state, |       | toggle_door_state)|       | Provides           |
| Provides toggle|       | service)          |       | set_stack_light   |
| _em_button_state)|     |                   |       | _state service)    |
| service)       |       |                   |       |                    |
+----------------+       +-------------------+       +---------^----------+
|                       |                             |
|         ROS 2 Topics & Services                     |
v                       v                             |
+-------------------------------------------------------------------------+
|                                  API Handler Node                        |
|                     (ROS 2 Node + Flask-SocketIO Web Server)            |
|                                                                         |
|  - Subscribes to: /emergency_button_state, /door_state, /stack_light_status |
|  - Publishes to: /system_busy_status                                    |
|  - Provides ROS 2 services: scan_barcode_service, get_last_barcode_service |
|  - Acts as client for ROS 2 services: toggle_emergency_button_state, toggle_door_state, set_stack_light_state |
|  - Flask HTTP Endpoints for HMI (e.g., /scan_barcode, /get_last_barcode, /hmi) |
|  - Socket.IO for real-time status updates to HMI (WebSockets)           |
+-------------------------------------------------------------------------+
^                               ^
|      HTTP POST requests       |  Socket.IO (WebSockets) for
|      for commands/data        |  real-time updates
v                               v
+-------------------------------------------------------------------------+
|                        Human-Machine Interface (HMI)                    |
|                        (Web Browser: HTML, CSS, JavaScript)             |
|                                                                         |
|  - Displays current states received from ROS 2                          |
|  - Sends commands (e.g., toggle button, toggle door, scan barcode)      |
+-------------------------------------------------------------------------+


## 4. Prerequisites

To set up and run this project, ensure you have the following installed:

* **ROS 2 [YOUR_ROS2_DISTRO]**:
    * For example, `Humble Hawksbill`, `Iron Irwini`, or `Jazzy Jalisco`.
    * Refer to the [official ROS 2 documentation](https://docs.ros.org/en/foxy/Installation.html) (adjust for your specific distro) for installation instructions.
* **Python 3.8+**: This is typically the default Python version for recent ROS 2 distributions.
* **pip**: Python package installer (usually comes with Python).
* **Node.js & npm/yarn**: While not strictly required for this simple HMI, it's good practice for managing future frontend dependencies.

## 5. Setup Instructions

Follow these steps to set up and build your ROS 2 workspace:

1.  **Create a ROS 2 Workspace:**
    ```bash
    mkdir -p ~/bin_picking_ws/src
    cd ~/bin_picking_ws/src
    ```

2.  **Create the ROS 2 Package and Place Files:**
    Create a new ROS 2 Python package (e.g., `api_handler`) within your `src` directory. Place all provided Python node files (`emergency_button_node.py`, `door_handle_node.py`, `stack_light_node.py`, `api_handler_node.py`) into this package.

    Also, ensure the `hmi_static` directory (containing `index.html`, `script.js`, `style.css`) is placed inside your package.

    Your directory structure should look similar to this:
    ```
    ~/bin_picking_ws/src/
    └── api_handler/
        ├── api_handler_node.py
        ├── emergency_button_node.py
        ├── door_handle_node.py
        ├── stack_light_node.py
        ├── hmi_static/
        │   ├── index.html
        │   ├── script.js
        │   └── style.css
        ├── srv/
        │   ├── ScanBarcode.srv
        │   ├── GetLastBarcode.srv
        │   ├── ToggleEmergencyButtonState.srv
        │   ├── ToggleDoorState.srv
        │   └── SetStackLightState.srv
        ├── package.xml
        └── setup.py
    ```

    **Example `package.xml` (within `api_handler` directory):**
    ```xml
    <?xml version="1.0"?>
    <?xml-model href="[http://download.ros.org/schema/package_format3.xsd](http://download.ros.org/schema/package_format3.xsd)" schematypens="[http://www.ros.org/ProjectSchema](http://www.ros.org/ProjectSchema)"?>
    <package format="3">
      <name>api_handler</name>
      <version>0.0.0</version>
      <description>Package for bin picking cell control and HMI integration</description>
      <maintainer email="your.email@example.com">Your Name</maintainer>
      <license>Apache License 2.0</license>

      <buildtool_depend>ament_cmake</buildtool_depend>
      <buildtool_depend>ament_python</buildtool_depend>
      <buildtool_depend>rosidl_default_generators</buildtool_depend>

      <depend>rclpy</depend>
      <depend>std_msgs</depend>

      <exec_depend>rosidl_default_runtime</exec_depend>
      <member_of_group>rosidl_interface_packages</member_of_group>

      <export>
        <build_type>ament_python</build_type>
      </export>
    </package>
    ```

    **Example `setup.py` (within `api_handler` directory):**
    ```python
    from setuptools import find_packages, setup
    import os
    from glob import glob

    package_name = 'api_handler'

    setup(
        name=package_name,
        version='0.0.0',
        packages=find_packages(exclude=['test']),
        data_files=[
            ('share/ament_index/resource_index/packages',
                ['resource/' + package_name]),
            ('share/' + package_name, ['package.xml']),
            # Include launch files if you have any
            (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yeml]'))),
            # Copy HMI static files to the install directory
            (os.path.join('share', package_name, 'hmi_static'), glob('hmi_static/*')),
        ],
        install_requires=['setuptools', 'Flask', 'Flask-SocketIO', 'eventlet'],
        zip_safe=True,
        maintainer='Your Name',
        maintainer_email='your.email@example.com',
        description='A ROS 2 package for bin picking cell control and HMI integration.',
        license='Apache License 2.0',
        tests_require=['pytest'],
        entry_points={
            'console_scripts': [
                'api_handler_node = api_handler.api_handler_node:main',
                'emergency_button_node = api_handler.emergency_button_node:main',
                'door_handle_node = api_handler.door_handle_node:main',
                'stack_light_node = api_handler.stack_light_node:main',
            ],
        },
    )
    ```

    **Custom Service Definitions (`.srv` files):**
    Ensure these files are correctly placed in the `srv` sub-directory of your `api_handler` package.
    * `ScanBarcode.srv`:
        ```
        string barcode_data
        string timestamp
        ---
        bool success
        string message
        ```
    * `GetLastBarcode.srv`:
        ```
        ---
        string barcode_data
        string timestamp
        bool success
        string message
        ```
    * `ToggleEmergencyButtonState.srv`:
        ```
        ---
        bool new_state
        string message
        ```
    * `ToggleDoorState.srv`:
        ```
        ---
        bool new_state
        string message
        ```
    * `SetStackLightState.srv`:
        ```
        int8 state # 0: RED, 1: YELLOW, 2: GREEN
        ---
        bool success
        string message
        ```

3.  **Install Python Dependencies:**
    Navigate to your workspace root (`~/bin_picking_ws/`) and install the required Python packages for Flask and SocketIO.
    ```bash
    cd ~/bin_picking_ws/
    pip install Flask Flask-SocketIO eventlet
    ```

4.  **Build the Workspace:**
    From your workspace root (`~/bin_picking_ws/`), build your package.
    ```bash
    colcon build --packages-select api_handler
    ```
    *(Replace `api_handler` with your actual package name if different)*

5.  **Source the Workspace:**
    After a successful build, you must source your ROS 2 environment to make the new nodes and services discoverable.
    ```bash
    source install/setup.bash
    ```
    *Tip: For persistent sourcing, add this line to your `~/.bashrc` or `~/.zshrc` file.*

## 6. How to Run the System

To run the entire Bin Picking Cell Control System, open **four separate terminal windows**. In each new terminal, **ensure you have sourced your ROS 2 workspace** (as done in step 5 of Setup Instructions).

1.  **Launch `emergency_button_node`:**
    ```bash
    ros2 run api_handler emergency_button_node
    ```
    This node will start in an "INACTIVE" state and continuously publish its status.

2.  **Launch `door_handle_node`:**
    ```bash
    ros2 run api_handler door_handle_node
    ```
    This node will start with the door in a "CLOSED" state and periodically publish its status.

3.  **Launch `stack_light_node`:**
    ```bash
    ros2 run api_handler stack_light_node
    ```
    This node will initialize its state to "RED" and publish its status.

4.  **Launch `api_handler_node` (Flask/SocketIO Server):**
    ```bash
    ros2 run api_handler api_handler_node
    ```
    This node will start the Flask web server, which listens for HTTP requests and handles real-time Socket.IO communication. It will report the address where it's listening (e.g., `http://0.0.0.0:5000` or `http://127.0.0.1:5000`).

5.  **Access the HMI in your Web Browser:**
    Open your preferred web browser and navigate to the address reported by the `api_handler_node`, typically:
    `http://127.0.0.1:5000/hmi`

## 7. HMI Usage

Once the HMI is loaded in your browser:

* **Observe Real-time Status:** The dashboard will immediately display the current states of the Emergency Button, Door Handle, System Busy status, and Stack Light, updating in real-time as events occur.
* **Toggle Emergency Button:** Click the "Toggle Emergency Button" to change its state between "ACTIVE" (pressed) and "INACTIVE" (released).
* **Toggle Door Handle:** Click the "Toggle Door Handle" to switch the door state between "OPEN" and "CLOSED".
* **Scan Barcode:** Enter any text into the "Barcode Data" input field and click "Scan Barcode". The system will process this request, applying safety interlocks (it will be denied if the Emergency Button is active or the Door is open). The "Last Request/Response" area will update with the outcome.
* **Toggle System Busy:** Click the "Toggle System Busy" button to manually switch the system's busy state between "BUSY" and "IDLE". Note that successful barcode scan operations automatically set the system to "BUSY" temporarily.

## 8. Node Descriptions

This section provides a brief overview of each ROS 2 node's responsibility:

* **`emergency_button_node.py`**:
    * **Role:** Simulates a physical emergency stop button.
    * **Publishes:** `/emergency_button_state` (std_msgs/Bool) - reports whether the button is `True` (active/pressed) or `False` (inactive/released).
    * **Provides Service:** `toggle_emergency_button_state` (api_handler/srv/ToggleEmergencyButtonState) - allows external entities (like the HMI via `api_handler_node`) to programmatically change its state.

* **`door_handle_node.py`**:
    * **Role:** Simulates a door sensor, indicating if a protective barrier is open or closed.
    * **Publishes:** `/door_state` (std_msgs/Bool) - reports `True` if the door is `CLOSED` and `False` if it's `OPEN`.
    * **Provides Service:** `toggle_door_state` (api_handler/srv/ToggleDoorState) - enables programmatic control of the door's state.

* **`stack_light_node.py`**:
    * **Role:** Simulates a multi-color industrial stack light, typically used for visual status indication.
    * **Publishes:** `/stack_light_status` (std_msgs/Int8) - reports the current color: `0` for RED, `1` for YELLOW, `2` for GREEN.
    * **Provides Service:** `set_stack_light_state` (api_handler/srv/SetStackLightState) - allows external nodes to directly set the light's color.

* **`api_handler_node.py`**:
    * **Role:** The central integration point, acting as a bridge between the ROS 2 ecosystem and the web HMI. It's a ROS 2 node that also embeds a Flask-SocketIO web server.
    * **Subscribes to:**
        * `/emergency_button_state`
        * `/door_state`
        * `/stack_light_status`
        (To receive real-time status updates from other ROS 2 nodes).
    * **Publishes to:** `/system_busy_status` (std_msgs/Bool) - indicates if the overall system is busy (e.g., processing a pick request).
    * **Acts as Client for ROS 2 Services:**
        * `toggle_emergency_button_state`
        * `toggle_door_state`
        * `set_stack_light_state`
        (To send commands to the respective ROS 2 nodes based on HMI interaction).
    * **Provides ROS 2 Services (for other ROS 2 nodes to call):**
        * `scan_barcode_service` (api_handler/srv/ScanBarcode) - Simulates a barcode scan and applies safety interlocks (Emergency Button, Door State).
        * `get_last_barcode_service` (api_handler/srv/GetLastBarcode) - Retrieves the data of the last successfully scanned barcode.
    * **Provides Flask HTTP Endpoints (for HMI web requests):**
        * `/hmi`: Serves the main `index.html` file to the browser.
        * `/scan_barcode` (POST): Accepts barcode data from the HMI to trigger a scan process.
        * `/get_last_barcode` (GET): Retrieves the last scanned barcode data for HMI display.
    * **Handles Socket.IO Events (for real-time HMI updates):**
        * `emergency_state_update`: Pushes emergency button status to HMI.
        * `door_state_update`: Pushes door status to HMI.
        * `system_busy_update`: Pushes system busy status to HMI.
        * `stack_light_update`: Pushes stack light color to HMI.
        * `request_response_update`: Pushes the latest barcode scan request/response details to HMI.

## 9. Custom Services (`.srv` files)

The project defines several custom ROS 2 service message types to facilitate specific communication patterns. These files are located in the `srv` directory of the `api_handler` package.

* `ScanBarcode.srv`:
    ```
    string barcode_data
    string timestamp
    ---
    bool success
    string message
    ```
    *Used for:* Requesting a barcode scan operation with input data and receiving its success status and a message.

* `GetLastBarcode.srv`:
    ```
    ---
    string barcode_data
    string timestamp
    bool success
    string message
    ```
    *Used for:* Retrieving the details of the most recently processed barcode scan.

* `ToggleEmergencyButtonState.srv`:
    ```
    ---
    bool new_state
    string message
    ```
    *Used for:* Requesting the emergency button's state to be flipped. The response includes the new state and a confirmation message.

* `ToggleDoorState.srv`:
    ```
    ---
    bool new_state
    string message
    ```
    *Used for:* Requesting the door's state to be flipped. The response includes the new state and a confirmation message.

* `SetStackLightState.srv`:
    ```
    int8 state # 0: RED, 1: YELLOW, 2: GREEN
    ---
    bool success
    string message
    ```
    *Used for:* Requesting the stack light to change to a specific color. The response indicates success and a message.

## 10. Future Improvements (Optional)

Consider these enhancements for further development:

* **Enhanced Error Handling:** Implement more robust error detection, reporting, and recovery mechanisms across all nodes and the HMI.
* **Persistent Configuration:** Allow saving and loading node configurations (e.g., initial states, QoS settings) to/from files.
* **Advanced HMI Visualizations:** Incorporate more dynamic and interactive graphical elements, such as animated robot models, bin content visualization, or historical data charts.
* **Database Integration:** Implement a database (e.g., SQLite, PostgreSQL) to store historical barcode scan data, system events, and operational logs.
* **Security:** Add authentication and authorization layers to the web HMI for secure access in a production environment.
* **Containerization:** Package the entire system (ROS 2 nodes + Flask app) using Docker for simplified deployment and environment consistency.
* **Automated Testing:** Develop unit tests for individual node logic and integration tests for inter-node communication and HMI interaction.
* **Launch Files:** Create ROS 2 launch files to simplify the startup process of all nodes from a single command.

---