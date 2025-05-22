document.addEventListener('DOMContentLoaded', (event) => {
    // Initialize Socket.IO connection to the Flask server.
    // This allows real-time, bidirectional communication for status updates.
    const socket = io('http://127.0.0.1:5000'); 

    // Get HTML elements by their IDs.
    // These are the elements on the HMI webpage where status information will be displayed.
    const emergencyStatus = document.getElementById('emergencyStatus');
    const doorStatus = document.getElementById('doorStatus');
    const systemBusyStatus = document.getElementById('systemBusyStatus');
    const stackLight = document.getElementById('stackLight');
    const stackLightValue = document.getElementById('stackLightValue');
    const reqBarcode = document.getElementById('reqBarcode');
    const reqTimestamp = document.getElementById('reqTimestamp');
    const resSuccess = document.getElementById('resSuccess');
    const resMessage = document.getElementById('resMessage');

    // Socket.IO event listeners
    socket.on('connect', () => {
        console.log('Connected to Socket.IO server');
    });

    socket.on('emergency_state_update', (data) => {
        if (data.active) {
            emergencyStatus.textContent = 'ACTIVE';
            emergencyStatus.className = 'status-text active';
        } else {
            emergencyStatus.textContent = 'INACTIVE';
            emergencyStatus.className = 'status-text inactive';
        }
    });

    socket.on('door_state_update', (data) => {
        if (data.closed) {
            doorStatus.textContent = 'CLOSED';
            doorStatus.className = 'status-text closed';
        } else {
            doorStatus.textContent = 'OPEN';
            doorStatus.className = 'status-text open';
        }
    });

    socket.on('system_busy_update', (data) => {
        if (data.busy) {
            systemBusyStatus.textContent = 'BUSY';
            systemBusyStatus.className = 'status-text busy';
        } else {
            systemBusyStatus.textContent = 'IDLE';
            systemBusyStatus.className = 'status-text idle';
        }
    });

    socket.on('stack_light_update', (data) => {
        stackLight.className = 'stack-light'; 
        let valueText = `Value: ${data.value}`;

        switch (data.value) {
            case 0:
                stackLight.classList.add('green');
                valueText += ' (GREEN)';
                break;
            case 1:
                stackLight.classList.add('yellow');
                valueText += ' (YELLOW)';
                break;
            case -1: 
                stackLight.classList.add('red');
                valueText += ' (RED)';
                break;
            case 2: 
                stackLight.classList.add('blue');
                valueText += ' (BLUE)';
                break;
            default:
                stackLight.classList.add('unknown');
                valueText += ' (UNKNOWN)';
                break;
        }
        stackLightValue.textContent = valueText;
    });

    socket.on('request_response_update', (data) => {
        if (data.request) {
            reqBarcode.textContent = data.request.barcode_data || 'N/A';
            reqTimestamp.textContent = data.request.timestamp || 'N/A';
        }
        if (data.response) {
            resSuccess.textContent = data.response.success ? 'True' : 'False';
            resSuccess.className = data.response.success ? 'status-text inactive' : 'status-text active'; // Green for success, red for false
            resMessage.textContent = data.response.message || 'N/A';
        }
    });
});