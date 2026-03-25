// Update status every 100ms (10Hz)
let statusInterval;
let isControlling = false;

// Function to adjust body padding based on header height
function adjustBodyPadding() {
    const header = document.querySelector('.fixed-header');
    if (header) {
        const headerHeight = header.offsetHeight;
        document.body.style.paddingTop = (headerHeight + 20) + 'px'; // Add 20px margin
    }
}

// Initialize when page loads
document.addEventListener('DOMContentLoaded', function() {
    adjustBodyPadding();
    initializeControls();
    startStatusUpdates();
    
    // Recalculate padding when window is resized
    window.addEventListener('resize', adjustBodyPadding);
});

function initializeControls() {
    // Slider value displays
    document.getElementById('position-slider').addEventListener('input', function(e) {
        document.getElementById('position-slider-value').textContent = e.target.value;
    });
    
    document.getElementById('speed-slider').addEventListener('input', function(e) {
        document.getElementById('speed-slider-value').textContent = e.target.value;
    });
    
    document.getElementById('force-slider').addEventListener('input', function(e) {
        document.getElementById('force-slider-value').textContent = e.target.value;
    });
    
    // Button handlers
    document.getElementById('activate-btn').addEventListener('click', activateGripper);
    document.getElementById('open-btn').addEventListener('click', () => moveToPosition(0));
    document.getElementById('half-btn').addEventListener('click', () => moveToPosition(128));
    document.getElementById('close-btn').addEventListener('click', () => moveToPosition(255));
    document.getElementById('move-btn').addEventListener('click', moveToCustomPosition);
}

function startStatusUpdates() {
    statusInterval = setInterval(updateStatus, 100);
}

function stopStatusUpdates() {
    clearInterval(statusInterval);
}

async function updateStatus() {
    try {
        const response = await fetch('/api/status');
        
        if (response.ok) {
            const data = await response.json();
            updateStatusDisplay(data);
            updateDebugOutput(data);
            document.getElementById('connection-status').textContent = 'Connected';
            document.getElementById('connection-status').className = 'value active';
        } else {
            document.getElementById('connection-status').textContent = 'Error';
            document.getElementById('connection-status').className = 'value inactive';
        }
    } catch (error) {
        document.getElementById('connection-status').textContent = 'Disconnected';
        document.getElementById('connection-status').className = 'value inactive';
    }
}

function updateStatusDisplay(data) {
    // Activated status
    const activatedEl = document.getElementById('activated-status');
    activatedEl.textContent = data.is_activated ? 'Yes' : 'No';
    activatedEl.className = data.is_activated ? 'value active' : 'value inactive';
    
    // Moving status
    const movingEl = document.getElementById('moving-status');
    movingEl.textContent = data.is_moving ? 'Yes' : 'No';
    movingEl.className = data.is_moving ? 'value warning' : 'value';
    
    // Object detected status
    const objectEl = document.getElementById('object-status');
    objectEl.textContent = data.object_detected ? 'Yes' : 'No';
    objectEl.className = data.object_detected ? 'value warning' : 'value';
    
    // Fault status
    const faultEl = document.getElementById('fault-status');
    faultEl.textContent = data.fault ? 'Error' : 'OK';
    faultEl.className = data.fault ? 'value inactive' : 'value active';
    
    // Position and force
    document.getElementById('position-value').textContent = data.position;
    document.getElementById('force-value').textContent = data.force;
    
    // Update visual gripper
    updateGripperVisual(data.position);
}

function updateGripperVisual(position) {
    // Position: 0 = open (max gap), 255 = closed (fingers touching)
    const percentage = (position / 255) * 100;
    
    // Update position bar
    document.getElementById('position-fill').style.width = percentage + '%';
    
    // Update finger positions
    // Left finger: starts at calc(50% - 50px), which is 50px left of center
    // Right finger: starts at calc(50% - 50px) from right, which is 50px right of center
    // Each finger is 30px wide
    // Left finger's right edge is at 50px - 30px = 20px left of center initially
    // To touch at center, left finger needs to move 20px right
    // When position = 0 (open): movement = 0
    // When position = 255 (closed): movement = 20px (inner edges meet at center)
    const maxMovement = 22; // Just enough to make inner edges touch at center
    const movement = maxMovement * (position / 255);
    
    document.getElementById('left-finger').style.transform = `translateX(${movement}px)`;
    document.getElementById('right-finger').style.transform = `translateX(-${movement}px)`;
}

function updateDebugOutput(data) {
    const output = document.getElementById('debug-output');
    const timestamp = new Date(data.timestamp * 1000).toLocaleTimeString();
    
    const debugInfo = `Last Update: ${timestamp}
    
Status:
  Activated: ${data.is_activated}
  Moving: ${data.is_moving}
  Object Detected: ${data.object_detected}
  Fault: ${data.fault}
  
Values:
  Position: ${data.position}
  Force: ${data.force}
  
Raw Registers:
  ${data.raw_registers.map((reg, i) => `Register ${i}: 0x${reg.toString(16).padStart(4, '0').toUpperCase()}`).join('\n  ')}`;
    
    output.textContent = debugInfo;
}

function showMessage(elementId, message, type) {
    const msgEl = document.getElementById(elementId);
    msgEl.textContent = message;
    msgEl.className = `message ${type}`;
    
    setTimeout(() => {
        msgEl.style.display = 'none';
    }, 5000);
}

async function activateGripper() {
    const btn = document.getElementById('activate-btn');
    btn.disabled = true;
    btn.textContent = 'Activating...';
    
    try {
        const response = await fetch('/api/activate', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json'
            }
        });
        
        const data = await response.json();
        
        if (data.success) {
            showMessage('activate-message', 'Gripper activated successfully!', 'success');
        } else {
            showMessage('activate-message', `Activation failed: ${data.message}`, 'error');
        }
    } catch (error) {
        showMessage('activate-message', `Error: ${error.message}`, 'error');
    } finally {
        btn.disabled = false;
        btn.textContent = 'Activate Gripper';
    }
}

async function moveToPosition(position, speed = null, force = null) {
    if (isControlling) {
        showMessage('control-message', 'Please wait for current command to complete', 'info');
        return;
    }
    
    isControlling = true;
    
    // Use slider values if not specified
    if (speed === null) {
        speed = parseInt(document.getElementById('speed-slider').value);
    }
    if (force === null) {
        force = parseInt(document.getElementById('force-slider').value);
    }
    
    // Disable all control buttons
    const buttons = document.querySelectorAll('.control-section .btn');
    buttons.forEach(btn => btn.disabled = true);
    
    showMessage('control-message', `Moving to position ${position}...`, 'info');
    
    try {
        const response = await fetch('/api/control', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json'
            },
            body: JSON.stringify({
                position: position,
                speed: speed,
                force: force
            })
        });
        
        const data = await response.json();
        
        if (data.success) {
            const objectMsg = data.object_detected ? ' (Object detected!)' : '';
            showMessage('control-message', 
                `Movement complete! Final position: ${data.final_position}${objectMsg}`, 
                'success');
        } else {
            showMessage('control-message', `Control failed: ${data.message || 'Unknown error'}`, 'error');
        }
    } catch (error) {
        showMessage('control-message', `Error: ${error.message}`, 'error');
    } finally {
        isControlling = false;
        buttons.forEach(btn => btn.disabled = false);
    }
}

async function moveToCustomPosition() {
    const position = parseInt(document.getElementById('position-slider').value);
    await moveToPosition(position);
}
