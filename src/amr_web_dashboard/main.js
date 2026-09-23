// ROS Setup
const ros = new ROSLIB.Ros({
    url: 'ws://localhost:9090'
});

// UI Elements
const statusEl = document.getElementById('ros-status');
const posXEl = document.getElementById('pos-x');
const posYEl = document.getElementById('pos-y');
const posYawEl = document.getElementById('pos-yaw');
const velLinEl = document.getElementById('vel-linear');
const velAngEl = document.getElementById('vel-angular');
const canvas = document.getElementById('warehouse-map');
const ctx = canvas.getContext('2d');

// Canvas Settings
const CANVAS_WIDTH = canvas.width;
const CANVAS_HEIGHT = canvas.height;
// Map real world (-10 to 10) to canvas (0 to 800 width, 0 to 600 height)
const SCALE = 40; 
const CENTER_X = CANVAS_WIDTH / 2;
const CENTER_Y = CANVAS_HEIGHT / 2;

// AMR State (Now an array of 3 robots)
let amrStates = {
    'synapse_amr_1': { x: 8.0, y: -6.5, yaw: Math.PI },
    'synapse_amr_2': { x: -8.0, y: 5.9, yaw: 0 },
    'synapse_amr_3': { x: 7.5, y: 4.5, yaw: Math.PI }
};
let amrBattery = 100;
let activeRobot = 'synapse_amr_1';

document.getElementById('active-robot').addEventListener('change', (e) => {
    activeRobot = e.target.value;
    updateTelemetryUI();
});

// ROS Events
ros.on('connection', () => {
    statusEl.textContent = 'Connected';
    statusEl.classList.remove('disconnected');
    statusEl.classList.add('connected');
});

ros.on('error', (error) => {
    console.error('Error connecting to websocket server: ', error);
    statusEl.textContent = 'Error';
});

ros.on('close', () => {
    statusEl.textContent = 'Disconnected';
    statusEl.classList.remove('connected');
    statusEl.classList.add('disconnected');
});

// Subscribers
const modelStatesSub = new ROSLIB.Topic({
    ros: ros,
    name: '/gazebo/model_states',
    messageType: 'gazebo_msgs/msg/ModelStates'
});

modelStatesSub.subscribe((message) => {
    const names = message.name;
    const poses = message.pose;
    const twists = message.twist;

    for (let i = 0; i < names.length; i++) {
        const name = names[i];
        if (amrStates.hasOwnProperty(name)) {
            const x = poses[i].position.x;
            const y = poses[i].position.y;
            
            // Quaternion to Euler Yaw
            const q = poses[i].orientation;
            const siny_cosp = 2 * (q.w * q.z + q.x * q.y);
            const cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
            const yaw = Math.atan2(siny_cosp, cosy_cosp);

            amrStates[name].x = x;
            amrStates[name].y = y;
            amrStates[name].yaw = yaw;
            
            // We also store twist for the active robot telemetry
            amrStates[name].vLin = twists[i].linear.x;
            amrStates[name].vAng = twists[i].angular.z;
        }
    }

    updateTelemetryUI(); // rAF animation loop handles canvas redraw
});

function updateTelemetryUI() {
    const state = amrStates[activeRobot];
    if (!state) return;
    
    posXEl.textContent = `${state.x.toFixed(2)} m`;
    posYEl.textContent = `${state.y.toFixed(2)} m`;
    posYawEl.textContent = `${(state.yaw * (180/Math.PI)).toFixed(1)}°`;

    if (state.vLin !== undefined) {
        velLinEl.textContent = `${state.vLin.toFixed(2)} m/s`;
        velAngEl.textContent = `${state.vAng.toFixed(2)} rad/s`;
    }
}

// Draw functions
function drawRect(rx, ry, rw, rh, color) {
    ctx.fillStyle = color;
    // Map coords: y is up in ROS, down in Canvas
    const cx = CENTER_X + (rx * SCALE) - ((rw * SCALE) / 2);
    const cy = CENTER_Y - (ry * SCALE) - ((rh * SCALE) / 2);
    ctx.fillRect(cx, cy, rw * SCALE, rh * SCALE);
}

function drawText(text, rx, ry, color) {
    ctx.fillStyle = color;
    ctx.font = "12px Inter";
    const cx = CENTER_X + (rx * SCALE);
    const cy = CENTER_Y - (ry * SCALE);
    ctx.fillText(text, cx - 20, cy);
}

function drawWarehouseEnvironment() {
    // Background Grid
    ctx.strokeStyle = "rgba(255,255,255,0.05)";
    ctx.lineWidth = 1;
    for(let i = 0; i < CANVAS_WIDTH; i += SCALE) {
        ctx.beginPath(); ctx.moveTo(i, 0); ctx.lineTo(i, CANVAS_HEIGHT); ctx.stroke();
    }
    for(let j = 0; j < CANVAS_HEIGHT; j += SCALE) {
        ctx.beginPath(); ctx.moveTo(0, j); ctx.lineTo(CANVAS_WIDTH, j); ctx.stroke();
    }

    // Docks
    drawRect(8, -6.5, 2, 2, 'rgba(46, 204, 113, 0.3)'); // Charging
    drawText("CHG", 8, -6.5, "#2ECC71");
    
    drawRect(-8, -6.5, 2, 2, 'rgba(242, 122, 20, 0.3)'); // Inbound
    drawText("INB", -8, -6.5, "#F27A14");

    drawRect(-8, 6.5, 2, 2, 'rgba(20, 76, 166, 0.3)'); // Outbound
    drawText("OUT", -8, 6.5, "#144CA6");

    drawRect(7.5, 4.5, 2, 2, 'rgba(155, 89, 182, 0.3)'); // QC
    drawText("QC", 7.5, 4.5, "#9B59B6");

    // Aisle A Racks (y = 4.8 to 2.4 approximately, x around -4.5 to 4.5)
    // Actually from stations, Aisle A bays are at y=3.6, so racks are above and below
    const rackColor = '#59616B';
    const rackW = 2.0;
    const rackH = 1.0;
    // North Rack Row
    drawRect(-4.5, 4.8, rackW, rackH, rackColor);
    drawRect(-1.5, 4.8, rackW, rackH, rackColor);
    drawRect(1.5, 4.8, rackW, rackH, rackColor);
    drawRect(4.5, 4.8, rackW, rackH, rackColor);
    // South Rack Row
    drawRect(-4.5, 2.4, rackW, rackH, rackColor);
    drawRect(-1.5, 2.4, rackW, rackH, rackColor);
    drawRect(1.5, 2.4, rackW, rackH, rackColor);
    drawRect(4.5, 2.4, rackW, rackH, rackColor);

    // Aisle B Racks (y = -2.4 to -4.8)
    // North Rack Row
    drawRect(-4.5, -2.4, rackW, rackH, rackColor);
    drawRect(-1.5, -2.4, rackW, rackH, rackColor);
    drawRect(1.5, -2.4, rackW, rackH, rackColor);
    drawRect(4.5, -2.4, rackW, rackH, rackColor);
    // South Rack Row
    drawRect(-4.5, -4.8, rackW, rackH, rackColor);
    drawRect(-1.5, -4.8, rackW, rackH, rackColor);
    drawRect(1.5, -4.8, rackW, rackH, rackColor);
    drawRect(4.5, -4.8, rackW, rackH, rackColor);

    // --- Dead Zone: Aisle B Wi-Fi Dead Zone Overlay ---
    // Aisle B spans: x from -7 to 7 (world), y from -1.8 to -5.4 (world)
    const dzX1 = CENTER_X + (-7 * SCALE);
    const dzY1 = CENTER_Y - (-1.8 * SCALE);   // top of dead zone (higher y = lower on canvas)
    const dzX2 = CENTER_X + (7 * SCALE);
    const dzY2 = CENTER_Y - (-5.4 * SCALE);   // bottom of dead zone
    const dzW = dzX2 - dzX1;
    const dzH = dzY2 - dzY1;

    // Pulsing opacity using time
    const pulse = 0.10 + 0.07 * Math.sin(Date.now() / 500);

    // Red fill
    ctx.fillStyle = `rgba(231, 76, 60, ${pulse})`;
    ctx.fillRect(dzX1, dzY1, dzW, dzH);

    // Red border
    ctx.strokeStyle = `rgba(231, 76, 60, ${pulse * 3})`;
    ctx.lineWidth = 2;
    ctx.setLineDash([8, 5]);
    ctx.strokeRect(dzX1, dzY1, dzW, dzH);
    ctx.setLineDash([]);

    // WiFi dead zone label
    ctx.fillStyle = `rgba(231, 76, 60, ${0.5 + 0.3 * Math.sin(Date.now() / 500)})`;
    ctx.font = 'bold 11px Inter';
    ctx.textAlign = 'center';
    ctx.fillText('WI-FI DEAD ZONE', CENTER_X, CENTER_Y - (-3.6 * SCALE) - 4);
    ctx.font = '9px Inter';
    ctx.fillStyle = `rgba(231, 76, 60, 0.6)`;
    ctx.fillText('VOS SHADOW ACTIVE', CENTER_X, CENTER_Y - (-3.6 * SCALE) + 10);
    ctx.textAlign = 'left';
}

function drawAMRs() {
    for (const [name, state] of Object.entries(amrStates)) {
        const cx = CENTER_X + (state.x * SCALE);
        const cy = CENTER_Y - (state.y * SCALE); // Invert Y
        
        ctx.save();
        ctx.translate(cx, cy);
        ctx.rotate(-state.yaw); // Invert rotation for canvas
    
        // Draw footprint (highlight active robot)
        ctx.fillStyle = name === activeRobot ? '#F27A14' : '#6C7A89';
        ctx.beginPath();
        ctx.roundRect(-15, -10, 30, 20, 4);
        ctx.fill();
    
        // Direction indicator
        ctx.fillStyle = name === activeRobot ? '#144CA6' : '#2C3E50';
        ctx.beginPath();
        ctx.moveTo(15, 0);
        ctx.lineTo(5, -6);
        ctx.lineTo(5, 6);
        ctx.fill();
    
        // Glow effect for active robot
        if (name === activeRobot) {
            ctx.shadowBlur = 10;
            ctx.shadowColor = 'rgba(242, 122, 20, 0.8)';
            ctx.lineWidth = 2;
            ctx.strokeStyle = '#FFF';
            ctx.stroke();
        }
        
        ctx.restore();
        
        // Draw label
        ctx.fillStyle = "#FFF";
        ctx.font = "10px Inter";
        ctx.fillText(name.replace('synapse_amr_', 'AMR '), cx - 15, cy - 15);
    }
}

function drawVOSPath() {
    if (!vosBroadcastedPath || vosBroadcastedPath.length === 0) return;
    
    ctx.save();
    ctx.strokeStyle = '#9b59b6'; // Violet
    ctx.lineWidth = 4;
    ctx.setLineDash([8, 8]);
    ctx.shadowBlur = 10;
    ctx.shadowColor = '#9b59b6';
    
    ctx.beginPath();
    const first = vosBroadcastedPath[0];
    ctx.moveTo(CENTER_X + (first.x * SCALE), CENTER_Y - (first.y * SCALE));
    
    for (let i = 1; i < vosBroadcastedPath.length; i++) {
        const p = vosBroadcastedPath[i];
        ctx.lineTo(CENTER_X + (p.x * SCALE), CENTER_Y - (p.y * SCALE));
    }
    
    ctx.stroke();
    
    // Draw an animated pulse along the path
    const pulseIdx = Math.floor((Date.now() / 200) % vosBroadcastedPath.length);
    const pulsePoint = vosBroadcastedPath[pulseIdx];
    if (pulsePoint) {
        ctx.fillStyle = '#fff';
        ctx.beginPath();
        ctx.arc(CENTER_X + (pulsePoint.x * SCALE), CENTER_Y - (pulsePoint.y * SCALE), 5, 0, Math.PI * 2);
        ctx.fill();
    }
    
    ctx.restore();
}

function drawMap() {
    // Clear
    ctx.clearRect(0, 0, CANVAS_WIDTH, CANVAS_HEIGHT);
    
    // Static env (includes dead zone overlay)
    drawWarehouseEnvironment();
    
    // Violet broadcasted path (VOS)
    drawVOSPath();
    
    // Dynamic AMRs
    drawAMRs();
}

// Throttled animation loop — runs at max 30fps to prevent lag
let lastFrameTime = 0;
function animationLoop(timestamp) {
    if (timestamp - lastFrameTime >= 33) { // 33ms = ~30fps
        lastFrameTime = timestamp;
        drawMap();
    }
    requestAnimationFrame(animationLoop);
}

// Start the animation loop
requestAnimationFrame(animationLoop);

// Simulate battery drain (Mock since we aren't subscribing to battery topic yet)
setInterval(() => {
    if(amrBattery > 20) {
        amrBattery -= 0.1;
        document.getElementById('battery-bar').style.width = `${amrBattery}%`;
        document.getElementById('battery-val').textContent = `${Math.floor(amrBattery)}%`;
    }
}, 5000);

// Dispatch Logic
const dispatchTopics = {
    'synapse_amr_1': new ROSLIB.Topic({
        ros: ros,
        name: '/synapse_amr_1/amr_dispatch',
        messageType: 'std_msgs/msg/String'
    }),
    'synapse_amr_2': new ROSLIB.Topic({
        ros: ros,
        name: '/synapse_amr_2/amr_dispatch',
        messageType: 'std_msgs/msg/String'
    }),
    'synapse_amr_3': new ROSLIB.Topic({
        ros: ros,
        name: '/synapse_amr_3/amr_dispatch',
        messageType: 'std_msgs/msg/String'
    })
};

document.getElementById('dispatch-btn').addEventListener('click', () => {
    const targetLoad = document.getElementById('target-load').value;
    const dropoffLoc = document.getElementById('dropoff-loc').value;
    
    const msgData = JSON.stringify({
        target_load: targetLoad,
        dropoff_location: dropoffLoc
    });
    
    const msg = new ROSLIB.Message({
        data: msgData
    });
    
    dispatchTopics[activeRobot].publish(msg);
    document.getElementById('rule-engine-status').textContent = 'DISPATCHED';
    document.getElementById('rule-engine-status').style.backgroundColor = 'rgba(20, 76, 166, 0.2)';
    document.getElementById('rule-engine-status').style.color = '#2A6EE0';
    document.getElementById('rule-engine-status').style.borderColor = '#2A6EE0';
});

// Stress Test Logic
document.getElementById('stress-test-btn').addEventListener('click', () => {
    fetch('/stress_test', { method: 'POST' })
        .then(response => response.json())
        .then(data => {
            console.log("Stress test started!");
            document.getElementById('rule-engine-status').textContent = 'ST-LEASE TEST';
            document.getElementById('rule-engine-status').style.backgroundColor = 'rgba(231, 76, 60, 0.2)';
            document.getElementById('rule-engine-status').style.color = '#E74C3C';
        })
        .catch(error => console.error('Error starting stress test:', error));
});

// VOS Stress Test Logic
document.getElementById('vos-stress-test-btn').addEventListener('click', () => {
    fetch('/vos_stress_test', { method: 'POST' })
        .then(response => response.json())
        .then(data => {
            console.log("VOS Stress test started!");
            document.getElementById('rule-engine-status').textContent = 'VOS TEST';
            document.getElementById('rule-engine-status').style.backgroundColor = 'rgba(155, 89, 182, 0.2)';
            document.getElementById('rule-engine-status').style.color = '#9B59B6';
        })
        .catch(error => console.error('Error starting VOS stress test:', error));
});

// Restart Simulation Logic
document.getElementById('restart-sim-btn').addEventListener('click', async () => {
    const btn = document.getElementById('restart-sim-btn');
    const originalText = btn.innerHTML;
    
    // UI Loading state
    btn.disabled = true;
    btn.innerHTML = '<span class="btn-text">Restarting System... Please wait...</span>';
    
    try {
        const response = await fetch('/restart_sim', {
            method: 'POST'
        });
        
        if (response.ok) {
            btn.innerHTML = '<span class="btn-text">System Restarted!</span>';
            setTimeout(() => {
                btn.disabled = false;
                btn.innerHTML = originalText;
            }, 3000);
        } else {
            throw new Error('Server returned ' + response.status);
        }
    } catch (error) {
        console.error("Restart failed:", error);
        btn.innerHTML = '<span class="btn-text">Restart Failed</span>';
        btn.style.backgroundColor = '#ff0000';
        setTimeout(() => {
            btn.disabled = false;
            btn.innerHTML = originalText;
            btn.style.backgroundColor = '';
        }, 3000);
    }
});

// Restart Simulation (VOS Mode) Logic
document.getElementById('restart-vos-btn').addEventListener('click', async () => {
    const btn = document.getElementById('restart-vos-btn');
    const originalText = btn.innerHTML;
    
    // UI Loading state
    btn.disabled = true;
    btn.innerHTML = '<span class="btn-text">Restarting in VOS Mode... Please wait...</span>';
    
    try {
        const response = await fetch('/restart_sim_vos', {
            method: 'POST'
        });
        
        if (response.ok) {
            btn.innerHTML = '<span class="btn-text">System Restarted (VOS)!</span>';
            setTimeout(() => {
                btn.disabled = false;
                btn.innerHTML = originalText;
            }, 3000);
        } else {
            throw new Error('Server returned ' + response.status);
        }
    } catch (error) {
        console.error("Restart failed:", error);
        btn.innerHTML = '<span class="btn-text">Restart Failed</span>';
        btn.style.backgroundColor = '#ff0000';
        setTimeout(() => {
            btn.disabled = false;
            btn.innerHTML = originalText;
            btn.style.backgroundColor = '';
        }, 3000);
    }
});

// ST-Lease Coordination Listener
let stLeaseState = {};

const stLeaseTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/st_lease/coordination',
    messageType: 'std_msgs/String'
});

stLeaseTopic.subscribe((message) => {
    try {
        const data = JSON.parse(message.data);
        stLeaseState[data.robot_id] = data;
        updateSTLeaseTable();
    } catch (e) {
        console.error("Error parsing ST-Lease heartbeat", e);
    }
});

function updateSTLeaseTable() {
    const tbody = document.getElementById('st-lease-body');
    if (!tbody) return;
    
    // Sort robots by ID
    const robots = Object.keys(stLeaseState).sort();
    
    if (robots.length === 0) {
        return;
    }
    
    let html = '';
    for (const robot of robots) {
        const state = stLeaseState[robot];
        
        let heldNode = state.held_nodes && state.held_nodes.length > 0 ? state.held_nodes.join(', ') : '-';
        if (state.held_zone) {
            heldNode = `${heldNode} [${state.held_zone}]`;
        }
        let requestedNode = state.requested_node || '-';
        let priority = state.priority ? state.priority.toFixed(1) : '-';
        
        let status = state.status_label || "IDLE";
        let statusColor = "#a0a0a0";
        if (status.includes("ZERO-WAIT")) {
            statusColor = "#f39c12"; // Amber for Velocity Modulation
        } else if (status.includes("WINNER") || status.includes("ACQUIRED")) {
            statusColor = "#2ecc71"; // Bright Green for Active Lease Holder
        } else if (status.includes("DELIVERING") || status === "DRIVING") {
            statusColor = "#3498db"; // Blue
        } else if (status.includes("YIELD") || status.includes("WAIT")) {
            statusColor = "#e74c3c"; // Red
        } else if (heldNode !== '-') {
            status = state.status_label || "AT STATION";
            statusColor = "#3498db";
        }
        
        const shortName = robot.replace('synapse_amr_', 'AMR ');
        
        html += `
            <tr style="border-bottom: 1px solid rgba(255,255,255,0.1);">
                <td style="padding: 8px;"><strong>${shortName}</strong></td>
                <td style="padding: 8px; color: #2ecc71;">${heldNode}</td>
                <td style="padding: 8px; color: #f1c40f;">${requestedNode}</td>
                <td style="padding: 8px;">${priority}</td>
                <td style="padding: 8px; color: ${statusColor}; font-weight: bold;">${status}</td>
            </tr>
        `;
    }
    
    tbody.innerHTML = html;
}

// =============================================================
// VOS LIVE MONITOR
// =============================================================
let vosState = {};
let vosActive = false;
let vosBroadcastedPath = null;
const VOS_DEAD_ZONE_NODES = ["w_aisle_b","aisle_b_1","aisle_b_2","aisle_b_3","aisle_b_4","e_aisle_b","box_rack_b1_bay3"];


function vosLog(msg, color) {
    color = color || '#9b59b6';
    const log = document.getElementById('vos-event-log');
    if (!log) return;
    const now = new Date().toLocaleTimeString();
    const entry = document.createElement('div');
    entry.style.color = color;
    entry.style.marginBottom = '2px';
    entry.innerHTML = `<span style="color:rgba(255,255,255,0.3)">[${now}]</span> ${msg}`;
    // Remove placeholder if present
    const placeholder = log.querySelector('div[style*="0.3"]');
    if (placeholder && placeholder.innerText === 'Waiting for VOS activity...') placeholder.remove();
    log.appendChild(entry);
    log.scrollTop = log.scrollHeight;
}

function setPhase(phase) {
    for (let i = 1; i <= 4; i++) {
        const el = document.getElementById(`vos-phase-${i}`);
        if (!el) continue;
        if (i < phase) {
            el.style.background = 'rgba(46,204,113,0.15)';
            el.style.borderColor = '#2ecc71';
            el.style.color = '#2ecc71';
        } else if (i === phase) {
            el.style.background = 'rgba(155,89,182,0.3)';
            el.style.borderColor = '#9b59b6';
            el.style.color = '#fff';
            el.style.boxShadow = '0 0 12px rgba(155,89,182,0.5)';
        } else {
            el.style.background = 'transparent';
            el.style.borderColor = 'rgba(255,255,255,0.1)';
            el.style.color = 'rgba(255,255,255,0.3)';
            el.style.boxShadow = 'none';
        }
    }
}

function updateVOSRobotCard(amrId, role, statusText, cardColor) {
    const num = amrId.replace('synapse_amr_', '');
    const roleEl = document.getElementById(`vos-amr${num}-role`);
    const statusEl = document.getElementById(`vos-amr${num}-status`);
    const cardEl = document.getElementById(`vos-amr${num}-card`);
    if (roleEl) { roleEl.textContent = role; roleEl.style.color = cardColor; }
    if (statusEl) statusEl.textContent = statusText;
    if (cardEl) cardEl.style.borderColor = cardColor;
}

function updateShadowBar(silenceSecs) {
    const bar = document.getElementById('vos-shadow-bar');
    const label = document.getElementById('vos-shadow-label');
    if (!bar || !label) return;
    const pct = Math.min(100, (silenceSecs / 12) * 100);
    bar.style.width = pct + '%';
    if (silenceSecs < 3) {
        label.textContent = `${silenceSecs.toFixed(0)}s — Signal OK`;
        bar.style.background = 'linear-gradient(90deg, #2ecc71, #27ae60)';
    } else if (silenceSecs < 5) {
        label.textContent = `${silenceSecs.toFixed(0)}s — Signal Weak`;
        bar.style.background = 'linear-gradient(90deg, #f1c40f, #e67e22)';
    } else if (silenceSecs < 10) {
        label.textContent = `${silenceSecs.toFixed(0)}s — SHADOW EXPANDING`;
        bar.style.background = 'linear-gradient(90deg, #9b59b6, #e74c3c)';
    } else {
        label.textContent = `${silenceSecs.toFixed(0)}s — GOSSIP TRIGGERED`;
        bar.style.background = 'linear-gradient(90deg, #e74c3c, #c0392b)';
    }
}

let vosShadowInterval = null;

// Subscribe to VOS Status topic for choreographed simulation events
const vosStatusTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/vos/status',
    messageType: 'std_msgs/String'
});

vosStatusTopic.subscribe((message) => {
    try {
        const data = JSON.parse(message.data);
        const event = data.event;
        const robot = data.robot;
        const msgText = data.message;
        const ts = data.timestamp;

        // Activate VOS UI mode on first event
        if (!vosActive) {
            vosActive = true;
            document.getElementById('vos-mode-badge').textContent = 'ACTIVE';
            document.getElementById('vos-mode-badge').style.background = 'rgba(155,89,182,0.4)';
            vosLog('VOS Simulation Engine Started', '#9b59b6');
        }

        // Handle Choreographed Events
        switch (event) {
            case "SIMULATION_START":
                vosLog(msgText, '#fff');
                break;
            case "NAVIGATING":
                vosLog(`AMR 3: ${msgText}`, '#3498db');
                updateVOSRobotCard('synapse_amr_3', 'NAVIGATING', 'Heading to Aisle B', '#3498db');
                break;
            case "STANDBY":
                vosLog(`AMR 2: ${msgText}`, '#f39c12');
                updateVOSRobotCard('synapse_amr_2', 'STANDBY', 'Ready to monitor shadow', '#f39c12');
                break;
            case "APPROACHING":
                vosLog(`AMR 3: ${msgText}`, '#f1c40f');
                setPhase(1); // RSSI Broadcast phase
                updateVOSRobotCard('synapse_amr_3', 'RSSI WARNING', 'Signal dropping', '#f1c40f');
                break;
            case "BROADCASTING_PATH":
                vosLog(`AMR 3: ${msgText}`, '#e67e22');
                updateVOSRobotCard('synapse_amr_3', 'BROADCASTING', 'Transmitting predictive path to AMR 2', '#e67e22');
                vosBroadcastedPath = data.path;
                break;
            case "ENTERING_DEAD_ZONE":
                vosLog(`AMR 3: ${msgText}`, '#e74c3c');
                setPhase(2); // Shadow Expanding
                updateVOSRobotCard('synapse_amr_3', 'SIGNAL LOST', 'VOS Shadow Active', '#e74c3c');
                // Start shadow bar animation
                if (vosShadowInterval) clearInterval(vosShadowInterval);
                let silenceSecs = 0;
                vosShadowInterval = setInterval(() => {
                    silenceSecs += 1;
                    updateShadowBar(silenceSecs);
                }, 1000);
                break;
            case "MONITORING":
                vosLog(`AMR 2: ${msgText}`, '#9b59b6');
                setPhase(3); // Gossip / Monitoring
                updateVOSRobotCard('synapse_amr_2', 'MONITORING', 'Tracking AMR 3 Shadow', '#9b59b6');
                break;
            case "SHADOW_ACTIVE":
                vosLog(`AMR 3: ${msgText}`, '#e74c3c');
                break;
            case "CONNECTION_REGAINED":
                vosLog(`AMR 3: ${msgText}`, '#2ecc71');
                setPhase(4); // Consensus / Resolved
                updateVOSRobotCard('synapse_amr_3', 'RECONNECTED', 'Shadow cleared', '#2ecc71');
                if (vosShadowInterval) {
                    clearInterval(vosShadowInterval);
                    updateShadowBar(0);
                }
                vosBroadcastedPath = null;
                break;
            case "AMR2_RESUME":
                vosLog(`AMR 2: ${msgText}`, '#2ecc71');
                updateVOSRobotCard('synapse_amr_2', 'RESUMED', 'Normal operations', '#2ecc71');
                break;
            case "AMR1_TASK":
                vosLog(`AMR 1: ${msgText}`, '#3498db');
                updateVOSRobotCard('synapse_amr_1', 'NAVIGATING', 'Working in Aisle A', '#3498db');
                break;
            case "SIMULATION_COMPLETE":
                vosLog(msgText, '#2ecc71');
                document.getElementById('vos-mode-badge').textContent = 'COMPLETED';
                document.getElementById('vos-mode-badge').style.background = 'rgba(46,204,113,0.3)';
                document.getElementById('vos-mode-badge').style.borderColor = '#2ecc71';
                break;
        }
    } catch (e) {
        console.error("Error parsing VOS status message", e);
    }
});

// Reset VOS panel when VOS stress test button is clicked
document.getElementById('vos-stress-test-btn').addEventListener('click', () => {
    // Reset VOS state
    vosActive = false;
    vosBroadcastedPath = null;
    if (vosShadowInterval) clearInterval(vosShadowInterval);
    
    document.getElementById('vos-mode-badge').textContent = 'STARTING...';
    document.getElementById('vos-mode-badge').style.background = 'rgba(241,196,15,0.2)';
    document.getElementById('vos-mode-badge').style.borderColor = '#f1c40f';
    document.getElementById('vos-mode-badge').style.color = '#f1c40f';
    document.getElementById('vos-event-log').innerHTML = '<div style="color:#f1c40f">VOS Stress Test dispatched — waiting for robots...</div>';
    document.getElementById('vos-shadow-bar').style.width = '0%';
    document.getElementById('vos-shadow-label').textContent = '0s — No Shadow';
    setPhase(0);
    ['synapse_amr_1','synapse_amr_2','synapse_amr_3'].forEach(id => {
        const num = id.replace('synapse_amr_', '');
        const roleEl = document.getElementById(`vos-amr${num}-role`);
        const statusEl = document.getElementById(`vos-amr${num}-status`);
        if (roleEl) { roleEl.style.color = 'rgba(255,255,255,0.4)'; roleEl.textContent = 'WAITING...'; }
        if (statusEl) statusEl.textContent = 'Dispatching...';
    });
}, true); // capture=true so this fires BEFORE the fetch handler below



// Tab Switching Logic
window.switchTab = function(tabId) {
    // Hide all tab contents
    document.querySelectorAll('.tab-content').forEach(el => {
        el.style.display = 'none';
        el.classList.remove('active');
    });

    // Remove active class from all buttons
    document.querySelectorAll('.tab-btn').forEach(el => {
        el.classList.remove('active');
    });

    // Show target tab
    const targetTab = document.getElementById(tabId);
    if (targetTab) {
        targetTab.style.display = 'flex';
        targetTab.classList.add('active');
    }

    // Add active class to corresponding button
    const targetBtn = document.getElementById('btn-' + tabId);
    if (targetBtn) {
        targetBtn.classList.add('active');
    }
};

