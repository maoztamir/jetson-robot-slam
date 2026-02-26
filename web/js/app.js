/**
 * Robot Trajectory Dashboard
 *
 * Real-time 2D trajectory viewer using Leaflet + AWS IoT MQTT over WebSocket.
 * Historical data from DynamoDB via AWS SDK v2 (browser bundle).
 *
 * Configuration: edit the CONFIG object below with your AWS resource IDs.
 */

// ============================================================================
// Configuration -- update these after CloudFormation deploy
// ============================================================================
const CONFIG = {
  region: "us-east-2",
  identityPoolId: "us-east-2:bad003b9-173e-4ac7-a257-bd60f6699d49",  // Cognito Identity Pool ID
  iotEndpoint: "aw4tyjjeoxel4-ats.iot.us-east-2.amazonaws.com",     // IoT data endpoint
  tableName: "RobotTrajectory",
  // SLAM origin for coordinate conversion (must match Lambda env vars)
  originLat: 32.0853,
  originLon: 34.7818,
  // Metres per degree (WGS-84 approximation)
  metresPerDegLat: 111320.0,
};

// Derived: metres per degree longitude at the origin latitude
CONFIG.metresPerDegLon =
  CONFIG.metresPerDegLat * Math.cos((CONFIG.originLat * Math.PI) / 180);

// ============================================================================
// Globals
// ============================================================================
let map;
let credentials;
let dynamodb;
let mqttClient;
let slamOnlyMode = false; // true when no GPS — uses blank grid map

// Per-device state: { deviceId: { polyline, marker, points: [[lat,lng],...] } }
const devices = {};
let activeDevice = null;

// Sensor status per device: { deviceId: { camera: "ok", ... } }
const sensorStatus = {};

// ============================================================================
// Local mode (no AWS)
// ============================================================================
let localMode = false;
let localLastTimestamp = 0;
let localTrajectoryTimer = null;
let localStatusTimer = null;

function isLocalMode() {
  return CONFIG.identityPoolId.includes("REPLACE_ME");
}

function initLocalMode() {
  localMode = true;
  const statusDot = document.getElementById("connection-status");
  statusDot.className = "status-dot connected";

  // Set default device
  const select = document.getElementById("device-select");
  select.innerHTML = '<option value="">Waiting for data...</option>';

  // Poll trajectory every 2s
  pollTrajectory();
  localTrajectoryTimer = setInterval(pollTrajectory, 2000);

  // Poll sensor status every 5s
  pollSensorStatus();
  localStatusTimer = setInterval(pollSensorStatus, 5000);
}

async function pollTrajectory() {
  try {
    const url = `/api/trajectory?since=${localLastTimestamp}`;
    const resp = await fetch(url);
    if (!resp.ok) return;
    const poses = await resp.json();
    if (!Array.isArray(poses) || poses.length === 0) return;

    poses.forEach((pose) => {
      const deviceId = pose.device_id || "unknown";
      // imu_only records: adapt z→y for the trajectory handler
      if (pose.type === "imu_only" && pose.position) {
        pose.position.y = pose.position.z;
      }
      if (pose.position) handleTrajectoryMessage(deviceId, pose);

      // Track latest timestamp for incremental polling
      const ts = pose.timestamp || 0;
      if (ts > localLastTimestamp) localLastTimestamp = ts;

      // Auto-select first device we see
      if (!activeDevice) {
        addDeviceOption(deviceId);
        selectDeviceLocal(deviceId);
      } else {
        addDeviceOption(deviceId);
      }
    });
  } catch (err) {
    console.warn("Local trajectory poll failed:", err);
  }
}

async function pollSensorStatus() {
  try {
    const resp = await fetch("/api/status");
    if (!resp.ok) return;
    const data = await resp.json();
    if (!data.device_id) return;

    const deviceId = data.device_id;
    const sensors = data.sensors || {};
    sensorStatus[deviceId] = sensors;

    // If GPS is not active, switch to blank SLAM map
    if (sensors.gps === "n/a" || sensors.gps === "error") {
      if (!slamOnlyMode) switchToSlamMap();
    }

    if (deviceId === activeDevice || !activeDevice) {
      updateSensorPanel(sensorStatus[deviceId]);
      document.getElementById("status-timestamp").textContent =
        "Updated: " + new Date().toLocaleTimeString();
    }

    addDeviceOption(deviceId);
  } catch (err) {
    console.warn("Local status poll failed:", err);
  }
}

function selectDeviceLocal(deviceId) {
  if (!deviceId) return;
  activeDevice = deviceId;

  const select = document.getElementById("device-select");
  select.value = deviceId;

  // Update stats from existing data
  if (devices[deviceId]) {
    document.getElementById("stat-points").textContent =
      devices[deviceId].points.length.toString();
  }
  if (sensorStatus[deviceId]) {
    updateSensorPanel(sensorStatus[deviceId]);
  }
}

// ============================================================================
// Initialisation
// ============================================================================
document.addEventListener("DOMContentLoaded", async () => {
  initMap();

  if (isLocalMode()) {
    console.log("Local mode: identityPoolId not configured, polling local API");
    initLocalMode();
  } else {
    await initAWS();
    await loadDevices();
    initMQTT();
  }

  document.getElementById("btn-refresh").addEventListener("click", () => {
    if (localMode) {
      localLastTimestamp = 0;
      // Clear existing data and re-poll
      Object.keys(devices).forEach((id) => {
        if (devices[id].polyline) map.removeLayer(devices[id].polyline);
        if (devices[id].marker) map.removeLayer(devices[id].marker);
      });
      Object.keys(devices).forEach((id) => delete devices[id]);
      pollTrajectory();
      pollSensorStatus();
    } else {
      loadDevices();
    }
  });

  document.getElementById("device-select").addEventListener("change", (e) => {
    if (localMode) {
      selectDeviceLocal(e.target.value);
    } else {
      selectDevice(e.target.value);
    }
  });
});

// ============================================================================
// Map
// ============================================================================
function initMap() {
  map = L.map("map", {
    center: [CONFIG.originLat, CONFIG.originLon],
    zoom: 18,
    zoomControl: true,
  });

  L.tileLayer("https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png", {
    maxZoom: 22,
    attribution: "&copy; OpenStreetMap contributors",
  }).addTo(map);
}

/** Switch to a blank grid map for SLAM-only (no GPS) mode. */
function switchToSlamMap() {
  if (slamOnlyMode) return;
  slamOnlyMode = true;

  // Remove existing map and recreate with CRS.Simple (pixel coordinates)
  const container = map.getContainer();
  map.remove();

  map = L.map(container, {
    crs: L.CRS.Simple,
    center: [0, 0],
    zoom: 2,
    zoomControl: true,
  });

  // Add a subtle grid background
  const gridSize = 200; // metres
  const gridLines = [];
  for (let i = -gridSize; i <= gridSize; i += 10) {
    gridLines.push(L.polyline([[i, -gridSize], [i, gridSize]], { color: "#333", weight: 0.5, opacity: 0.3 }));
    gridLines.push(L.polyline([[-gridSize, i], [gridSize, i]], { color: "#333", weight: 0.5, opacity: 0.3 }));
  }
  const grid = L.layerGroup(gridLines).addTo(map);

  // Add origin marker
  L.circleMarker([0, 0], {
    radius: 4,
    color: "#666",
    fillColor: "#666",
    fillOpacity: 0.5,
  }).addTo(map).bindTooltip("Origin (0,0)", { permanent: false });

  // Re-add existing device layers
  Object.keys(devices).forEach((id) => {
    const dev = devices[id];
    if (dev.points.length > 0) {
      dev.polyline = L.polyline(dev.points, {
        color: "#00b4d8", weight: 3, opacity: 0.8,
      }).addTo(map);
      dev.marker = L.circleMarker(dev.points[dev.points.length - 1], {
        radius: 6, color: "#06d6a0", fillColor: "#06d6a0", fillOpacity: 1,
      }).addTo(map);
    }
  });

  console.log("Switched to SLAM-only blank grid map (no GPS)");
}

// ============================================================================
// AWS Credentials (Cognito)
// ============================================================================
async function initAWS() {
  AWS.config.region = CONFIG.region;
  AWS.config.credentials = new AWS.CognitoIdentityCredentials({
    IdentityPoolId: CONFIG.identityPoolId,
  });

  await new Promise((resolve, reject) => {
    AWS.config.credentials.get((err) => {
      if (err) {
        console.error("Cognito auth failed:", err);
        reject(err);
      } else {
        resolve();
      }
    });
  });

  credentials = AWS.config.credentials;
  dynamodb = new AWS.DynamoDB({ region: CONFIG.region });
  console.log("AWS credentials obtained");
}

// ============================================================================
// DynamoDB -- Load devices and sessions
// ============================================================================
async function loadDevices() {
  const select = document.getElementById("device-select");

  try {
    // Scan for distinct device_ids (small table, OK to scan)
    const result = await dynamodb
      .scan({
        TableName: CONFIG.tableName,
        ProjectionExpression: "device_id",
      })
      .promise();

    const deviceIds = [...new Set(result.Items.map((i) => i.device_id.S))];
    deviceIds.sort();

    select.innerHTML = "";
    if (deviceIds.length === 0) {
      select.innerHTML = '<option value="">No devices found</option>';
      return;
    }

    deviceIds.forEach((id) => {
      const opt = document.createElement("option");
      opt.value = id;
      opt.textContent = id;
      select.appendChild(opt);
    });

    // Auto-select first device
    if (!activeDevice || !deviceIds.includes(activeDevice)) {
      selectDevice(deviceIds[0]);
      select.value = deviceIds[0];
    }
  } catch (err) {
    console.error("Failed to load devices:", err);
    select.innerHTML = '<option value="">Error loading devices</option>';
  }
}

async function selectDevice(deviceId) {
  if (!deviceId) return;
  activeDevice = deviceId;

  // Load trajectory from DynamoDB
  await loadTrajectory(deviceId);

  // Load sessions
  await loadSessions(deviceId);

  // Update sensor panel if we have status
  if (sensorStatus[deviceId]) {
    updateSensorPanel(sensorStatus[deviceId]);
  } else {
    clearSensorPanel();
  }
}

async function loadTrajectory(deviceId) {
  try {
    const result = await dynamodb
      .query({
        TableName: CONFIG.tableName,
        KeyConditionExpression: "device_id = :did",
        ExpressionAttributeValues: { ":did": { S: deviceId } },
        ScanIndexForward: true,
      })
      .promise();

    // Clear existing device layer
    if (devices[deviceId]) {
      if (devices[deviceId].polyline) map.removeLayer(devices[deviceId].polyline);
      if (devices[deviceId].marker) map.removeLayer(devices[deviceId].marker);
    }

    // Check first record for GPS data to decide map mode
    if (result.Items.length > 0 && !result.Items[0].gps) {
      if (!slamOnlyMode) switchToSlamMap();
    }

    const points = result.Items
      .filter((item) => item.position && item.position.M)  // skip records without position
      .map((item) => {
        const pos = item.position.M;
        const x = parseFloat(pos.x.N);
        const y = parseFloat(pos.y.N);
        const z = pos.z ? parseFloat(pos.z.N) : 0;
        // Use GPS coordinates if available
        if (item.gps && item.gps.M) {
          const lat = parseFloat(item.gps.M.lat.N);
          const lon = parseFloat(item.gps.M.lon.N);
          return [lat, lon];
        }
        // SLAM pose: y=north, x=east.  IMU dead-reckoning: x=east, z=forward (use as north).
        const isImuOnly = item.type && item.type.S === "imu_only";
        return isImuOnly ? slamToLatLng(x, z) : slamToLatLng(x, y);
      });

    if (points.length === 0) {
      devices[deviceId] = { polyline: null, marker: null, points: [] };
      document.getElementById("stat-points").textContent = "0";
      return;
    }

    const polyline = L.polyline(points, {
      color: "#00b4d8",
      weight: 3,
      opacity: 0.8,
    }).addTo(map);

    const lastPoint = points[points.length - 1];
    const marker = L.circleMarker(lastPoint, {
      radius: 6,
      color: "#06d6a0",
      fillColor: "#06d6a0",
      fillOpacity: 1,
    }).addTo(map);

    devices[deviceId] = { polyline, marker, points };
    map.fitBounds(polyline.getBounds(), { padding: [40, 40] });

    document.getElementById("stat-points").textContent = points.length.toString();
    document.getElementById("stat-last-update").textContent = new Date().toLocaleTimeString();
  } catch (err) {
    console.error("Failed to load trajectory:", err);
  }
}

async function loadSessions(deviceId) {
  const list = document.getElementById("session-list");

  try {
    const result = await dynamodb
      .query({
        TableName: CONFIG.tableName,
        KeyConditionExpression: "device_id = :did",
        ExpressionAttributeValues: { ":did": { S: deviceId } },
        ProjectionExpression: "#ts",
        ExpressionAttributeNames: { "#ts": "timestamp" },
        ScanIndexForward: true,
      })
      .promise();

    const timestamps = result.Items.map((i) => parseFloat(i.timestamp.N));

    // Group by time gaps > 60s
    const sessions = [];
    let sessionStart = 0;
    for (let i = 0; i < timestamps.length; i++) {
      if (i === 0 || timestamps[i] - timestamps[i - 1] > 60) {
        if (i > 0) {
          sessions.push({ start: timestamps[sessionStart], end: timestamps[i - 1], count: i - sessionStart });
        }
        sessionStart = i;
      }
    }
    // Last session
    if (timestamps.length > 0) {
      sessions.push({
        start: timestamps[sessionStart],
        end: timestamps[timestamps.length - 1],
        count: timestamps.length - sessionStart,
      });
    }

    list.innerHTML = "";
    if (sessions.length === 0) {
      list.innerHTML = "<li>No sessions</li>";
      return;
    }

    sessions.forEach((s, idx) => {
      const li = document.createElement("li");
      const startTime = new Date(s.start * 1000).toLocaleString();
      const duration = Math.round(s.end - s.start);
      li.textContent = `#${idx + 1}: ${startTime} (${duration}s, ${s.count} pts)`;
      li.title = `${s.count} points over ${duration}s`;
      list.appendChild(li);
    });
  } catch (err) {
    console.error("Failed to load sessions:", err);
    list.innerHTML = "<li>Error loading sessions</li>";
  }
}

// ============================================================================
// MQTT over WebSocket (using Paho)
// ============================================================================
function initMQTT() {
  const statusDot = document.getElementById("connection-status");
  statusDot.className = "status-dot connecting";

  // Use AWS SDK SigV4 signed URL for IoT WebSocket
  const endpoint = CONFIG.iotEndpoint;
  const clientId = "dashboard-" + Math.random().toString(36).substring(2, 10);

  // Build a signed WebSocket URL
  const url = buildSignedUrl(endpoint, credentials);

  // Paho.MQTT.Client(uri, clientId) for WebSocket connections
  mqttClient = new Paho.MQTT.Client(url, clientId);

  mqttClient.onConnectionLost = (resp) => {
    console.warn("MQTT disconnected:", resp.errorMessage);
    statusDot.className = "status-dot disconnected";
    // Reconnect after 3s
    setTimeout(() => initMQTT(), 3000);
  };

  mqttClient.onMessageArrived = (msg) => {
    try {
      const topic = msg.destinationName;
      const payload = JSON.parse(msg.payloadString);
      handleMqttMessage(topic, payload);
    } catch (e) {
      console.warn("Bad MQTT message:", e);
    }
  };

  mqttClient.connect({
    onSuccess: () => {
      console.log("MQTT connected");
      statusDot.className = "status-dot connected";

      // Subscribe to all robot trajectory, IMU, and status topics
      mqttClient.subscribe("robot/+/trajectory", { qos: 0 });
      mqttClient.subscribe("robot/+/imu", { qos: 0 });
      mqttClient.subscribe("robot/+/status", { qos: 0 });
    },
    onFailure: (err) => {
      console.error("MQTT connect failed:", err);
      statusDot.className = "status-dot disconnected";
      setTimeout(() => initMQTT(), 5000);
    },
    useSSL: true,
    timeout: 10,
  });
}

/**
 * Build a SigV4-signed WebSocket URL for AWS IoT.
 * Uses the AWS SDK v2 credentials that are already refreshed.
 * Crypto via AWS.util.crypto (bundled in the SDK v2 browser build).
 */
function buildSignedUrl(endpoint, creds) {
  const now = new Date();
  const dateStamp = now.toISOString().replace(/[-:]/g, "").substring(0, 8);
  const amzDate = now.toISOString().replace(/[-:]/g, "").replace(/\.\d+Z/, "Z");

  const service = "iotdevicegateway";
  const region = CONFIG.region;
  const method = "GET";
  const canonicalUri = "/mqtt";

  const credentialScope = `${dateStamp}/${region}/${service}/aws4_request`;

  const canonicalQuerystring = [
    `X-Amz-Algorithm=AWS4-HMAC-SHA256`,
    `X-Amz-Credential=${encodeURIComponent(creds.accessKeyId + "/" + credentialScope)}`,
    `X-Amz-Date=${amzDate}`,
    `X-Amz-SignedHeaders=host`,
  ]
    .sort()
    .join("&");

  const canonicalHeaders = `host:${endpoint}\n`;
  const signedHeaders = "host";

  const canonicalRequest = [
    method,
    canonicalUri,
    canonicalQuerystring,
    canonicalHeaders,
    signedHeaders,
    "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855", // empty payload hash
  ].join("\n");

  const stringToSign = [
    "AWS4-HMAC-SHA256",
    amzDate,
    credentialScope,
    AWS.util.crypto.sha256(canonicalRequest, "hex"),
  ].join("\n");

  // HMAC chain for the signing key
  const signingKey = getSignatureKey(creds.secretAccessKey, dateStamp, region, service);
  const signature = AWS.util.crypto.hmac(signingKey, stringToSign, "hex");

  let url =
    `wss://${endpoint}${canonicalUri}?${canonicalQuerystring}` +
    `&X-Amz-Signature=${signature}`;

  if (creds.sessionToken) {
    url += `&X-Amz-Security-Token=${encodeURIComponent(creds.sessionToken)}`;
  }

  return url;
}

function getSignatureKey(secretKey, dateStamp, region, service) {
  const kDate = AWS.util.crypto.hmac("AWS4" + secretKey, dateStamp, "buffer");
  const kRegion = AWS.util.crypto.hmac(kDate, region, "buffer");
  const kService = AWS.util.crypto.hmac(kRegion, service, "buffer");
  const kSigning = AWS.util.crypto.hmac(kService, "aws4_request", "buffer");
  return kSigning;
}

// ============================================================================
// MQTT Message Handling
// ============================================================================
function handleMqttMessage(topic, payload) {
  // Topic format: robot/<device_id>/trajectory or robot/<device_id>/status
  const parts = topic.split("/");
  if (parts.length !== 3) return;

  const deviceId = parts[1];
  const msgType = parts[2];

  if (msgType === "trajectory") {
    handleTrajectoryMessage(deviceId, payload);
  } else if (msgType === "imu") {
    // IMU-only record — render trajectory if dead-reckoned position is present
    if (payload.position) handleTrajectoryMessage(deviceId, payload);
  } else if (msgType === "status") {
    handleStatusMessage(deviceId, payload);
  }
}

function handleTrajectoryMessage(deviceId, payload) {
  const pos = payload.position;
  if (!pos) return;

  // Check if this record has real GPS data
  const hasGps = payload.gps && payload.gps.lat != null && payload.gps.lon != null;

  let latLng;
  if (hasGps) {
    latLng = [payload.gps.lat, payload.gps.lon];
  } else {
    // No GPS — use SLAM coordinates directly on a blank grid
    if (!slamOnlyMode) switchToSlamMap();
    latLng = [pos.y, pos.x]; // SLAM y=north, x=east → [lat, lng] in CRS.Simple
  }

  // Ensure device entry exists
  if (!devices[deviceId]) {
    devices[deviceId] = { polyline: null, marker: null, points: [] };
  }

  const dev = devices[deviceId];
  dev.points.push(latLng);

  // Update or create polyline
  if (dev.polyline) {
    dev.polyline.addLatLng(latLng);
  } else {
    dev.polyline = L.polyline(dev.points, {
      color: "#00b4d8",
      weight: 3,
      opacity: 0.8,
    }).addTo(map);
  }

  // Update or create marker (current position)
  if (dev.marker) {
    dev.marker.setLatLng(latLng);
  } else {
    dev.marker = L.circleMarker(latLng, {
      radius: 6,
      color: "#06d6a0",
      fillColor: "#06d6a0",
      fillOpacity: 1,
    }).addTo(map);
  }

  // Update stats if this is the active device
  if (deviceId === activeDevice) {
    document.getElementById("stat-points").textContent = dev.points.length.toString();
    document.getElementById("stat-last-update").textContent = new Date().toLocaleTimeString();
  }

  // Add device to dropdown if new
  addDeviceOption(deviceId);
}

function handleStatusMessage(deviceId, payload) {
  sensorStatus[deviceId] = payload.sensors || {};

  // Update panel if this is the active device
  if (deviceId === activeDevice) {
    updateSensorPanel(sensorStatus[deviceId]);
    document.getElementById("status-timestamp").textContent =
      "Updated: " + new Date().toLocaleTimeString();
  }

  // Add device to dropdown if new
  addDeviceOption(deviceId);
}

// ============================================================================
// Sensor Panel
// ============================================================================
function updateSensorPanel(sensors) {
  const sensorNames = ["camera", "imu", "slam", "gps"];
  sensorNames.forEach((name) => {
    const dot = document.getElementById(`sensor-${name}`);
    if (!dot) return;

    const status = sensors[name] || "n/a";
    dot.className = "sensor-dot";
    if (status === "ok") dot.classList.add("ok");
    else if (status === "mock") dot.classList.add("mock");
    else if (status === "error") dot.classList.add("error");
    else dot.classList.add("na");

    dot.title = `${name}: ${status}`;
  });
}

function clearSensorPanel() {
  ["camera", "imu", "slam", "gps"].forEach((name) => {
    const dot = document.getElementById(`sensor-${name}`);
    if (dot) {
      dot.className = "sensor-dot na";
      dot.title = `${name}: unknown`;
    }
  });
  document.getElementById("status-timestamp").textContent = "";
}

// ============================================================================
// Helpers
// ============================================================================

/** Convert SLAM x/y (metres) to Leaflet [lat, lng].
 *  In SLAM-only mode (no GPS), returns raw [y, x] for CRS.Simple. */
function slamToLatLng(x, y) {
  if (slamOnlyMode) {
    return [y, x];
  }
  const lat = CONFIG.originLat + y / CONFIG.metresPerDegLat;
  const lng = CONFIG.originLon + x / CONFIG.metresPerDegLon;
  return [lat, lng];
}

/** Add a device to the dropdown if not already present. */
function addDeviceOption(deviceId) {
  const select = document.getElementById("device-select");
  const existing = Array.from(select.options).find((o) => o.value === deviceId);
  if (!existing) {
    const opt = document.createElement("option");
    opt.value = deviceId;
    opt.textContent = deviceId;
    select.appendChild(opt);

    // Auto-select if it's the first device
    if (!activeDevice) {
      selectDevice(deviceId);
      select.value = deviceId;
    }
  }
}
