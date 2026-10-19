#include <WiFi.h>
#include <WebServer.h>
#include <MAVLink.h>

const char* ssid = "FlyingTable";
const char* password = "12345678";

WebServer server(80);


// =====================================================
// MAVLINK
// =====================================================

#define RXD2 16
#define TXD2 17

#define PIXHAWK_SYSID 1
#define PIXHAWK_COMPID 1
#define ESP32_SYSID 42

mavlink_message_t txMsg;
mavlink_message_t rxMsg;
mavlink_status_t rxStatus;

uint8_t buffer[MAVLINK_MAX_PACKET_LEN];


// =====================================================
// VARIABLES
// =====================================================

int throttle = 0;

bool armed = false;
bool emergencyStop = false;

bool sliderTouched = false;

unsigned long lastTouchTime = 0;
unsigned long lastDecreaseTime = 0;

unsigned long lastMAVLinkSend = 0;
unsigned long lastHeartbeat = 0;

unsigned long lastWebActivity = 0;
const unsigned long WEB_TIMEOUT = 2000;  // 2 seconds

int maxThrottle = 100;

bool pixhawkArmed = false;


// =====================================================
// PIXHAWK TELEMETRY VALUES
// =====================================================

// Roll / Pitch / Yaw in degrees

float roll = 0;
float pitch = 0;
float yaw = 0;


// Motor/servo output PWM values

uint16_t motor1PWM = 0;
uint16_t motor2PWM = 0;
uint16_t motor3PWM = 0;
uint16_t motor4PWM = 0;


// Battery telemetry
float batteryVoltage = 0;
float batteryCurrent = 0;
int batteryRemaining = -1;


// =====================================================
// SEND MAVLINK MESSAGE
// =====================================================

void sendMAVLinkMessage()
{
  uint16_t len =
    mavlink_msg_to_send_buffer(
      buffer,
      &txMsg
    );

  Serial2.write(buffer, len);
}


// =====================================================
// SEND HEARTBEAT
// =====================================================

void sendHeartbeat()
{
  mavlink_msg_heartbeat_pack(
    ESP32_SYSID,
    MAV_COMP_ID_ONBOARD_COMPUTER,
    &txMsg,
    MAV_TYPE_GCS,
    MAV_AUTOPILOT_INVALID,
    0,
    0,
    MAV_STATE_ACTIVE
  );

  sendMAVLinkMessage();
}


// =====================================================
// READ MAVLINK TELEMETRY
// =====================================================

void readMAVLink()
{

  while (Serial2.available())
  {

    uint8_t c =
      Serial2.read();


    if (
      mavlink_parse_char(
        MAVLINK_COMM_0,
        c,
        &rxMsg,
        &rxStatus
      )
    )
    {

      // =================================================
      // PIXHAWK HEARTBEAT
      // =================================================

      if (
        rxMsg.msgid == MAVLINK_MSG_ID_HEARTBEAT &&
        rxMsg.sysid == PIXHAWK_SYSID
      )
      {

        mavlink_heartbeat_t heartbeat;

        mavlink_msg_heartbeat_decode(
          &rxMsg,
          &heartbeat
        );

        pixhawkArmed =
          (heartbeat.base_mode & MAV_MODE_FLAG_SAFETY_ARMED);
      }


      // =================================================
      // ATTITUDE
      // =================================================

      else if (
        rxMsg.msgid ==
        MAVLINK_MSG_ID_ATTITUDE
      )
      {

        mavlink_attitude_t attitude;

        mavlink_msg_attitude_decode(
          &rxMsg,
          &attitude
        );


        roll =
          attitude.roll *
          180.0 /
          PI;


        pitch =
          attitude.pitch *
          180.0 /
          PI;


        yaw =
          attitude.yaw *
          180.0 /
          PI;
      }


      // =================================================
      // SERVO OUTPUT
      // =================================================

      else if (
        rxMsg.msgid ==
        MAVLINK_MSG_ID_SERVO_OUTPUT_RAW
      )
      {

        mavlink_servo_output_raw_t servoOutput;

        mavlink_msg_servo_output_raw_decode(
          &rxMsg,
          &servoOutput
        );


        motor1PWM =
          servoOutput.servo1_raw;


        motor2PWM =
          servoOutput.servo2_raw;


        motor3PWM =
          servoOutput.servo3_raw;


        motor4PWM =
          servoOutput.servo4_raw;
      }


      // =================================================
      // BATTERY STATUS
      // =================================================

      else if (
        rxMsg.msgid ==
        MAVLINK_MSG_ID_BATTERY_STATUS
      )
      {

        mavlink_battery_status_t batteryStatus;

        mavlink_msg_battery_status_decode(
          &rxMsg,
          &batteryStatus
        );

        // Battery 1 = battery_id 0
        if (batteryStatus.id == 0)
        {
          // voltage[0] is in millivolts
          if (batteryStatus.voltages[0] != UINT16_MAX)
          {
            batteryVoltage =
              batteryStatus.voltages[0] / 1000.0;
          }

          // current_battery is in centi-amperes
          if (batteryStatus.current_battery != -1)
          {
            batteryCurrent =
              batteryStatus.current_battery / 100.0;
          }

          // -1 means unknown
          batteryRemaining =
            batteryStatus.battery_remaining;
        }
      }
    }


    yield();
  }
}


// =====================================================
// CONVERT PWM TO PERCENTAGE
// =====================================================

int pwmToPercent(uint16_t pwm)
{

  if (pwm <= 1000)
    return 0;

  if (pwm >= 2000)
    return 100;

  return map(
    pwm,
    1000,
    2000,
    0,
    100
  );
}


// =====================================================
// ARM PIXHAWK
// =====================================================

void armPixhawk()
{

  Serial.println(
    "Sending ARM..."
  );


  mavlink_msg_command_long_pack(

    ESP32_SYSID,

    MAV_COMP_ID_ONBOARD_COMPUTER,

    &txMsg,

    PIXHAWK_SYSID,

    PIXHAWK_COMPID,

    MAV_CMD_COMPONENT_ARM_DISARM,

    0,

    1,      // ARM

    0,

    0,

    0,

    0,

    0,

    0
  );


  sendMAVLinkMessage();
}


// =====================================================
// FORCE DISARM PIXHAWK
// =====================================================

void disarmPixhawk()
{

  Serial.println(
    "Sending FORCE DISARM..."
  );


  mavlink_msg_command_long_pack(

    ESP32_SYSID,

    MAV_COMP_ID_ONBOARD_COMPUTER,

    &txMsg,

    PIXHAWK_SYSID,

    PIXHAWK_COMPID,

    MAV_CMD_COMPONENT_ARM_DISARM,

    0,

    0,        // DISARM

    21196,    // FORCE DISARM

    0,

    0,

    0,

    0,

    0
  );


  sendMAVLinkMessage();
}


// =====================================================
// SEND THROTTLE TO PIXHAWK
// =====================================================

void sendThrottleToPixhawk(int percent)
{
  percent = constrain(percent, 0, maxThrottle);

  // 0-1000 MANUAL_CONTROL thrust scale
  int16_t thrust = percent * 10;

  mavlink_msg_manual_control_pack(
    ESP32_SYSID,                              // ESP32 system ID
    MAV_COMP_ID_ONBOARD_COMPUTER,     // ESP32 component ID
    &txMsg,

    PIXHAWK_SYSID,                    // target

    0,                                // x
    0,                                // y
    thrust,                           // z = thrust
    0,                                // r

    0,                                // buttons
    0,                                // buttons2
    0,                                // enabled_extensions

    0,                                // s
    0,                                // t
    0,                                // aux1
    0,                                // aux2
    0,                                // aux3
    0,                                // aux4
    0,                                // aux5
    0                                 // aux6
  );

  sendMAVLinkMessage();
}


// =====================================================
// WEBPAGE
// =====================================================

const char controlPage[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<style>
html, body {
  margin: 0;
  padding: 0;
  width: 100%;
  height: 100%;
  overflow: hidden;
  background: #f5f5f5;
  font-family: Arial, sans-serif;
  user-select: none;
}
.container {
  width: 100%;
  height: 100%;
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
}
.nav {
  position: absolute;
  top: 10px;
  left: 0;
  right: 0;
  text-align: center;
}
.nav a {
  display: inline-block;
  margin: 0 5px;
  padding: 8px 12px;
  background: #dddddd;
  color: #222222;
  text-decoration: none;
  border-radius: 8px;
  font-size: 14px;
}
.slider-container {
  height: 40vh;
  display: flex;
  align-items: center;
  justify-content: center;
}
input[type=range] {
  width: 40vh;
  height: 70px;
  transform: rotate(-90deg);
  appearance: none;
  background: transparent;
  touch-action: none;
}
input[type=range]::-webkit-slider-runnable-track {
  height: 45px;
  background: #cccccc;
  border-radius: 25px;
}
input[type=range]::-webkit-slider-thumb {
  appearance: none;
  width: 65px;
  height: 65px;
  border-radius: 50%;
  background: #222222;
  margin-top: -10px;
}
.max-control {
  display: flex;
  align-items: center;
  gap: 8px;
  margin-top: 5px;
}
.max-control input {
  width: 70px;
  height: 38px;
  box-sizing: border-box;
  font-size: 18px;
  text-align: center;
}
.max-control button {
  width: 80px;
  height: 38px;
  font-size: 15px;
  border: none;
  border-radius: 8px;
  background: #444444;
  color: white;
}
.telemetry {
  width: 90%;
  max-width: 600px;
  background: white;
  border-radius: 15px;
  padding: 15px;
  box-sizing: border-box;
  margin-top: 10px;
  box-shadow: 0 3px 8px rgba(0,0,0,0.15);
}
.telemetry-title {
  font-size: 20px;
  font-weight: bold;
  margin-bottom: 10px;
}
.telemetry-grid {
  display: grid;
  grid-template-columns: 1fr 1fr;
  gap: 8px;
}
.current-throttle {
  font-size: 22px;
  font-weight: bold;
  margin-top: 5px;
}
.telemetry-value {
  font-size: 16px;
}
.buttons {
  display: flex;
  gap: 20px;
  margin-top: 15px;
  flex-wrap: wrap;
  justify-content: center;
}
.arm-button, .emergency-button {
  width: 180px;
  height: 65px;
  font-size: 20px;
  font-weight: bold;
  border: none;
  border-radius: 15px;
  color: white;
  box-shadow: 0 3px 8px rgba(0,0,0,0.2);
}
.arm-disarmed { background: #555555; }
.arm-armed { background: #168a3a; }
.emergency-button { background: #c62828; }
.emergency-button:active { background: #8e0000; }
</style>
</head>
<body>

<div class="container">

  <div class="slider-container">
    <input type="range" min="0" max="100" value="0" id="throttle">
  </div>

  <div class="current-throttle">
    Current: <span id="throttleValue">0%</span>
  </div>

  <div class="max-control">
    <label for="maxThrottle">Max %:</label>
    <input type="number" id="maxThrottle" min="1" max="100" value="100">
    <button onclick="setMaxThrottle()">SET</button>
  </div>

  <div class="telemetry">
    <div class="telemetry-title">Pixhawk Telemetry</div>
    <div class="telemetry-grid">
      <div class="telemetry-value">Roll: <span id="roll">0</span> °</div>
      <div class="telemetry-value">Pitch: <span id="pitch">0</span> °</div>
      <div class="telemetry-value">Yaw: <span id="yaw">0</span> °</div>
      <div></div>
      <div class="telemetry-value">Motor 1: <span id="motor1">0</span></div>
      <div class="telemetry-value">Motor 2: <span id="motor2">0</span></div>
      <div class="telemetry-value">Motor 3: <span id="motor3">0</span></div>
      <div class="telemetry-value">Motor 4: <span id="motor4">0</span></div>
    </div>
  </div>

  <div class="buttons">
    <button id="armButton" class="arm-button arm-disarmed" onclick="toggleArm()">DISARMED</button>
    <button class="emergency-button" onclick="emergencyStop()">EMERGENCY STOP</button>
  </div>

</div>

<script>
const slider = document.getElementById("throttle");
const maxInput = document.getElementById("maxThrottle");
let armed = false;

slider.addEventListener("pointerdown", function() {
  fetch("/touch?state=1");
});

slider.addEventListener("input", function() {
  document.getElementById("throttleValue").innerText = slider.value + "%";
  fetch("/throttle?value=" + slider.value);
});

slider.addEventListener("pointerup", function() {
  fetch("/touch?state=0");
});

slider.addEventListener("pointercancel", function() {
  fetch("/touch?state=0");
});

function updateStatus() {
  fetch("/status")
    .then(response => response.json())
    .then(data => {
      if (!slider.matches(":active")) {
        slider.value = data.throttle;
        document.getElementById("throttleValue").innerText = data.throttle + "%";
      }

      slider.max = data.maxThrottle;

      // Do not overwrite the number box while the user is editing it.
      if (document.activeElement !== maxInput) {
        maxInput.value = data.maxThrottle;
      }
    });
}

function updateTelemetry() {
  fetch("/telemetry")
    .then(response => response.json())
    .then(data => {
      document.getElementById("roll").innerText = data.roll.toFixed(2);
      document.getElementById("pitch").innerText = data.pitch.toFixed(2);
      document.getElementById("yaw").innerText = data.yaw.toFixed(2);
      document.getElementById("motor1").innerText = data.motor1 + " µs (" + data.motor1Percent + "%)";
      document.getElementById("motor2").innerText = data.motor2 + " µs (" + data.motor2Percent + "%)";
      document.getElementById("motor3").innerText = data.motor3 + " µs (" + data.motor3Percent + "%)";
      document.getElementById("motor4").innerText = data.motor4 + " µs (" + data.motor4Percent + "%)";
    })
    .catch(error => console.log("Telemetry error:", error));
}

function setMaxThrottle() {
  let value = parseInt(maxInput.value);
  if (isNaN(value)) value = 70;
  if (value < 1) value = 1;
  if (value > 100) value = 100;

  fetch("/max?value=" + value)
    .then(response => response.text())
    .then(result => {
      const maxValue = parseInt(result);
      slider.max = maxValue;
      maxInput.value = maxValue;

      if (parseInt(slider.value) > maxValue) {
        slider.value = maxValue;
      }

      document.getElementById("throttleValue").innerText = slider.value + "%";
      updateStatus();
    });
}

maxInput.addEventListener("change", setMaxThrottle);
maxInput.addEventListener("keydown", function(event) {
  if (event.key === "Enter") {
    setMaxThrottle();
  }
});

function toggleArm() {
  fetch("/arm")
    .then(() => {
      armed = !armed;
      const button = document.getElementById("armButton");

      if (armed) {
        button.innerHTML = "ARMED";
        button.classList.remove("arm-disarmed");
        button.classList.add("arm-armed");
      } else {
        button.innerHTML = "DISARMED";
        button.classList.remove("arm-armed");
        button.classList.add("arm-disarmed");
      }
    });
}

function emergencyStop() {
  fetch("/stop")
    .then(response => response.text())
    .then(() => {
      armed = false;
      document.getElementById("armButton").innerHTML = "DISARMED";
      document.getElementById("armButton").classList.remove("arm-armed");
      document.getElementById("armButton").classList.add("arm-disarmed");
      slider.value = 0;
      document.getElementById("throttleValue").innerText = "0%";
    })
    .catch(error => console.log("Emergency stop error:", error));
}

setInterval(updateStatus, 200);
setInterval(updateTelemetry, 200);
updateStatus();
updateTelemetry();
</script>

</body>
</html>
)rawliteral";


// =====================================================
// EMERGENCY PAGE
// =====================================================

const char emergencyPage[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<style>
html, body { margin:0; padding:0; width:100%; height:100%; background:#f5f5f5; font-family:Arial,sans-serif; }
.container { width:100%; height:100%; display:flex; flex-direction:column; align-items:center; justify-content:center; }
button { width:85%; max-width:500px; height:180px; font-size:38px; font-weight:bold; border:none; border-radius:25px; color:white; background:#c62828; }
button:active { background:#8e0000; }
.status { margin-top:25px; font-size:22px; }
a { margin-top:25px; font-size:18px; }
</style>
</head>
<body>
<div class="container">
  <button onclick="emergencyStop()">EMERGENCY STOP</button>
  <div id="status" class="status">Ready</div>
  <a href="/">Back to Control Page</a>
</div>
<script>
function emergencyStop() {
  fetch("/stop")
    .then(response => response.text())
    .then(value => document.getElementById("status").innerText = value)
    .catch(() => document.getElementById("status").innerText = "Request failed");
}
</script>
</body>
</html>
)rawliteral";


// =====================================================
// FLIGHT LOG PAGE
// =====================================================

const char logPage[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<style>
html, body { margin:0; padding:0; width:100%; min-height:100%; background:#f5f5f5; font-family:Arial,sans-serif; }
.container { max-width:900px; margin:0 auto; padding:25px 15px; }
h1 { font-size:28px; text-align:center; }
.card { background:white; border-radius:15px; padding:20px; margin-top:20px; box-shadow:0 3px 8px rgba(0,0,0,0.15); }
.info { margin:8px 0; font-size:18px; }
.buttons { text-align:center; margin-top:15px; }
button, a.button { display:inline-block; padding:14px 20px; margin:8px; border:none; border-radius:10px; font-size:17px; text-decoration:none; color:white; background:#333333; cursor:pointer; }
.clear { background:#c62828; }
.table-wrap { overflow-x:auto; margin-top:15px; }
table { width:100%; border-collapse:collapse; min-width:950px; }
th, td { border:1px solid #ddd; padding:6px; font-size:13px; text-align:center; white-space:nowrap; }
th { background:#eeeeee; }
.status { font-weight:bold; }
.note { font-size:14px; color:#555; margin-top:10px; }
a.back { display:block; text-align:center; margin-top:20px; }
</style>
</head>
<body>
<div class="container">
  <h1>Flight Data Logger</h1>

  <div class="card">
    <div class="info">Logger: <span id="status" class="status">STARTING</span></div>
    <div class="info">Records in browser memory: <span id="records">0</span></div>
    <div class="info">Current Roll: <span id="liveRoll">0</span> °</div>
    <div class="info">Current Pitch: <span id="livePitch">0</span> °</div>
    <div class="info">Current Yaw: <span id="liveYaw">0</span> °</div>
    <div class="info">Battery Voltage: <span id="liveBatteryVoltage">0.00</span> V</div>
    <div class="info">Battery Current: <span id="liveBatteryCurrent">0.00</span> A</div>
    <div class="info">Battery Remaining: <span id="liveBatteryRemaining">--</span> %</div>
    <div class="buttons">
      <button onclick="downloadCSV()">DOWNLOAD CSV</button>
      <button class="clear" onclick="clearMemory()">CLEAR MEMORY</button>
    </div>
    <div class="note">Data is kept only in this webpage's browser memory. Closing or refreshing this page clears the current log. The table shows the latest 100 records; the CSV download contains all records collected by this page.</div>
  </div>

  <div class="card table-wrap">
    <table>
      <thead>
        <tr>
          <th>Time (ms)</th>
          <th>Roll (°)</th>
          <th>Pitch (°)</th>
          <th>Yaw (°)</th>
          <th>Battery (V)</th>
          <th>Current (A)</th>
          <th>Battery (%)</th>
          <th>M1 (µs)</th>
          <th>M2 (µs)</th>
          <th>M3 (µs)</th>
          <th>M4 (µs)</th>
          <th>M1 (%)</th>
          <th>M2 (%)</th>
          <th>M3 (%)</th>
          <th>M4 (%)</th>
          <th>Throttle (%)</th>
          <th>Max (%)</th>
          <th>Armed</th>
        </tr>
      </thead>
      <tbody id="logBody"></tbody>
    </table>
  </div>

  <a class="back" href="/">Back to Control Page</a>
</div>

<script>
const flightLog = [];
let lastDisplayedRows = 0;

function csvEscape(value) {
  const text = String(value);
  if (text.includes(',') || text.includes('"') || text.includes('\n')) {
    return '"' + text.replaceAll('"', '""') + '"';
  }
  return text;
}

function addRowToTable(data) {
  const row = document.createElement('tr');
  const values = [
    data.time, data.roll.toFixed(2), data.pitch.toFixed(2), data.yaw.toFixed(2),
    data.batteryVoltage.toFixed(2), data.batteryCurrent.toFixed(2),
    data.batteryRemaining < 0 ? "--" : data.batteryRemaining,
    data.motor1, data.motor2, data.motor3, data.motor4,
    data.motor1Percent, data.motor2Percent, data.motor3Percent, data.motor4Percent,
    data.throttle, data.maxThrottle, data.armed ? 1 : 0
  ];

  values.forEach(value => {
    const cell = document.createElement('td');
    cell.innerText = value;
    row.appendChild(cell);
  });

  const body = document.getElementById('logBody');
  body.appendChild(row);

  // Keep the full log in browser memory, but display only the latest 100 rows.
  while (body.rows.length > 100) {
    body.deleteRow(0);
  }
}

function updateLive(data) {
  document.getElementById('liveRoll').innerText = data.roll.toFixed(2);
  document.getElementById('livePitch').innerText = data.pitch.toFixed(2);
  document.getElementById('liveYaw').innerText = data.yaw.toFixed(2);
  document.getElementById('liveBatteryVoltage').innerText = data.batteryVoltage.toFixed(2);
  document.getElementById('liveBatteryCurrent').innerText = data.batteryCurrent.toFixed(2);
  document.getElementById('liveBatteryRemaining').innerText = data.batteryRemaining < 0 ? '--' : data.batteryRemaining;
}

function pollFlightData() {
  fetch('/flightdata', { cache: 'no-store' })
    .then(response => response.json())
    .then(data => {
      updateLive(data);
      flightLog.push(data);
      addRowToTable(data);
      document.getElementById('records').innerText = flightLog.length;
      document.getElementById('status').innerText = 'RECORDING';
    })
    .catch(() => {
      document.getElementById('status').innerText = 'CONNECTION LOST';
    });
}

function downloadCSV() {
  if (flightLog.length === 0) {
    alert('No flight data has been recorded yet.');
    return;
  }

  const header = [
    'time_ms','roll_deg','pitch_deg','yaw_deg',
    'battery_voltage_v','battery_current_a','battery_remaining_percent',
    'motor1_pwm','motor2_pwm','motor3_pwm','motor4_pwm',
    'motor1_percent','motor2_percent','motor3_percent','motor4_percent',
    'throttle_command_percent','max_throttle_percent','pixhawk_armed'
  ];

  const lines = [header.join(',')];

  flightLog.forEach(data => {
    lines.push([
      data.time, data.roll, data.pitch, data.yaw,
      data.batteryVoltage, data.batteryCurrent,
      data.batteryRemaining < 0 ? '' : data.batteryRemaining,
      data.motor1, data.motor2, data.motor3, data.motor4,
      data.motor1Percent, data.motor2Percent, data.motor3Percent, data.motor4Percent,
      data.throttle, data.maxThrottle, data.armed ? 1 : 0
    ].map(csvEscape).join(','));
  });

  const blob = new Blob([lines.join('\n')], { type: 'text/csv' });
  const url = URL.createObjectURL(blob);
  const link = document.createElement('a');
  link.href = url;
  link.download = 'flightlog.csv';
  link.click();
  URL.revokeObjectURL(url);
}

function clearMemory() {
  if (!confirm('Clear the flight data currently stored in this webpage?')) return;
  flightLog.length = 0;
  document.getElementById('logBody').innerHTML = '';
  document.getElementById('records').innerText = '0';
}

pollFlightData();
setInterval(pollFlightData, 200);
</script>
</body>
</html>
)rawliteral";


// =====================================================
// MAIN PAGE
// =====================================================

void handleRoot()
{

  server.send(
    200,
    "text/html",
    controlPage
  );

}


// =====================================================
// EMERGENCY PAGE
// =====================================================

void handleEmergencyPage()
{
  server.send(
    200,
    "text/html",
    emergencyPage
  );
}


// =====================================================
// FLIGHT LOG PAGE
// =====================================================

void handleLogPage()
{
  server.send(
    200,
    "text/html",
    logPage
  );
}


// =====================================================
// THROTTLE
// =====================================================

void handleThrottle()
{

  if (
    server.hasArg("value")
  )
  {

    throttle =
      server.arg(
        "value"
      ).toInt();


    throttle =
      constrain(
        throttle,
        0,
        maxThrottle
      );


    lastTouchTime =
      millis();


    lastDecreaseTime =
      millis();


    emergencyStop =
      false;


    Serial.print(
      "Throttle: "
    );

    Serial.print(
      throttle
    );

    Serial.println(
      "%"
    );


    // Immediately send new value

    if (armed)
    {

      sendThrottleToPixhawk(
        throttle
      );

    }

  }


  server.send(
    200,
    "text/plain",
    "OK"
  );

}


// =====================================================
// MAXIMUM THROTTLE
// =====================================================

void handleMaxThrottle()
{
  if (server.hasArg("value"))
  {
    int requestedMax =
      server.arg("value").toInt();

    maxThrottle =
      constrain(requestedMax, 1, 100);

    if (throttle > maxThrottle)
    {
      throttle = maxThrottle;

      if (armed)
      {
        sendThrottleToPixhawk(throttle);
      }
    }

    Serial.print("Maximum throttle: ");
    Serial.print(maxThrottle);
    Serial.println("%");
  }

  lastWebActivity = millis();

  server.send(
    200,
    "text/plain",
    String(maxThrottle)
  );
}


// =====================================================
// TOUCH STATE
// =====================================================

void handleTouch()
{

  if (
    server.hasArg("state")
  )
  {

    int state =
      server.arg(
        "state"
      ).toInt();


    if (
      state == 1
    )
    {

      sliderTouched =
        true;


      lastTouchTime =
        millis();


      Serial.println(
        "Slider: TOUCHING"
      );

    }

    else
    {

      sliderTouched =
        false;


      lastTouchTime =
        millis();


      lastDecreaseTime =
        millis();


      Serial.println(
        "Slider: RELEASED"
      );

    }

  }


  server.send(
    200,
    "text/plain",
    "OK"
  );

}


// =====================================================
// STATUS
// =====================================================

void handleStatus()
{

  lastWebActivity = millis();

  String json = "{";
  json += "\"throttle\":" + String(throttle) + ",";
  json += "\"maxThrottle\":" + String(maxThrottle) + ",";
  json += "\"pixhawkArmed\":" + String(pixhawkArmed ? "true" : "false");
  json += "}";

  server.send(
    200,
    "application/json",
    json
  );

}


// =====================================================
// FLIGHT LOGGING
// =====================================================

// =====================================================
// TELEMETRY
// =====================================================

void handleTelemetry()
{

  lastWebActivity = millis();

  String json = "{";

  json +=
    "\"roll\":" +
    String(roll, 2) +
    ",";

  json +=
    "\"pitch\":" +
    String(pitch, 2) +
    ",";

  json +=
    "\"yaw\":" +
    String(yaw, 2) +
    ",";

  json +=
    "\"motor1\":" +
    String(motor1PWM) +
    ",";

  json +=
    "\"motor2\":" +
    String(motor2PWM) +
    ",";

  json +=
    "\"motor3\":" +
    String(motor3PWM) +
    ",";

  json +=
    "\"motor4\":" +
    String(motor4PWM) +
    ",";

  json +=
    "\"motor1Percent\":" +
    String(pwmToPercent(motor1PWM)) +
    ",";

  json +=
    "\"motor2Percent\":" +
    String(pwmToPercent(motor2PWM)) +
    ",";

  json +=
    "\"motor3Percent\":" +
    String(pwmToPercent(motor3PWM)) +
    ",";

  json +=
    "\"motor4Percent\":" +
    String(pwmToPercent(motor4PWM));

  json += "}";


  server.send(
    200,
    "application/json",
    json
  );

}




// =====================================================
// LIVE FLIGHT DATA FOR LOGGER PAGE
// =====================================================

void handleFlightData()
{
  String json = "{";

  json += "\"time\":" + String(millis()) + ",";
  json += "\"roll\":" + String(roll, 3) + ",";
  json += "\"pitch\":" + String(pitch, 3) + ",";
  json += "\"yaw\":" + String(yaw, 3) + ",";
  json += "\"batteryVoltage\":" + String(batteryVoltage, 3) + ",";
  json += "\"batteryCurrent\":" + String(batteryCurrent, 3) + ",";
  json += "\"batteryRemaining\":" + String(batteryRemaining) + ",";
  json += "\"motor1\":" + String(motor1PWM) + ",";
  json += "\"motor2\":" + String(motor2PWM) + ",";
  json += "\"motor3\":" + String(motor3PWM) + ",";
  json += "\"motor4\":" + String(motor4PWM) + ",";
  json += "\"motor1Percent\":" + String(pwmToPercent(motor1PWM)) + ",";
  json += "\"motor2Percent\":" + String(pwmToPercent(motor2PWM)) + ",";
  json += "\"motor3Percent\":" + String(pwmToPercent(motor3PWM)) + ",";
  json += "\"motor4Percent\":" + String(pwmToPercent(motor4PWM)) + ",";
  json += "\"throttle\":" + String(throttle) + ",";
  json += "\"maxThrottle\":" + String(maxThrottle) + ",";
  json += "\"armed\":" + String(pixhawkArmed ? "true" : "false");

  json += "}";

  server.send(
    200,
    "application/json",
    json
  );
}
// =====================================================
// ARM / DISARM
// =====================================================

void handleArm()
{

  if (!armed)
  {

    throttle =
      0;


    armPixhawk();


    armed =
      true;


    Serial.println(
      "ARM: ON"
    );

  }

  else
  {

    throttle =
      0;


    sendThrottleToPixhawk(
      0
    );


    disarmPixhawk();


    armed =
      false;


    Serial.println(
      "ARM: OFF"
    );

  }


  server.send(
    200,
    "text/plain",
    "OK"
  );

}


// =====================================================
// EMERGENCY STOP
// =====================================================

void handleStop()
{

  emergencyStop =
    true;


  throttle =
    0;


  armed =
    false;


  sliderTouched =
    false;


  sendThrottleToPixhawk(
    0
  );


  disarmPixhawk();


  Serial.println(
    "!!! EMERGENCY STOP !!!"
  );


  server.send(
    200,
    "text/plain",
    "STOPPED"
  );

}


// =====================================================
// SETUP
// =====================================================

void setup()
{

  Serial.begin(
    115200
  );


  // ===================================================
  // PIXHAWK TELEM2
  // ===================================================

  Serial2.begin(
    57600,
    SERIAL_8N1,
    RXD2,
    TXD2
  );


  // ===================================================
  // WIFI
  // ===================================================

  WiFi.softAP(
    ssid,
    password
  );


  Serial.println();

  Serial.println(
    "Flying Table Wi-Fi started"
  );


  Serial.print(
    "IP address: "
  );

  Serial.println(
    WiFi.softAPIP()
  );


  // ===================================================
  // WEB SERVER ROUTES
  // ===================================================

  server.on(
    "/",
    handleRoot
  );


  server.on(
    "/throttle",
    handleThrottle
  );


  server.on(
    "/touch",
    handleTouch
  );


  server.on(
    "/status",
    handleStatus
  );


  server.on(
    "/telemetry",
    handleTelemetry
  );


  server.on(
    "/flightdata",
    handleFlightData
  );


  server.on(
    "/max",
    handleMaxThrottle
  );


  server.on(
    "/emergency",
    handleEmergencyPage
  );


  server.on(
    "/log",
    handleLogPage
  );


  server.on(
    "/arm",
    handleArm
  );


  server.on(
    "/stop",
    handleStop
  );


  server.begin();


  Serial.println(
    "Web server started"
  );

}


// =====================================================
// MAIN LOOP
// =====================================================

void loop()
{

  // Handle web requests

  server.handleClient();


  // ===================================================
  // READ PIXHAWK TELEMETRY
  // ===================================================

  readMAVLink();


  // ===================================================
  // SEND HEARTBEAT
  // ===================================================

  bool wifiControllerConnected =
    WiFi.softAPgetStationNum() > 0 &&
    millis() - lastWebActivity < WEB_TIMEOUT;

  if (wifiControllerConnected)
  {
    if (millis() - lastHeartbeat >= 1000)
    {
      sendHeartbeat();
      lastHeartbeat = millis();
    }
  }


  // ===================================================
  // SEND THROTTLE
  // ===================================================

  if (
    armed &&
    !emergencyStop &&
    WiFi.softAPgetStationNum() > 0 &&
    millis() - lastWebActivity < WEB_TIMEOUT
  )
  {

    if (
      millis() -
      lastMAVLinkSend >=
      50
    )
    {

      sendThrottleToPixhawk(
        throttle
      );


      lastMAVLinkSend =
        millis();

    }

  }


  // ===================================================
  // AUTOMATIC THROTTLE DECREASE
  // ===================================================

  if (

    throttle > 0 &&

    !sliderTouched &&

    !emergencyStop

  )
  {

    unsigned long now =
      millis();


    if (

      now -
      lastTouchTime >=
      500

    )
    {

      if (

        now -
        lastDecreaseTime >=
        500

      )
      {

        throttle--;


        lastDecreaseTime =
          now;


        Serial.print(
          "Automatic throttle: "
        );

        Serial.print(
          throttle
        );

        Serial.println(
          "%"
        );

      }

    }

  }


  yield();

}