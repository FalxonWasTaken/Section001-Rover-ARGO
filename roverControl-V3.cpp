// ============================================================
//  Mars Rover - ESP32 WiFi Control  (BLHeli_32 ESC edition)
//
//  1. Power on the ESP32
//  2. Connect to WiFi:  MarsRover  (password below)
//  3. Open browser:     192.168.4.1
//
//  ESC signal scheme (center-point):
//    1000 µs = full reverse
//    1500 µs = neutral / stop
//    2000 µs = full forward
// ============================================================

#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>

// ============================================================
//  WIFI CONFIGURATION
// ============================================================
const char* AP_SSID     = "MarsRover";
const char* AP_PASSWORD = "rover12345";

// ============================================================
//  ESC PIN CONFIGURATION
// ============================================================
const int ESC1_PIN = 33;   // Left motor
const int ESC2_PIN = 32;   // Right motor

// ============================================================
//  ESC SIGNAL RANGE  (µs)
// ============================================================
const int ESC_MIN     = 1000;   // full reverse
const int ESC_NEUTRAL = 1500;   // stop / armed idle
const int ESC_MAX     = 2000;   // full forward

// ============================================================
//  THROTTLE SETTINGS
//
//  DEADBAND    Minimum µs offset from neutral before the motors
//              will actually start spinning.  Tune this upward
//              if the motors hum but don't move at low throttle.
//
//  driveOffset How many µs above/below neutral the rover cruises
//              at.  Controlled live by the web slider (100–500).
//              e.g. 350 → forward signal = 1850 µs
//                        reverse signal = 1150 µs
// ============================================================
const int DEADBAND  = 100;   // tune me if motors don't spin at low throttle
int driveOffset     = 350;   // cruise offset – updated live by the slider

// ============================================================
//  RAMP CONFIGURATION
//
//  RAMP_STEP      µs added/removed per tick
//  RAMP_INTERVAL  ms between each ramp tick
//
//  Approx ramp time = ((driveOffset - DEADBAND) / RAMP_STEP) * RAMP_INTERVAL ms
//  Default: ((350 - 100) / 10) * 15 ≈ 375 ms
// ============================================================
const int RAMP_STEP     = 10;   // tune me
const int RAMP_INTERVAL = 15;   // tune me  (ms)

// ============================================================
//  STATE MACHINE
//
//  IDLE ──fwd/rev──► RAMPING_UP ──reaches target──► RUNNING
//   ▲                    │                              │
//   │                    └──stop/reverse────────────────┘
//   │                                                   │
//   └──────────────── RAMPING_DOWN ◄────────────────────┘
//                          │
//                   reaches neutral
//                    /          \
//             no pending      pending dir queued
//              dir queued      → RAMPING_UP new dir
//                │
//               IDLE
//
//  Spins bypass the ramp entirely (see cmdSpinLeft/Right).
//  isSpinning flag prevents updateRamp() from interfering.
// ============================================================
enum RoverState { IDLE, RAMPING_UP, RUNNING, RAMPING_DOWN };
enum Direction  { DIR_NONE, DIR_FORWARD, DIR_BACKWARD };

RoverState roverState    = IDLE;
Direction  currentDir    = DIR_NONE;
Direction  pendingDir    = DIR_NONE;
bool       isSpinning    = false;
int        currentMicros = ESC_NEUTRAL;

unsigned long lastRampMillis = 0;

Servo esc1, esc2;
WebServer server(80);

// ============================================================
//  HTML INTERFACE
// ============================================================
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Mars Rover Control</title>
  <style>
    * { box-sizing: border-box; margin: 0; padding: 0; }
    body {
      background: #0d0d0d;
      color: #e8e8e8;
      font-family: 'Segoe UI', Arial, sans-serif;
      display: flex;
      flex-direction: column;
      align-items: center;
      min-height: 100vh;
      padding: 24px 16px;
      gap: 28px;
    }
    header { text-align: center; }
    header h1 {
      font-size: 2rem;
      letter-spacing: 3px;
      color: #e05a1b;
      text-transform: uppercase;
    }
    header p { font-size: 0.85rem; color: #888; margin-top: 4px; }

    #status {
      padding: 6px 20px;
      border-radius: 999px;
      font-size: 0.85rem;
      font-weight: 600;
      letter-spacing: 1px;
      background: #1a1a1a;
      border: 1px solid #333;
      transition: background 0.3s, color 0.3s;
    }
    #status.ok  { background: #0f2d1a; color: #4caf50; border-color: #4caf50; }
    #status.err { background: #2d0f0f; color: #f44336; border-color: #f44336; }

    .dpad {
      display: grid;
      grid-template-columns: repeat(3, 110px);
      grid-template-rows:    repeat(3, 110px);
      gap: 10px;
    }
    .btn {
      border: none;
      border-radius: 14px;
      font-size: 1rem;
      font-weight: 700;
      letter-spacing: 1px;
      text-transform: uppercase;
      cursor: pointer;
      transition: filter 0.15s, transform 0.1s;
      user-select: none;
      display: flex;
      flex-direction: column;
      align-items: center;
      justify-content: center;
      gap: 6px;
      color: #fff;
      -webkit-tap-highlight-color: transparent;
    }
    .btn .icon { font-size: 2rem; line-height: 1; }
    .btn:active { transform: scale(0.93); filter: brightness(0.8); }
    .btn-dir {
      background: linear-gradient(145deg, #1e3a5f, #0e2040);
      box-shadow: 0 4px 12px rgba(0,0,0,0.5);
    }
    .btn-stop {
      background: linear-gradient(145deg, #7a1a1a, #4a0f0f);
      box-shadow: 0 4px 12px rgba(0,0,0,0.5);
    }
    .btn-fwd  { grid-column: 2; grid-row: 1; }
    .btn-stop { grid-column: 2; grid-row: 2; }
    .btn-bwd  { grid-column: 2; grid-row: 3; }

    .speed-section {
      width: 100%;
      max-width: 360px;
      background: #1a1a1a;
      border-radius: 14px;
      padding: 18px 22px;
      border: 1px solid #2a2a2a;
    }
    .speed-section label {
      display: flex;
      justify-content: space-between;
      font-size: 0.85rem;
      color: #aaa;
      margin-bottom: 10px;
    }
    .speed-section label span { color: #e05a1b; font-weight: 700; font-size: 1rem; }
    .speed-section .sublabel {
      font-size: 0.75rem;
      color: #555;
      margin-top: 8px;
      text-align: right;
    }
    input[type=range] {
      width: 100%;
      accent-color: #e05a1b;
      height: 6px;
      cursor: pointer;
    }

    .extras {
      display: flex;
      flex-wrap: wrap;
      gap: 10px;
      justify-content: center;
      max-width: 360px;
      width: 100%;
    }
    .btn-extra {
      flex: 1 1 150px;
      height: 64px;
      border-radius: 12px;
      background: linear-gradient(145deg, #2a1e3f, #180f28);
      box-shadow: 0 4px 12px rgba(0,0,0,0.5);
      font-size: 0.85rem;
    }
  </style>
</head>
<body>

  <header>
    <h1>&#127756; Mars Rover</h1>
    <p>ESP32 Wireless Control</p>
  </header>

  <div id="status">READY</div>

  <div class="dpad">
    <button class="btn btn-dir btn-fwd" onclick="send('forward')">
      <span class="icon">&#9650;</span> FWD
    </button>
    <button class="btn btn-stop" onclick="send('stop')">
      <span class="icon">&#9632;</span> STOP
    </button>
    <button class="btn btn-dir btn-bwd" onclick="send('backward')">
      <span class="icon">&#9660;</span> REV
    </button>
  </div>

  <!-- Slider controls driveOffset: 100 (deadband) to 500 (near full throttle) -->
  <div class="speed-section">
    <label>
      THROTTLE
      <span id="speedVal">350</span>
    </label>
    <input type="range" id="speedSlider"
           min="100" max="500" value="350"
           oninput="updateSpeed(this.value)">
    <p class="sublabel">Signal offset from neutral (µs)</p>
  </div>

  <div class="extras">
    <button class="btn btn-extra" onclick="send('spin_left')">
      <span class="icon">&#8634;</span> SPIN L
    </button>
    <button class="btn btn-extra" onclick="send('spin_right')">
      <span class="icon">&#8635;</span> SPIN R
    </button>
    <!--
    <button class="btn btn-extra" onclick="send('your_action')">
      <span class="icon">&#128270;</span> LABEL
    </button>
    -->
  </div>

  <script>
    function send(action) {
      const status = document.getElementById('status');
      status.className = '';
      status.textContent = action.toUpperCase() + '...';
      fetch('/cmd?action=' + action)
        .then(r => r.text())
        .then(t => {
          status.className = 'ok';
          status.textContent = t;
          setTimeout(() => { status.className = ''; status.textContent = 'READY'; }, 2000);
        })
        .catch(() => { status.className = 'err'; status.textContent = 'NO RESPONSE'; });
    }
    let speedTimer;
    function updateSpeed(val) {
      document.getElementById('speedVal').textContent = val;
      clearTimeout(speedTimer);
      speedTimer = setTimeout(() => send('speed_' + val), 150);
    }
  </script>
</body>
</html>
)rawliteral";


// ============================================================
//  LOW-LEVEL ESC PRIMITIVES
// ============================================================

// Write the same signal to both ESCs (used for forward/reverse/stop)
void applyBothESCs(int micros) {
  micros = constrain(micros, ESC_MIN, ESC_MAX);
  esc1.writeMicroseconds(micros);
  esc2.writeMicroseconds(micros);
  currentMicros = micros;
}

// Immediately return both ESCs to neutral and reset all state.
// Used at boot and when stopping a spin.
void hardStop() {
  esc1.writeMicroseconds(ESC_NEUTRAL);
  esc2.writeMicroseconds(ESC_NEUTRAL);
  currentMicros = ESC_NEUTRAL;
  currentDir    = DIR_NONE;
  pendingDir    = DIR_NONE;
  isSpinning    = false;
  roverState    = IDLE;
}


// ============================================================
//  RAMP ENGINE  –  called every loop(), never blocks
//
//  Moves currentMicros one RAMP_STEP toward its target each
//  RAMP_INTERVAL ms.  Spins are excluded via isSpinning flag.
// ============================================================
void updateRamp() {
  if (isSpinning) return;

  unsigned long now = millis();
  if (now - lastRampMillis < (unsigned long)RAMP_INTERVAL) return;
  lastRampMillis = now;

  switch (roverState) {

    // ── Accelerating toward cruise ────────────────────────────
    case RAMPING_UP: {
      int target = (currentDir == DIR_FORWARD)
                   ? ESC_NEUTRAL + driveOffset
                   : ESC_NEUTRAL - driveOffset;

      if (currentDir == DIR_FORWARD) {
        currentMicros += RAMP_STEP;
        if (currentMicros >= target) { currentMicros = target; roverState = RUNNING; }
      } else {
        currentMicros -= RAMP_STEP;
        if (currentMicros <= target) { currentMicros = target; roverState = RUNNING; }
      }
      applyBothESCs(currentMicros);
      break;
    }

    // ── Decelerating toward neutral ───────────────────────────
    case RAMPING_DOWN:
      if      (currentMicros > ESC_NEUTRAL) currentMicros -= RAMP_STEP;
      else if (currentMicros < ESC_NEUTRAL) currentMicros += RAMP_STEP;

      // Clamp to neutral if we've crossed it
      if (abs(currentMicros - ESC_NEUTRAL) < RAMP_STEP) currentMicros = ESC_NEUTRAL;

      applyBothESCs(currentMicros);

      if (currentMicros == ESC_NEUTRAL) {
        if (pendingDir != DIR_NONE) {
          // Direction change was queued – kick off the new direction from deadband
          currentDir    = pendingDir;
          pendingDir    = DIR_NONE;
          currentMicros = (currentDir == DIR_FORWARD)
                          ? ESC_NEUTRAL + DEADBAND
                          : ESC_NEUTRAL - DEADBAND;
          applyBothESCs(currentMicros);
          roverState = RAMPING_UP;
        } else {
          // Plain stop
          currentDir = DIR_NONE;
          roverState = IDLE;
        }
      }
      break;

    // ── Cruising – tracks live slider changes ─────────────────
    case RUNNING: {
      int target = (currentDir == DIR_FORWARD)
                   ? ESC_NEUTRAL + driveOffset
                   : ESC_NEUTRAL - driveOffset;
      if (currentMicros != target) {
        currentMicros = target;
        applyBothESCs(currentMicros);
      }
      break;
    }

    case IDLE:
    default:
      break;
  }
}


// ============================================================
//  COMMAND HANDLERS
// ============================================================

void cmdMove(Direction dir) {
  // If a spin was active, bring ESCs to neutral before proceeding
  if (isSpinning) {
    esc1.writeMicroseconds(ESC_NEUTRAL);
    esc2.writeMicroseconds(ESC_NEUTRAL);
    isSpinning    = false;
    currentMicros = ESC_NEUTRAL;
    currentDir    = DIR_NONE;
    roverState    = IDLE;
  }

  if (dir == currentDir) {
    // Same direction tapped again while slowing – reverse the ramp back up
    if (roverState == RAMPING_DOWN) {
      pendingDir = DIR_NONE;
      roverState = RAMPING_UP;
    }
    return;
  }

  if (roverState == IDLE || currentMicros == ESC_NEUTRAL) {
    // Standing start – begin from deadband to avoid the motor hum zone
    currentDir    = dir;
    currentMicros = (dir == DIR_FORWARD)
                    ? ESC_NEUTRAL + DEADBAND
                    : ESC_NEUTRAL - DEADBAND;
    applyBothESCs(currentMicros);
    roverState = RAMPING_UP;
  } else {
    // Moving the other way – queue the new direction and ramp down first
    pendingDir = dir;
    roverState = RAMPING_DOWN;
  }
}

void cmdStop() {
  if (isSpinning) {
    // Spins don't have momentum worth ramping, cut to neutral immediately
    hardStop();
    return;
  }
  pendingDir = DIR_NONE;
  if (roverState == IDLE) return;
  roverState = RAMPING_DOWN;
}

// Spins send opposite signals to each ESC – no ramp needed since the
// forces cancel across the chassis and traction isn't a concern
void cmdSpinLeft() {
  isSpinning = true;
  currentDir = DIR_NONE;
  pendingDir = DIR_NONE;
  roverState = RUNNING;   // lets cmdStop() know we're active
  esc1.writeMicroseconds(ESC_NEUTRAL - driveOffset);   // left  → reverse
  esc2.writeMicroseconds(ESC_NEUTRAL + driveOffset);   // right → forward
}

void cmdSpinRight() {
  isSpinning = true;
  currentDir = DIR_NONE;
  pendingDir = DIR_NONE;
  roverState = RUNNING;
  esc1.writeMicroseconds(ESC_NEUTRAL + driveOffset);   // left  → forward
  esc2.writeMicroseconds(ESC_NEUTRAL - driveOffset);   // right → reverse
}


// ============================================================
//  WEB HANDLERS
// ============================================================

void handleRoot() {
  server.send_P(200, "text/html", INDEX_HTML);
}

void handleCmd() {
  if (!server.hasArg("action")) {
    server.send(400, "text/plain", "MISSING ACTION");
    return;
  }

  String action   = server.arg("action");
  String response = "OK";

  if      (action == "forward")    { cmdMove(DIR_FORWARD);  response = "FORWARD";  }
  else if (action == "backward")   { cmdMove(DIR_BACKWARD); response = "REVERSE";  }
  else if (action == "stop")       { cmdStop();              response = "STOPPING"; }
  else if (action == "spin_left")  { cmdSpinLeft();          response = "SPIN L";   }
  else if (action == "spin_right") { cmdSpinRight();         response = "SPIN R";   }
  else if (action.startsWith("speed_")) {
    int val    = action.substring(6).toInt();
    val        = constrain(val, DEADBAND, 500);
    driveOffset = val;
    // If cruising, apply the new target immediately without waiting for next ramp tick
    if (roverState == RUNNING && !isSpinning) {
      int target = (currentDir == DIR_FORWARD)
                   ? ESC_NEUTRAL + driveOffset
                   : ESC_NEUTRAL - driveOffset;
      applyBothESCs(target);
    }
    response = "THROTTLE " + String(val);
  }
  else { response = "UNKNOWN"; }

  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "text/plain", response);
}

void handleNotFound() {
  server.send(404, "text/plain", "NOT FOUND");
}


// ============================================================
//  SETUP
// ============================================================

void setup() {
  Serial.begin(115200);

  // Use timers 2 & 3 – timers 0 & 1 are reserved by the ESP32 WiFi stack
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  esc1.attach(ESC1_PIN, ESC_MIN, ESC_MAX);
  esc2.attach(ESC2_PIN, ESC_MIN, ESC_MAX);

  hardStop();
  delay(2000);

  Serial.println("\nESCs armed.");
  Serial.println("Starting WiFi Access Point...");

  WiFi.softAP(AP_SSID, AP_PASSWORD);

  IPAddress ip = WiFi.softAPIP();
  Serial.printf("SSID     : %s\n", AP_SSID);
  Serial.printf("Password : %s\n", AP_PASSWORD);
  Serial.printf("URL      : http://%s\n", ip.toString().c_str());

  server.on("/",    handleRoot);
  server.on("/cmd", handleCmd);
  server.onNotFound(handleNotFound);

  server.begin();
  Serial.println("Web server started.");
}

// ============================================================
//  LOOP
// ============================================================

void loop() {
  server.handleClient();
  updateRamp();
}
