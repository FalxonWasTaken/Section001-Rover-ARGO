// ============================================================
//  Mars Rover - ESP32 WiFi Control
//  BLHeli_32 ESC edition  |  Non-blocking ramp  |  Auto-paths
//
//  1. Power on the ESP32
//  2. Connect to WiFi:  MarsRover
//  3. Open browser:     192.168.4.1
//
//  ESC signal scheme:
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
const char* AP_PASSWORD = "rover1234";

// ============================================================
//  ESC PIN CONFIGURATION
// ============================================================
const int ESC1_PIN = 33;   // Left motor
const int ESC2_PIN = 32;   // Right motor

// ============================================================
//  ESC SIGNAL RANGE (µs)
// ============================================================
const int ESC_MIN     = 1000;
const int ESC_NEUTRAL = 1500;
const int ESC_MAX     = 2000;

// ============================================================
//  THROTTLE SETTINGS
// ============================================================
const int DEADBAND  = 80;
int driveOffset = 350;

// ============================================================
//  RAMP CONFIGURATION
// ============================================================
const int RAMP_STEP     = 10;
const int RAMP_INTERVAL = 15;

// ============================================================
//  DRIVE STATE MACHINE
// ============================================================
enum RoverState { IDLE, RAMPING_UP, RUNNING, RAMPING_DOWN };
enum Direction  { DIR_NONE, DIR_FORWARD, DIR_BACKWARD };

RoverState    roverState     = IDLE;
Direction     currentDir     = DIR_NONE;
Direction     pendingDir     = DIR_NONE;
bool          isSpinning     = false;
int           currentMicros  = ESC_NEUTRAL;
unsigned long lastRampMillis = 0;

Servo     esc1, esc2;
WebServer server(80);


// ============================================================
//  ████████████████████████████████████████████████████████
//  ██                                                    ██
//  ██              AUTONOMOUS ROUTE BOOK                 ██
//  ██                                                    ██
//  ██  This is the only section you need to edit to      ██
//  ██  add, remove, or tune autonomous paths.            ██
//  ██                                                    ██
//  ████████████████████████████████████████████████████████
//
//  HOW TO DEFINE A PATH
//  ─────────────────────────────────────────────────────────
//  Each path is an array of PathStep entries:
//
//    { ACTION, DURATION_MS, SPEED_OFFSET }
//
//  ACTION        What the rover does for this step:
//    ACT_FORWARD     Drive forward
//    ACT_BACKWARD    Drive backward
//    ACT_SPIN_LEFT   Pivot left  (both wheels opposing)
//    ACT_SPIN_RIGHT  Pivot right (both wheels opposing)
//    ACT_STOP        Hold still  (motors at neutral)
//
//  DURATION_MS   How long to hold this action in milliseconds
//                1000 = 1 second
//
//  SPEED_OFFSET  ESC µs offset from neutral for this step.
//                Same as the web slider:  100 (slow) – 500 (fast)
//                Overrides driveOffset just for this step.
//                Use 0 to inherit the current slider value.
//
//  EXAMPLE STEP:
//    { ACT_FORWARD, 2000, 300 }   // Forward for 2 s at offset 300
//    { ACT_SPIN_LEFT, 600, 400 }  // Spin left for 0.6 s at offset 400
//    { ACT_STOP, 500, 0 }         // Pause for 0.5 s
//
//  HOW TO ADD A NEW PATH
//  ─────────────────────────────────────────────────────────
//  1. Copy one of the route arrays below and rename it
//  2. Edit its steps
//  3. Add a matching PATH_LEN_* constant
//  4. Add a button in the HTML sound board panel (search for
//     "ROUTE BUTTONS" in the HTML below)
//  5. Add one else-if line in handleCmd() (search for
//     "ROUTE COMMANDS" near the bottom of this file)
//
//  TUNING TIPS
//  ─────────────────────────────────────────────────────────
//  - Spin duration controls turning angle.  On a hard floor
//    ~500 ms ≈ 90°, ~1000 ms ≈ 180°.  Calibrate on your surface.
//  - Add ACT_STOP steps between moves for cleaner transitions.
//  - The ramp engine still runs during paths, so forward/backward
//    steps will always accelerate smoothly – no jerky starts.
// ============================================================

enum PathAction {
  ACT_FORWARD,
  ACT_BACKWARD,
  ACT_SPIN_LEFT,
  ACT_SPIN_RIGHT,
  ACT_STOP
};

struct PathStep {
  PathAction action;
  uint16_t   duration;      // ms
  uint16_t   speedOffset;   // µs offset, 0 = use current driveOffset
};

// ── Route 1: Square ──────────────────────────────────────────
//  Drives in an approximate square: forward, turn, repeat x4
const PathStep ROUTE_SQUARE[] = {
  { ACT_FORWARD,    2000, 300 },
  { ACT_STOP,        300,   0 },
  { ACT_SPIN_RIGHT,  550, 350 },   // ~90° right turn
  { ACT_STOP,        300,   0 },
  { ACT_FORWARD,    2000, 300 },
  { ACT_STOP,        300,   0 },
  { ACT_SPIN_RIGHT,  550, 350 },
  { ACT_STOP,        300,   0 },
  { ACT_FORWARD,    2000, 300 },
  { ACT_STOP,        300,   0 },
  { ACT_SPIN_RIGHT,  550, 350 },
  { ACT_STOP,        300,   0 },
  { ACT_FORWARD,    2000, 300 },
  { ACT_STOP,        300,   0 },
  { ACT_SPIN_RIGHT,  550, 350 },
  { ACT_STOP,        500,   0 },
};
const int PATH_LEN_SQUARE = sizeof(ROUTE_SQUARE) / sizeof(ROUTE_SQUARE[0]);

// ── Route 2: Figure-8 ────────────────────────────────────────
//  Two loops in opposing spin directions
const PathStep ROUTE_FIGURE8[] = {
  { ACT_FORWARD,   1500, 300 },
  { ACT_STOP,       300,   0 },
  { ACT_SPIN_RIGHT, 550, 350 },
  { ACT_STOP,       200,   0 },
  { ACT_FORWARD,   1500, 300 },
  { ACT_STOP,       300,   0 },
  { ACT_SPIN_RIGHT, 550, 350 },
  { ACT_STOP,       200,   0 },
  { ACT_FORWARD,   1500, 300 },
  { ACT_STOP,       300,   0 },
  { ACT_SPIN_RIGHT, 550, 350 },
  { ACT_STOP,       200,   0 },
  { ACT_FORWARD,   1500, 300 },
  { ACT_STOP,       300,   0 },
  { ACT_SPIN_RIGHT, 550, 350 },   // complete first loop
  { ACT_STOP,       400,   0 },
  { ACT_FORWARD,   1500, 300 },
  { ACT_STOP,       300,   0 },
  { ACT_SPIN_LEFT,  550, 350 },   // second loop opposite direction
  { ACT_STOP,       200,   0 },
  { ACT_FORWARD,   1500, 300 },
  { ACT_STOP,       300,   0 },
  { ACT_SPIN_LEFT,  550, 350 },
  { ACT_STOP,       200,   0 },
  { ACT_FORWARD,   1500, 300 },
  { ACT_STOP,       300,   0 },
  { ACT_SPIN_LEFT,  550, 350 },
  { ACT_STOP,       200,   0 },
  { ACT_FORWARD,   1500, 300 },
  { ACT_STOP,       300,   0 },
  { ACT_SPIN_LEFT,  550, 350 },
  { ACT_STOP,       500,   0 },
};
const int PATH_LEN_FIGURE8 = sizeof(ROUTE_FIGURE8) / sizeof(ROUTE_FIGURE8[0]);

// ── Route 3: Patrol ───────────────────────────────────────────
//  Forward, reverse, repeat – simple back-and-forth run
const PathStep ROUTE_PATROL[] = {
  { ACT_FORWARD,  3000, 280 },
  { ACT_STOP,      500,   0 },
  { ACT_BACKWARD, 3000, 280 },
  { ACT_STOP,      500,   0 },
  { ACT_FORWARD,  3000, 280 },
  { ACT_STOP,      500,   0 },
  { ACT_BACKWARD, 3000, 280 },
  { ACT_STOP,      500,   0 },
};
const int PATH_LEN_PATROL = sizeof(ROUTE_PATROL) / sizeof(ROUTE_PATROL[0]);

// ── ADD YOUR OWN ROUTES BELOW THIS LINE ──────────────────────
//
// const PathStep ROUTE_MYPATH[] = {
//   { ACT_FORWARD,    2000, 300 },
//   { ACT_STOP,        400,   0 },
//   { ACT_SPIN_LEFT,   500, 350 },
//   { ACT_STOP,        400,   0 },
// };
// const int PATH_LEN_MYPATH = sizeof(ROUTE_MYPATH) / sizeof(ROUTE_MYPATH[0]);

// ── Path runner state ─────────────────────────────────────────
bool           pathRunning    = false;
const PathStep* activePath    = nullptr;
int            pathLength     = 0;
int            pathStepIdx    = 0;
unsigned long  pathStepStart  = 0;


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
    header h1 { font-size: 2rem; letter-spacing: 3px; color: #e05a1b; text-transform: uppercase; }
    header p  { font-size: 0.85rem; color: #888; margin-top: 4px; }

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
    #status.ok   { background: #0f2d1a; color: #4caf50; border-color: #4caf50; }
    #status.err  { background: #2d0f0f; color: #f44336; border-color: #f44336; }
    #status.auto { background: #0a1f2d; color: #44aaff; border-color: #44aaff; }

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

    .btn-dir  { background: linear-gradient(145deg, #1e3a5f, #0e2040); box-shadow: 0 4px 12px rgba(0,0,0,0.5); }
    .btn-spin-l  { background: linear-gradient(145deg, #1e3a5f, #0e2040); box-shadow: 0 4px 12px rgba(0,0,0,0.5); }
    .btn-spin-r  { background: linear-gradient(145deg, #1e3a5f, #0e2040); box-shadow: 0 4px 12px rgba(0,0,0,0.5); }
    .btn-stop { background: linear-gradient(145deg, #7a1a1a, #4a0f0f); box-shadow: 0 4px 12px rgba(0,0,0,0.5); }

    .btn-fwd  { grid-column: 2; grid-row: 1; }
	.btn-stop { grid-column: 2; grid-row: 2; }
	.btn-bwd  { grid-column: 2; grid-row: 3; }

	.btn-spin-l { grid-column: 1; grid-row: 2; }
	.btn-spin-r { grid-column: 3; grid-row: 2; }

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
    .speed-section .sublabel  { font-size: 0.75rem; color: #555; margin-top: 8px; text-align: right; }
    input[type=range] { width: 100%; accent-color: #e05a1b; height: 6px; cursor: pointer; }

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

    /* ── Auto route panel ──────────────────────────────────── */
    .section-label {
      font-size: 0.72rem;
      letter-spacing: 2px;
      color: #555;
      text-transform: uppercase;
      width: 100%;
      max-width: 360px;
      text-align: center;
      border-top: 1px solid #222;
      padding-top: 16px;
    }
    .routes {
      display: flex;
      flex-wrap: wrap;
      gap: 10px;
      justify-content: center;
      max-width: 360px;
      width: 100%;
      padding-bottom: 24px;
    }
    .btn-route {
      flex: 1 1 150px;
      height: 72px;
      border-radius: 12px;
      font-size: 0.9rem;
      font-weight: 700;
      letter-spacing: 1px;
      text-transform: uppercase;
      cursor: pointer;
      transition: filter 0.15s, transform 0.1s;
      user-select: none;
      color: #44aaff;
      background: linear-gradient(145deg, #0a1e3f, #060f28);
      border: 1px solid #0a3a6a;
      box-shadow: 0 4px 12px rgba(0,0,0,0.5);
      -webkit-tap-highlight-color: transparent;
      display: flex;
      flex-direction: column;
      align-items: center;
      justify-content: center;
      gap: 4px;
    }
    .btn-route:active { transform: scale(0.93); filter: brightness(0.8); }
    .btn-abort {
      flex: 1 1 100%;
      height: 56px;
      border-radius: 12px;
      font-size: 0.9rem;
      font-weight: 700;
      letter-spacing: 1px;
      text-transform: uppercase;
      cursor: pointer;
      color: #fff;
      background: linear-gradient(145deg, #5a2a00, #3a1800);
      border: 1px solid #8a4400;
      box-shadow: 0 4px 12px rgba(0,0,0,0.5);
      -webkit-tap-highlight-color: transparent;
    }
    .btn-abort:active { transform: scale(0.97); filter: brightness(0.8); }
  </style>
</head>
<body>

  <header>
    <h1>&#127756; Mars Rover</h1>
    <p>ESP32 Wireless Control</p>
  </header>

  <div id="status">READY</div>

  <!-- ── D-PAD ─────────────────────────────────────────────── -->
  <div class="dpad">
  <button class="btn btn-dir btn-fwd" onclick="send('forward')">
    <span class="icon">&#9650;</span> FWD
  </button>

  <button class="btn btn-spin-l" onclick="send('spin_left')">
    <span class="icon">&#8634;</span> SPIN L
  </button>

  <button class="btn btn-stop" onclick="send('stop')">
    <span class="icon">&#9632;</span> STOP
  </button>

  <button class="btn btn-spin-r" onclick="send('spin_right')">
    <span class="icon">&#8635;</span> SPIN R
  </button>

  <button class="btn btn-dir btn-bwd" onclick="send('backward')">
    <span class="icon">&#9660;</span> REV
  </button>
</div>

  <!-- ── THROTTLE SLIDER ───────────────────────────────────── -->
  <div class="speed-section">
    <label>THROTTLE <span id="speedVal">350</span></label>
    <input type="range" id="speedSlider" min="80" max="500" value="350"
           oninput="updateSpeed(this.value)">
    <p class="sublabel">Signal offset from neutral (&micro;s)</p>
  </div>

  <!-- ── ROUTE BUTTONS ─────────────────────────────────────── -->
  <!--  To add a route: copy a btn-route block, change the     -->
  <!--  onclick action string to match your new handleCmd()    -->
  <!--  entry and give it a label.                             -->
  <div class="section-label">&#9654;&#9654; Autonomous Routes</div>
  <div class="routes">

    <button class="btn-route" onclick="sendRoute('route_square')">
      <span>&#9633; SQUARE</span>
    </button>

    <button class="btn-route" onclick="sendRoute('route_figure8')">
      <span>&#8734; FIGURE-8</span>
    </button>

    <button class="btn-route" onclick="sendRoute('route_patrol')">
      <span>&#8596; PATROL</span>
    </button>

    <!-- Template: copy and rename for new routes
    <button class="btn-route" onclick="sendRoute('route_mypath')">
      <span>&#9654; MY PATH</span>
    </button>
    -->

    <!-- ABORT button – always visible, stops any running route -->
    <button class="btn-abort" onclick="send('stop')">
      &#9632; ABORT ROUTE
    </button>

  </div>

  <script>
    // Manual drive command
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

    // Autonomous route command – blue status persists until route ends or is aborted
    function sendRoute(action) {
      const status = document.getElementById('status');
      status.className = 'auto';
      status.textContent = '\u25b6\u25b6 RUNNING...';
      fetch('/cmd?action=' + action)
        .then(r => r.text())
        .then(t => {
          status.className = 'auto';
          status.textContent = '\u25b6\u25b6 ' + t;
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

void applyBothESCs(int micros) {
  micros = constrain(micros, ESC_MIN, ESC_MAX);
  esc1.writeMicroseconds(micros);
  esc2.writeMicroseconds(micros);
  currentMicros = micros;
}

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
//  PATH RUNNER  –  non-blocking autonomous execution
//
//  startPath()  loads a route and fires the first step.
//  stopPath()   aborts immediately and hard-stops the rover.
//  updatePath() is called every loop() tick. It checks whether
//               the current step's duration has elapsed and if
//               so, executes the next step.
//
//  Steps with ACT_STOP insert a guaranteed motors-at-neutral
//  pause between moves, giving the ramp engine time to fully
//  settle before the next command fires.  Always add a short
//  ACT_STOP between direction changes in your routes.
//
//  Manual drive commands call stopPath() before executing,
//  so the driver always has full override authority.
// ============================================================

// Apply a single path step to the motors immediately.
// speedOffset of 0 means inherit the current driveOffset.
void applyPathStep(const PathStep& step) {
  int spd = (step.speedOffset > 0) ? step.speedOffset : driveOffset;

  switch (step.action) {
    case ACT_FORWARD:
      isSpinning    = false;
      currentDir    = DIR_FORWARD;
      pendingDir    = DIR_NONE;
      currentMicros = ESC_NEUTRAL + DEADBAND;
      // Store step speed so ramp engine targets it
      driveOffset   = spd;
      applyBothESCs(currentMicros);
      roverState    = RAMPING_UP;
      break;

    case ACT_BACKWARD:
      isSpinning    = false;
      currentDir    = DIR_BACKWARD;
      pendingDir    = DIR_NONE;
      currentMicros = ESC_NEUTRAL - DEADBAND;
      driveOffset   = spd;
      applyBothESCs(currentMicros);
      roverState    = RAMPING_UP;
      break;

    case ACT_SPIN_LEFT:
      isSpinning = true;
      roverState = RUNNING;
      currentDir = DIR_NONE;
      esc1.writeMicroseconds(ESC_NEUTRAL - spd);   // left  → reverse
      esc2.writeMicroseconds(ESC_NEUTRAL + spd);   // right → forward
      break;

    case ACT_SPIN_RIGHT:
      isSpinning = true;
      roverState = RUNNING;
      currentDir = DIR_NONE;
      esc1.writeMicroseconds(ESC_NEUTRAL + spd);   // left  → forward
      esc2.writeMicroseconds(ESC_NEUTRAL - spd);   // right → reverse
      break;

    case ACT_STOP:
    default:
      isSpinning = false;
      hardStop();
      break;
  }
}

void startPath(const PathStep* path, int length) {
  activePath   = path;
  pathLength   = length;
  pathStepIdx  = 0;
  pathRunning  = true;
  pathStepStart = millis();
  applyPathStep(activePath[0]);
}

void stopPath() {
  pathRunning = false;
  activePath  = nullptr;
  hardStop();
}

// Advance the path one step at a time using millis() – never blocks.
void updatePath() {
  if (!pathRunning) return;

  unsigned long now = millis();
  unsigned long elapsed = now - pathStepStart;

  if (elapsed >= (unsigned long)activePath[pathStepIdx].duration) {
    pathStepIdx++;

    if (pathStepIdx >= pathLength) {
      // Route complete
      stopPath();
      Serial.println("Route complete.");
      return;
    }

    pathStepStart = now;
    applyPathStep(activePath[pathStepIdx]);

    Serial.printf("Path step %d / %d\n", pathStepIdx + 1, pathLength);
  }
}

// ============================================================
//  RAMP ENGINE  –  non-blocking, called every loop()
// ============================================================

void updateRamp() {
  if (isSpinning) return;

  unsigned long now = millis();
  if (now - lastRampMillis < (unsigned long)RAMP_INTERVAL) return;
  lastRampMillis = now;

  switch (roverState) {

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

    case RAMPING_DOWN:
      if      (currentMicros > ESC_NEUTRAL) currentMicros -= RAMP_STEP;
      else if (currentMicros < ESC_NEUTRAL) currentMicros += RAMP_STEP;
      if (abs(currentMicros - ESC_NEUTRAL) < RAMP_STEP) currentMicros = ESC_NEUTRAL;
      applyBothESCs(currentMicros);
      if (currentMicros == ESC_NEUTRAL) {
        if (pendingDir != DIR_NONE) {
          currentDir    = pendingDir;
          pendingDir    = DIR_NONE;
          currentMicros = (currentDir == DIR_FORWARD)
                          ? ESC_NEUTRAL + DEADBAND
                          : ESC_NEUTRAL - DEADBAND;
          applyBothESCs(currentMicros);
          roverState = RAMPING_UP;
        } else {
          currentDir = DIR_NONE;
          roverState = IDLE;
        }
      }
      break;

    case RUNNING: {
      if (!isSpinning) {
        int target = (currentDir == DIR_FORWARD)
                     ? ESC_NEUTRAL + driveOffset
                     : ESC_NEUTRAL - driveOffset;
        if (currentMicros != target) {
          currentMicros = target;
          applyBothESCs(currentMicros);
        }
      }
      break;
    }

    case IDLE:
    default:
      break;
  }
}

// ============================================================
//  MANUAL COMMAND HANDLERS
//  All of these call stopPath() first so manual control
//  always immediately overrides any running route.
// ============================================================

void cmdMove(Direction dir) {
  if (pathRunning) stopPath();

  if (isSpinning) {
    esc1.writeMicroseconds(ESC_NEUTRAL);
    esc2.writeMicroseconds(ESC_NEUTRAL);
    isSpinning    = false;
    currentMicros = ESC_NEUTRAL;
    currentDir    = DIR_NONE;
    roverState    = IDLE;
  }

  if (dir == currentDir) {
    if (roverState == RAMPING_DOWN) { pendingDir = DIR_NONE; roverState = RAMPING_UP; }
    return;
  }

  if (roverState == IDLE || currentMicros == ESC_NEUTRAL) {
    currentDir    = dir;
    currentMicros = (dir == DIR_FORWARD)
                    ? ESC_NEUTRAL + DEADBAND
                    : ESC_NEUTRAL - DEADBAND;
    applyBothESCs(currentMicros);
    roverState = RAMPING_UP;
  } else {
    pendingDir = dir;
    roverState = RAMPING_DOWN;
  }
}

void cmdStop() {
  if (pathRunning) { stopPath(); return; }
  if (isSpinning)  { hardStop(); return; }
  pendingDir = DIR_NONE;
  if (roverState == IDLE) return;
  roverState = RAMPING_DOWN;
}

void cmdSpinLeft() {
  if (pathRunning) stopPath();
  isSpinning = true;
  currentDir = DIR_NONE;
  pendingDir = DIR_NONE;
  roverState = RUNNING;
  esc1.writeMicroseconds(ESC_NEUTRAL - driveOffset);
  esc2.writeMicroseconds(ESC_NEUTRAL + driveOffset);
}

void cmdSpinRight() {
  if (pathRunning) stopPath();
  isSpinning = true;
  currentDir = DIR_NONE;
  pendingDir = DIR_NONE;
  roverState = RUNNING;
  esc1.writeMicroseconds(ESC_NEUTRAL + driveOffset);
  esc2.writeMicroseconds(ESC_NEUTRAL - driveOffset);
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

  // ── Manual drive ──────────────────────────────────────────
  if      (action == "forward")    { cmdMove(DIR_FORWARD);  response = "FORWARD";  }
  else if (action == "backward")   { cmdMove(DIR_BACKWARD); response = "REVERSE";  }
  else if (action == "stop")       { cmdStop();              response = "STOPPED";  }
  else if (action == "spin_left")  { cmdSpinLeft();          response = "SPIN L";   }
  else if (action == "spin_right") { cmdSpinRight();         response = "SPIN R";   }

  // ── ROUTE COMMANDS ────────────────────────────────────────
  //  To add a route: copy one of these else-if lines,
  //  change the action string and the startPath() arguments.
  else if (action == "route_square")  {
    startPath(ROUTE_SQUARE,   PATH_LEN_SQUARE);
    response = "SQUARE ROUTE";
  }
  else if (action == "route_figure8") {
    startPath(ROUTE_FIGURE8,  PATH_LEN_FIGURE8);
    response = "FIGURE-8 ROUTE";
  }
  else if (action == "route_patrol")  {
    startPath(ROUTE_PATROL,   PATH_LEN_PATROL);
    response = "PATROL ROUTE";
  }
  // Template:
  // else if (action == "route_mypath") {
  //   startPath(ROUTE_MYPATH, PATH_LEN_MYPATH);
  //   response = "MY PATH";
  // }

  // ── Throttle slider ───────────────────────────────────────
  else if (action.startsWith("speed_")) {
    int val    = action.substring(6).toInt();
    val        = constrain(val, DEADBAND, 500);
    driveOffset = val;
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

  // Timers 2 & 3 – 0 & 1 are reserved by the WiFi stack
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  esc1.attach(ESC1_PIN, ESC_MIN, ESC_MAX);
  esc2.attach(ESC2_PIN, ESC_MIN, ESC_MAX);

  hardStop();
  delay(2000);   // hold neutral so ESCs arm cleanly

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
//  LOOP  –  three non-blocking tasks share this thread
// ============================================================

void loop() {
  server.handleClient();   // service web requests
  updateRamp();            // advance drive speed ramp
  updatePath();            // advance autonomous route
}
