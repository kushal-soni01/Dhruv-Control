#include <ArduinoBLE.h>

// ============================================================
// DHRUVBRAIN — JETSON NANO + ARDUINO UNO R4 HOLONOMIC CONTROLLER V2
// 4-Wheel Mecanum H-Drive + Jetson Command Protocol Translator
//
// Translates Jetson Nano / Web Dashboard high-level commands
// (<DIR,SPEED,TIME>|<LED,MOOD>) into native Arduino holonomic
// kinematic vectors (N, S, W, E, NE, NW, SE, SW, CW, CCW, Q)
//
// WHEEL LAYOUT (Mecanum 4-Wheel Holonomic)
//
//                    FRONT
//                      ↑
//             M1                 M4
//            FRONT-L           FRONT-R
//               \               /
//                \             /
//
//                /             \
//               /               \
//             M2                 M3
//            REAR-L            REAR-R
//                      ↓
//                     REAR
//
// Roller orientation:
//     M1 = "\"       M4 = "/"
//     M2 = "/"       M3 = "\"
//
// M1 and M3 = same roller orientation (\)
// M2 and M4 = opposite roller orientation (/)
//
// ============================================================
//
// COMMAND PROTOCOL & JETSON -> ARDUINO MAPPING
//
// 1. COMBINED JETSON PAYLOAD (Pipe Separated):
//    "<MOTOR_DIR,SPEED,TIME_MS>|<LED,MOOD>\n"
//    Example: "<FWD,150,500>|<LED,FLIRT_PINK>"
//
// 2. COLON FORMAT:
//    "MOTOR:<DIR>:<SPEED>:<TIME_MS>"
//    "LED:<MOOD>"
//    Example: "MOTOR:STRAFE_L:180:1000"
//
// 3. BRACKETED / STANDALONE MOTOR COMMANDS:
//    "<FWD,150,500>" or "<STOP>" or "FWD"
//
// 4. JETSON TO ARDUINO DIRECTION TRANSLATION TABLE:
//    ----------------------------------------------------------
//    Jetson Command     | Arduino Command | Holonomic Vector (Vx, Vy, W)
//    ----------------------------------------------------------
//    FWD / APPROACH     | N               | (+Vx, 0, 0)
//    REV / BACKUP / BWD | S               | (-Vx, 0, 0)
//    STRAFE_L / LEFT    | W               | (0, +Vy, 0)
//    STRAFE_R / RIGHT   | E               | (0, -Vy, 0)
//    DIAG_FL / FWD_L    | NW              | (+Vx, +Vy, 0)
//    DIAG_FR / FWD_R    | NE              | (+Vx, -Vy, 0)
//    DIAG_BL / BWD_L    | SW              | (-Vx, +Vy, 0)
//    DIAG_BR / BWD_R    | SE              | (-Vx, -Vy, 0)
//    PIVOT_L / CCW      | CCW             | (0, 0, +W)
//    PIVOT_R / CW       | CW              | (0, 0, -W)
//    STOP / HALT        | Q / X           | (0, 0, 0)
//    WANDER             | WANDER          | (+Vx, 0, +W/3)
//    ----------------------------------------------------------
//
// 5. MOTOR CALIBRATION TOKENS (Direct Testing):
//    T1_150             -> M1 forward at PWM 150
//    t1_150             -> M1 reverse at PWM 150
//    T1_150 T4_150      -> M1 & M4 forward at PWM 150
//
// ============================================================

// ============================================================
// BLE DEFINITIONS (Jetson Nano Central & Web Dashboard Compatible)
// ============================================================

// Primary Jetson Nano Service UUID
BLEService jetsonService("19b10000-e8f2-537e-4f6c-d104768a1214");

// Combined Command & Telemetry Characteristic
BLEStringCharacteristic commandChar(
  "19b10001-e8f2-537e-4f6c-d104768a1214",
  BLEWrite | BLEWriteWithoutResponse | BLENotify | BLERead,
  100
);

// Optional Secondary Service for Web Dashboard Compatibility
BLEService dashboardService("12345678-1234-1234-1234-1234567890AB");
BLEStringCharacteristic dashCmdChar("12345678-1234-1234-1234-1234567890CD", BLEWrite | BLEWriteWithoutResponse, 40);
BLEStringCharacteristic dashLogChar("12345678-1234-1234-1234-1234567890EF", BLERead | BLENotify, 100);

bool bleCentralConnected = false;

// ============================================================
// BLE / SERIAL LOGGING
// ============================================================

void bleLog(String msg)
{
  Serial.println(msg);

  if (bleCentralConnected)
  {
    String truncMsg = msg;
    if (truncMsg.length() > 99)
      truncMsg = truncMsg.substring(0, 99);

    commandChar.writeValue(truncMsg);
    dashLogChar.writeValue(truncMsg);
  }
}

// ============================================================
// MOTOR PINS (IBT-2 / BTS7960 Motor Drivers on UNO R4)
//
// REN and LEN are tied to 5V (VCC).
// Arduino PWM controls RPWM and LPWM.
//
// M1 = Front Left
// M2 = Rear Left
// M3 = Rear Right
// M4 = Front Right
// ============================================================

#define M1_RPWM 2
#define M1_LPWM 3

#define M2_RPWM 4
#define M2_LPWM 5

#define M3_RPWM 6
#define M3_LPWM 7

#define M4_RPWM 8
#define M4_LPWM 9

// ============================================================
// SETTINGS & CALIBRATION
// ============================================================

const int SPEED_LIMIT = 180;
const bool DEBUG = true;

// Direction inversion toggles (change to true if a motor spins backwards)
bool M1_INVERT = false;
bool M2_INVERT = false;
bool M3_INVERT = false;
bool M4_INVERT = false;

bool testModeActive = false;

// ============================================================
// NON-BLOCKING TIMED MOTOR EXECUTION
// ============================================================

unsigned long motorStartTime = 0;
unsigned long motorDuration = 0;
bool motorTimedActive = false;

// ============================================================
// 12V LED / MOSFET CONTROLLER (JETSON MOOD SYSTEM)
// ============================================================

const int LED_PIN = 13; // Connects to MOSFET gate for 12V Blue Light

enum LedMood
{
  MOOD_STARTUP,
  MOOD_IDLE_WHITE,       // Solid ON (Passive waiting)
  MOOD_CURIOSITY_GREEN,  // Slow pulsing breathing fade (Analyzing)
  MOOD_FLIRT_PINK,       // Double blink pattern (Approach / Flirt)
  MOOD_ALERT_RED,        // Rapid aggressive strobe (Obstacle / Error)
  MOOD_THINKING_BLUE,    // Neural pulse random flicker (LLM inference)
  MOOD_SPEAKING,         // 200ms rhythmic blink (Speech output)
  MOOD_OFF               // Solid OFF
};

LedMood currentLedMood = MOOD_STARTUP;

const int blinkInterval = 50;
const unsigned long startupBlinkDuration = 2500;

unsigned long ledPreviousMillis = 0;
unsigned long ledPhaseStartMillis = 0;
int ledBrightness = 0;
bool ledState = LOW;
bool ledFadeDirection = true;

// ============================================================
// FUNCTION DECLARATIONS
// ============================================================

void handleIncomingLine(String line);
void parseJetsonProtocol(String line);
void parseMotorSegment(String str);
void parseLedSegment(String str);
void parseColonMotor(String str);
void parseColonLed(String str);

void translateJetsonToArduino(String jetsonDir, int speed, unsigned long durationMs);
void executeArduinoCommand(String arduinoCmd, int speed = SPEED_LIMIT);

void setLedMood(LedMood mood);
void setLedMoodByName(String moodName);
void updateStatusLED();
void checkMotorTimer();

void mixDrive(int Vx, int Vy, int W);
void moveForward(int speed = SPEED_LIMIT);
void moveBackward(int speed = SPEED_LIMIT);
void strafeLeft(int speed = SPEED_LIMIT);
void strafeRight(int speed = SPEED_LIMIT);
void moveForwardRight(int speed = SPEED_LIMIT);
void moveForwardLeft(int speed = SPEED_LIMIT);
void moveBackwardRight(int speed = SPEED_LIMIT);
void moveBackwardLeft(int speed = SPEED_LIMIT);
void rotateCW(int speed = SPEED_LIMIT);
void rotateCCW(int speed = SPEED_LIMIT);
void stopAll();

void setMotor(int motor, int speed);
void setRawMotor(int motor, int speed);
void runMotorTokens(String line);
bool parseMotorToken(String token, int &motor, int &speed, bool &reverse);

// ============================================================
// SETUP
// ============================================================

void setup()
{
  Serial.begin(115200);

  // Initialize Motor PWM Pins
  pinMode(M1_RPWM, OUTPUT);
  pinMode(M1_LPWM, OUTPUT);
  pinMode(M2_RPWM, OUTPUT);
  pinMode(M2_LPWM, OUTPUT);
  pinMode(M3_RPWM, OUTPUT);
  pinMode(M3_LPWM, OUTPUT);
  pinMode(M4_RPWM, OUTPUT);
  pinMode(M4_LPWM, OUTPUT);

  // Safety startup: stop all motors
  stopAll();

  // Status MOSFET Pin
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  ledPhaseStartMillis = millis();

  // Start BLE
  if (!BLE.begin())
  {
    Serial.println("ERROR: Bluetooth Low Energy initialization failed!");
    while (1)
    {
      stopAll();
      delay(200);
    }
  }

  // Configure Advertised Services
  BLE.setLocalName("Dhruv_Arduino");
  BLE.setAdvertisedService(jetsonService);

  jetsonService.addCharacteristic(commandChar);
  BLE.addService(jetsonService);

  dashboardService.addCharacteristic(dashCmdChar);
  dashboardService.addCharacteristic(dashLogChar);
  BLE.addService(dashboardService);

  BLE.advertise();

  Serial.println();
  Serial.println("==========================================================");
  Serial.println("  DHRUVBRAIN JETSON-TO-ARDUINO COMMAND PARSER V2");
  Serial.println("  4-Wheel Holonomic Mecanum H-Drive & Mood Light Controller");
  Serial.println("==========================================================");
  Serial.println("BLE Device Name:   Dhruv_Arduino");
  Serial.println("Jetson Service:    19b10000-e8f2-537e-4f6c-d104768a1214");
  Serial.println("Dashboard Service: 12345678-1234-1234-1234-1234567890AB");
  Serial.println("Listening for Terminal (Serial USB) and BLE Dashboard...");
  Serial.println("==========================================================");
}

// ============================================================
// MAIN LOOP
// ============================================================

void loop()
{
  // 1. Maintain LED Mood animation
  updateStatusLED();

  // 2. Check non-blocking auto-stop motor timer
  checkMotorTimer();

  // 3. Check Serial Input (from Terminal / USB / Jetson Python)
  if (Serial.available())
  {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() > 0)
    {
      handleIncomingLine(line);
    }
  }

  // 4. Check BLE Central Connection
  BLEDevice central = BLE.central();

  if (central)
  {
    bleCentralConnected = true;
    Serial.print("BLE Central Connected: ");
    Serial.println(central.address());
    bleLog("SYS: Central Connected (" + central.address() + ")");

    while (central.connected())
    {
      updateStatusLED();
      checkMotorTimer();

      // Check Jetson BLE Command Characteristic
      if (commandChar.written())
      {
        String val = commandChar.value();
        val.trim();
        if (val.length() > 0)
        {
          handleIncomingLine(val);
        }
      }

      // Check Dashboard BLE Command Characteristic
      if (dashCmdChar.written())
      {
        String val = dashCmdChar.value();
        val.trim();
        if (val.length() > 0)
        {
          handleIncomingLine(val);
        }
      }

      // Check Serial input concurrently while BLE connected
      if (Serial.available())
      {
        String line = Serial.readStringUntil('\n');
        line.trim();
        if (line.length() > 0)
        {
          handleIncomingLine(line);
        }
      }

      BLE.poll();
    }

    // Central disconnected safety
    stopAll();
    motorTimedActive = false;
    bleCentralConnected = false;
    Serial.println("BLE Disconnected. Motors safely stopped.");
  }
}

// ============================================================
// TIMED MOTOR CHECK (NON-BLOCKING)
// ============================================================

void checkMotorTimer()
{
  if (motorTimedActive && motorDuration > 0)
  {
    if (millis() - motorStartTime >= motorDuration)
    {
      stopAll();
      motorTimedActive = false;
      motorDuration = 0;
      bleLog("[AUTO-STOP] Duration elapsed (" + String(motorDuration) + "ms) -> Robot halted.");
    }
  }
}

// ============================================================
// MAIN INCOMING LINE ROUTER
// ============================================================

void handleIncomingLine(String line)
{
  line.trim();
  if (line.length() == 0)
    return;

  // 1. Motor Calibration Tokens (e.g. "T1_150", "t2_120", "T1_150 T4_150")
  char first = line.charAt(0);
  if ((first == 'T' || first == 't') && line.indexOf('_') != -1 && line.indexOf('<') == -1)
  {
    testModeActive = true;
    runMotorTokens(line);
    return;
  }

  // 2. Parse Jetson Protocol or direct commands
  parseJetsonProtocol(line);
}

// ============================================================
// JETSON PROTOCOL PARSER & TRANSLATOR
// ============================================================

void parseJetsonProtocol(String line)
{
  testModeActive = false;

  // Log incoming raw line
  bleLog("[RX RAW] " + line);

  // Case 1: Combined Pipe Format "<MOTOR_CMD>|<LED_CMD>"
  int pipeIndex = line.indexOf('|');
  if (pipeIndex != -1)
  {
    String motorPart = line.substring(0, pipeIndex);
    String ledPart = line.substring(pipeIndex + 1);

    motorPart.trim();
    ledPart.trim();

    if (motorPart.length() > 0)
      parseMotorSegment(motorPart);

    if (ledPart.length() > 0)
      parseLedSegment(ledPart);

    return;
  }

  // Case 2: Tagged Colon Format ("MOTOR:..." or "LED:...")
  if (line.startsWith("MOTOR:") || line.startsWith("motor:"))
  {
    parseColonMotor(line);
    return;
  }

  if (line.startsWith("LED:") || line.startsWith("led:"))
  {
    parseColonLed(line);
    return;
  }

  // Case 3: Single Bracketed Token "<...>"
  if (line.startsWith("<") && line.endsWith(">"))
  {
    String inner = line.substring(1, line.length() - 1);
    inner.trim();

    if (inner.startsWith("LED,") || inner.startsWith("led,") || inner.startsWith("LED:") || inner.startsWith("led:"))
    {
      parseLedSegment(inner);
    }
    else
    {
      parseMotorSegment(line);
    }
    return;
  }

  // Case 4: Standalone direct token (e.g., "FWD", "STRAFE_L", "N", "Q", "CW", "STOP")
  translateJetsonToArduino(line, SPEED_LIMIT, 0);
}

// ============================================================
// PARSE MOTOR SEGMENT: "<DIR,SPEED,TIME_MS>" or "<STOP>"
// ============================================================

void parseMotorSegment(String str)
{
  str.trim();

  // Strip '<' and '>'
  if (str.startsWith("<"))
    str = str.substring(1);
  if (str.endsWith(">"))
    str = str.substring(0, str.length() - 1);
  str.trim();

  if (str.length() == 0)
    return;

  // Split by commas
  int firstComma = str.indexOf(',');
  if (firstComma == -1)
  {
    // Single token like "STOP", "FWD", "Q", "WANDER"
    translateJetsonToArduino(str, SPEED_LIMIT, 0);
    return;
  }

  String dirStr = str.substring(0, firstComma);
  dirStr.trim();

  int speed = SPEED_LIMIT;
  unsigned long duration = 0;

  int secondComma = str.indexOf(',', firstComma + 1);
  if (secondComma == -1)
  {
    // Format: <DIR, SPEED>
    String speedStr = str.substring(firstComma + 1);
    speedStr.trim();
    if (speedStr.length() > 0)
      speed = speedStr.toInt();
  }
  else
  {
    // Format: <DIR, SPEED, TIME_MS>
    String speedStr = str.substring(firstComma + 1, secondComma);
    String timeStr = str.substring(secondComma + 1);
    speedStr.trim();
    timeStr.trim();

    if (speedStr.length() > 0)
      speed = speedStr.toInt();
    if (timeStr.length() > 0)
      duration = (unsigned long)timeStr.toInt();
  }

  translateJetsonToArduino(dirStr, speed, duration);
}

// ============================================================
// PARSE LED SEGMENT: "<LED,MOOD>" or "<MOOD>"
// ============================================================

void parseLedSegment(String str)
{
  str.trim();

  if (str.startsWith("<"))
    str = str.substring(1);
  if (str.endsWith(">"))
    str = str.substring(0, str.length() - 1);
  str.trim();

  if (str.startsWith("LED,") || str.startsWith("led,"))
  {
    str = str.substring(4);
    str.trim();
  }
  else if (str.startsWith("LED:") || str.startsWith("led:"))
  {
    str = str.substring(4);
    str.trim();
  }

  setLedMoodByName(str);
}

// ============================================================
// PARSE COLON MOTOR: "MOTOR:FWD:255:500" or "MOTOR:STOP"
// ============================================================

void parseColonMotor(String str)
{
  str = str.substring(6);
  str.trim();

  int firstColon = str.indexOf(':');
  if (firstColon == -1)
  {
    translateJetsonToArduino(str, SPEED_LIMIT, 0);
    return;
  }

  String dirStr = str.substring(0, firstColon);
  dirStr.trim();

  int speed = SPEED_LIMIT;
  unsigned long duration = 0;

  int secondColon = str.indexOf(':', firstColon + 1);
  if (secondColon == -1)
  {
    String speedStr = str.substring(firstColon + 1);
    speedStr.trim();
    if (speedStr.length() > 0)
      speed = speedStr.toInt();
  }
  else
  {
    String speedStr = str.substring(firstColon + 1, secondColon);
    String timeStr = str.substring(secondColon + 1);
    speedStr.trim();
    timeStr.trim();

    if (speedStr.length() > 0)
      speed = speedStr.toInt();
    if (timeStr.length() > 0)
      duration = (unsigned long)timeStr.toInt();
  }

  translateJetsonToArduino(dirStr, speed, duration);
}

// ============================================================
// PARSE COLON LED: "LED:FLIRT_PINK"
// ============================================================

void parseColonLed(String str)
{
  str = str.substring(4);
  str.trim();
  setLedMoodByName(str);
}

// ============================================================
// TRANSLATOR: JETSON FORMAT -> ARDUINO COMMAND & MOTION
// ============================================================

void translateJetsonToArduino(String jetsonDir, int speed, unsigned long durationMs)
{
  jetsonDir.toUpperCase();
  jetsonDir.trim();

  if (speed <= 0)
    speed = SPEED_LIMIT;
  speed = constrain(speed, 0, 255);

  // Setup duration timer
  if (durationMs > 0)
  {
    motorStartTime = millis();
    motorDuration = durationMs;
    motorTimedActive = true;
  }
  else
  {
    motorTimedActive = false;
    motorDuration = 0;
  }

  // ----------------------------------------------------------
  // Map Jetson Direction String back to Native Arduino Command
  // ----------------------------------------------------------
  String arduinoCmd = "";
  String friendlyName = "";

  // 1. FORWARD / APPROACH -> "N"
  if (jetsonDir == "FWD" || jetsonDir == "FORWARD" || jetsonDir == "APPROACH_0.5M" || jetsonDir == "N")
  {
    arduinoCmd = "N";
    friendlyName = "FORWARD";
    moveForward(speed);
  }
  // 2. BACKWARD / BACKUP -> "S"
  else if (jetsonDir == "REV" || jetsonDir == "BACK" || jetsonDir == "BACKWARD" || jetsonDir == "BWD" || jetsonDir == "BACKUP_0.5M" || jetsonDir == "S")
  {
    arduinoCmd = "S";
    friendlyName = "BACKWARD";
    moveBackward(speed);
  }
  // 3. STRAFE LEFT -> "W"
  else if (jetsonDir == "STRAFE_L" || jetsonDir == "STRAFE_LEFT" || jetsonDir == "LEFT" || jetsonDir == "W")
  {
    arduinoCmd = "W";
    friendlyName = "STRAFE_LEFT";
    strafeLeft(speed);
  }
  // 4. STRAFE RIGHT -> "E"
  else if (jetsonDir == "STRAFE_R" || jetsonDir == "STRAFE_RIGHT" || jetsonDir == "RIGHT" || jetsonDir == "E")
  {
    arduinoCmd = "E";
    friendlyName = "STRAFE_RIGHT";
    strafeRight(speed);
  }
  // 5. DIAGONAL FORWARD-LEFT -> "NW"
  else if (jetsonDir == "DIAG_FL" || jetsonDir == "DIAGONAL_FL" || jetsonDir == "FWD_L" || jetsonDir == "FORWARD_LEFT" || jetsonDir == "NW")
  {
    arduinoCmd = "NW";
    friendlyName = "FORWARD_LEFT";
    moveForwardLeft(speed);
  }
  // 6. DIAGONAL FORWARD-RIGHT -> "NE"
  else if (jetsonDir == "DIAG_FR" || jetsonDir == "DIAGONAL_FR" || jetsonDir == "FWD_R" || jetsonDir == "FORWARD_RIGHT" || jetsonDir == "NE")
  {
    arduinoCmd = "NE";
    friendlyName = "FORWARD_RIGHT";
    moveForwardRight(speed);
  }
  // 7. DIAGONAL BACKWARD-LEFT -> "SW"
  else if (jetsonDir == "DIAG_BL" || jetsonDir == "DIAGONAL_BL" || jetsonDir == "BWD_L" || jetsonDir == "BACKWARD_LEFT" || jetsonDir == "SW")
  {
    arduinoCmd = "SW";
    friendlyName = "BACKWARD_LEFT";
    moveBackwardLeft(speed);
  }
  // 8. DIAGONAL BACKWARD-RIGHT -> "SE"
  else if (jetsonDir == "DIAG_BR" || jetsonDir == "DIAGONAL_BR" || jetsonDir == "BWD_R" || jetsonDir == "BACKWARD_RIGHT" || jetsonDir == "SE")
  {
    arduinoCmd = "SE";
    friendlyName = "BACKWARD_RIGHT";
    moveBackwardRight(speed);
  }
  // 9. ROTATE COUNTER-CLOCKWISE / PIVOT LEFT -> "CCW"
  else if (jetsonDir == "PIVOT_L" || jetsonDir == "PIVOT_LEFT_30" || jetsonDir == "CCW" || jetsonDir == "ROTATE_CCW" || jetsonDir == "ROTATE_L")
  {
    arduinoCmd = "CCW";
    friendlyName = "ROTATE_CCW";
    rotateCCW(speed);
  }
  // 10. ROTATE CLOCKWISE / PIVOT RIGHT -> "CW"
  else if (jetsonDir == "PIVOT_R" || jetsonDir == "PIVOT_RIGHT_30" || jetsonDir == "CW" || jetsonDir == "ROTATE_CW" || jetsonDir == "ROTATE_R")
  {
    arduinoCmd = "CW";
    friendlyName = "ROTATE_CW";
    rotateCW(speed);
  }
  // 11. STOP / HALT -> "Q"
  else if (jetsonDir == "STOP" || jetsonDir == "HALT" || jetsonDir == "Q" || jetsonDir == "X")
  {
    arduinoCmd = "Q";
    friendlyName = "STOP";
    stopAll();
  }
  // 12. WANDER (Special AI exploration mode)
  else if (jetsonDir == "WANDER")
  {
    arduinoCmd = "WANDER";
    friendlyName = "AI_WANDER";
    mixDrive(speed, 0, speed / 3);
  }
  else
  {
    bleLog("[ERR] Unrecognized Jetson direction token: " + jetsonDir);
    return;
  }

  // Telemetry output showing Jetson parsed back into Arduino format
  String logStr = "[PARSED] Jetson: '" + jetsonDir + "' -> Arduino: '" + arduinoCmd +
                  "' (" + friendlyName + ") | Spd=" + String(speed) + " [Continuous]";
  bleLog(logStr);
}

// ============================================================
// DIRECT ARDUINO COMMAND EXECUTOR
// ============================================================

void executeArduinoCommand(String arduinoCmd, int speed)
{
  arduinoCmd.toUpperCase();
  arduinoCmd.trim();

  if (arduinoCmd == "N")   { moveForward(speed); }
  else if (arduinoCmd == "S")   { moveBackward(speed); }
  else if (arduinoCmd == "W")   { strafeLeft(speed); }
  else if (arduinoCmd == "E")   { strafeRight(speed); }
  else if (arduinoCmd == "NW")  { moveForwardLeft(speed); }
  else if (arduinoCmd == "NE")  { moveForwardRight(speed); }
  else if (arduinoCmd == "SW")  { moveBackwardLeft(speed); }
  else if (arduinoCmd == "SE")  { moveBackwardRight(speed); }
  else if (arduinoCmd == "CW")  { rotateCW(speed); }
  else if (arduinoCmd == "CCW") { rotateCCW(speed); }
  else if (arduinoCmd == "Q" || arduinoCmd == "X" || arduinoCmd == "STOP") { stopAll(); }
}

// ============================================================
// 12V LED / MOSFET MOOD STATE MACHINE
// ============================================================

void setLedMood(LedMood mood)
{
  currentLedMood = mood;
  ledPhaseStartMillis = millis();
  ledPreviousMillis = millis();
  ledBrightness = 0;
}

void setLedMoodByName(String moodName)
{
  moodName.toUpperCase();
  moodName.trim();

  if (moodName == "IDLE_WHITE" || moodName == "IDLE" || moodName == "WHITE")
  {
    setLedMood(MOOD_IDLE_WHITE);
    bleLog("[LED] Mood: IDLE_WHITE (Solid ON)");
  }
  else if (moodName == "CURIOSITY_GREEN" || moodName == "CURIOSITY" || moodName == "GREEN")
  {
    setLedMood(MOOD_CURIOSITY_GREEN);
    bleLog("[LED] Mood: CURIOSITY_GREEN (Breathing Pulse)");
  }
  else if (moodName == "FLIRT_PINK" || moodName == "FLIRT" || moodName == "PINK")
  {
    setLedMood(MOOD_FLIRT_PINK);
    bleLog("[LED] Mood: FLIRT_PINK (Double Blink)");
  }
  else if (moodName == "ALERT_RED" || moodName == "ALERT" || moodName == "RED")
  {
    setLedMood(MOOD_ALERT_RED);
    bleLog("[LED] Mood: ALERT_RED (Aggressive Strobe)");
  }
  else if (moodName == "THINKING_BLUE" || moodName == "THINKING" || moodName == "BLUE")
  {
    setLedMood(MOOD_THINKING_BLUE);
    bleLog("[LED] Mood: THINKING_BLUE (Neural Flicker)");
  }
  else if (moodName == "SPEAKING" || moodName == "SPEAK")
  {
    setLedMood(MOOD_SPEAKING);
    bleLog("[LED] Mood: SPEAKING (Speech Rhythm)");
  }
  else if (moodName == "OFF" || moodName == "0")
  {
    setLedMood(MOOD_OFF);
    bleLog("[LED] Mood: OFF");
  }
  else
  {
    bleLog("[LED ERR] Unknown Mood: " + moodName);
  }
}

void updateStatusLED()
{
  unsigned long currentMillis = millis();

  switch (currentLedMood)
  {
    // Startup double blink sequence
    case MOOD_STARTUP:
      if (currentMillis - ledPhaseStartMillis >= startupBlinkDuration)
      {
        setLedMood(MOOD_IDLE_WHITE);
      }
      else if (currentMillis - ledPreviousMillis >= blinkInterval)
      {
        ledPreviousMillis = currentMillis;
        ledState = !ledState;
        digitalWrite(LED_PIN, ledState ? HIGH : LOW);
      }
      break;

    // Solid ON
    case MOOD_IDLE_WHITE:
      digitalWrite(LED_PIN, HIGH);
      break;

    // Solid OFF
    case MOOD_OFF:
      digitalWrite(LED_PIN, LOW);
      break;

    // Curiosity breathing fade
    case MOOD_CURIOSITY_GREEN:
      if (currentMillis - ledPreviousMillis >= 10)
      {
        ledPreviousMillis = currentMillis;
        if (ledFadeDirection)
        {
          ledBrightness += 2;
          if (ledBrightness >= 255)
          {
            ledBrightness = 255;
            ledFadeDirection = false;
          }
        }
        else
        {
          ledBrightness -= 2;
          if (ledBrightness <= 10)
          {
            ledBrightness = 10;
            ledFadeDirection = true;
          }
        }
        analogWrite(LED_PIN, ledBrightness);
      }
      break;

    // Flirt double-blink: ON(80ms) -> OFF(80ms) -> ON(80ms) -> OFF(760ms)
    case MOOD_FLIRT_PINK:
      {
        unsigned long cycleTime = (currentMillis - ledPhaseStartMillis) % 1000;
        if (cycleTime < 80)
          digitalWrite(LED_PIN, HIGH);
        else if (cycleTime < 160)
          digitalWrite(LED_PIN, LOW);
        else if (cycleTime < 240)
          digitalWrite(LED_PIN, HIGH);
        else
          digitalWrite(LED_PIN, LOW);
      }
      break;

    // Fast aggressive strobe (40ms ON / 40ms OFF)
    case MOOD_ALERT_RED:
      if (currentMillis - ledPreviousMillis >= 40)
      {
        ledPreviousMillis = currentMillis;
        ledState = !ledState;
        digitalWrite(LED_PIN, ledState ? HIGH : LOW);
      }
      break;

    // Thinking: neural rapid flickering
    case MOOD_THINKING_BLUE:
      if (currentMillis - ledPreviousMillis >= 25)
      {
        ledPreviousMillis = currentMillis;
        int flickerVal = random(30, 255);
        analogWrite(LED_PIN, flickerVal);
      }
      break;

    // Speaking: 200ms speech rhythm
    case MOOD_SPEAKING:
      if (currentMillis - ledPreviousMillis >= 200)
      {
        ledPreviousMillis = currentMillis;
        ledState = !ledState;
        digitalWrite(LED_PIN, ledState ? HIGH : LOW);
      }
      break;
  }
}

// ============================================================
// BASIC MOVEMENTS
// ============================================================

void moveForward(int speed)      { mixDrive(speed, 0, 0); }
void moveBackward(int speed)     { mixDrive(-speed, 0, 0); }
void strafeLeft(int speed)       { mixDrive(0, speed, 0); }
void strafeRight(int speed)      { mixDrive(0, -speed, 0); }

void moveForwardRight(int speed) { mixDrive(speed, -speed, 0); }
void moveForwardLeft(int speed)  { mixDrive(speed, speed, 0); }
void moveBackwardRight(int speed){ mixDrive(-speed, -speed, 0); }
void moveBackwardLeft(int speed) { mixDrive(-speed, speed, 0); }

void rotateCW(int speed)         { mixDrive(0, 0, -speed); }
void rotateCCW(int speed)        { mixDrive(0, 0, speed); }

void stopAll()
{
  setMotor(1, 0);
  setMotor(2, 0);
  setMotor(3, 0);
  setMotor(4, 0);
}

// ============================================================
// HOLONOMIC MECANUM MIXING (KINEMATICS)
//
// Coordinate system:
//   Vx = Forward (+) / Backward (-)
//   Vy = Left (+) / Right (-)
//   W  = Rotation Counter-Clockwise (+) / Clockwise (-)
//
// Equations:
//   M1 = Vx - Vy - W
//   M4 = Vx + Vy + W
//   M2 = Vx + Vy - W
//   M3 = Vx - Vy + W
// ============================================================

void mixDrive(int Vx, int Vy, int W)
{
  float M1 = Vx - Vy - W;
  float M4 = Vx + Vy + W;
  float M2 = Vx + Vy - W;
  float M3 = Vx - Vy + W;

  // Normalization: scale proportionally if any motor exceeds SPEED_LIMIT
  float maxMag = max(max(fabs(M1), fabs(M2)), max(fabs(M3), fabs(M4)));

  if (maxMag > SPEED_LIMIT)
  {
    float scale = (float)SPEED_LIMIT / maxMag;
    M1 *= scale;
    M2 *= scale;
    M3 *= scale;
    M4 *= scale;
  }

  setMotor(1, (int)M1);
  setMotor(2, (int)M2);
  setMotor(3, (int)M3);
  setMotor(4, (int)M4);

  if (DEBUG)
  {
    String line = "[MIX] Vx=" + String(Vx) + " Vy=" + String(Vy) + " W=" + String(W) +
                  " -> M1=" + String((int)M1) + " M2=" + String((int)M2) +
                  " M3=" + String((int)M3) + " M4=" + String((int)M4);
    bleLog(line);
  }
}

// ============================================================
// MOTOR CALIBRATION TOKEN PARSER
// ============================================================

bool parseMotorToken(String token, int &motor, int &speed, bool &reverse)
{
  if (token.length() < 4)
    return false;

  char dirChar = token.charAt(0);
  if (dirChar != 'T' && dirChar != 't')
    return false;

  reverse = (dirChar == 't');
  char motorChar = token.charAt(1);
  if (motorChar < '1' || motorChar > '4')
    return false;

  motor = motorChar - '0';
  if (token.charAt(2) != '_')
    return false;

  String speedStr = token.substring(3);
  if (speedStr.length() == 0)
    return false;

  for (unsigned int i = 0; i < speedStr.length(); i++)
  {
    if (!isDigit(speedStr.charAt(i)))
      return false;
  }

  speed = constrain(speedStr.toInt(), 0, 255);
  return true;
}

void runMotorTokens(String line)
{
  bool used[5] = {false, false, false, false, false};
  int speed[5] = {0, 0, 0, 0, 0};
  bool rev[5] = {false, false, false, false, false};
  String summary = "";
  bool anyValid = false;

  int start = 0;
  while (start < (int)line.length())
  {
    int spaceIdx = line.indexOf(' ', start);
    String token;
    if (spaceIdx == -1)
      token = line.substring(start);
    else
      token = line.substring(start, spaceIdx);

    token.trim();
    if (token.length() > 0)
    {
      int m;
      int spd;
      bool rv;
      if (parseMotorToken(token, m, spd, rv))
      {
        used[m] = true;
        speed[m] = spd;
        rev[m] = rv;
        anyValid = true;
        summary += "M" + String(m) + "=" + (rv ? "-" : "+") + String(spd) + " ";
      }
      else
      {
        bleLog("[TEST] Bad token: " + token);
      }
    }

    if (spaceIdx == -1)
      break;
    start = spaceIdx + 1;
  }

  if (!anyValid)
  {
    bleLog("[TEST] No valid tokens.");
    return;
  }

  for (int m = 1; m <= 4; m++)
  {
    int s = used[m] ? (rev[m] ? -speed[m] : speed[m]) : 0;
    setMotor(m, s);
  }

  bleLog("[TEST CALIBRATION] " + summary + "(others stopped)");
}

// ============================================================
// MOTOR INVERSION & RAW PWM CONTROL (IBT-2 BTS7960)
// ============================================================

void setMotor(int motor, int speed)
{
  switch (motor)
  {
    case 1: if (M1_INVERT) speed = -speed; break;
    case 2: if (M2_INVERT) speed = -speed; break;
    case 3: if (M3_INVERT) speed = -speed; break;
    case 4: if (M4_INVERT) speed = -speed; break;
    default: return;
  }
  setRawMotor(motor, speed);
}

void setRawMotor(int motor, int speed)
{
  int rpwmPin;
  int lpwmPin;

  switch (motor)
  {
    case 1: rpwmPin = M1_RPWM; lpwmPin = M1_LPWM; break;
    case 2: rpwmPin = M2_RPWM; lpwmPin = M2_LPWM; break;
    case 3: rpwmPin = M3_RPWM; lpwmPin = M3_LPWM; break;
    case 4: rpwmPin = M4_RPWM; lpwmPin = M4_LPWM; break;
    default: return;
  }

  speed = constrain(speed, -255, 255);

  if (speed > 0)
  {
    analogWrite(lpwmPin, 0);
    analogWrite(rpwmPin, speed);
  }
  else if (speed < 0)
  {
    analogWrite(rpwmPin, 0);
    analogWrite(lpwmPin, -speed);
  }
  else
  {
    analogWrite(rpwmPin, 0);
    analogWrite(lpwmPin, 0);
  }
}
