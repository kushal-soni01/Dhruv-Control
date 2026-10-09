#include <ArduinoBLE.h>

// ============================================================
// XTRIVE — 4-WHEEL HOLONOMIC / H-DRIVE MOTOR & LED CONTROLLER
// WITH JETSON NANO COMMAND PARSER & BLE TELEMETRY
//
// WHEEL LAYOUT
//
//                    FRONT
//                      ↑
//
//             M1                 M4
//            FRONT-L           FRONT-R
//               \               /
//                \             /
//
//                /             \
//               /               \
//             M2                 M3
//            REAR-L            REAR-R
//
//                      ↓
//                     REAR
//
// Roller orientation:
//     M1 = "\"       M4 = "/"
//     M2 = "/"       M3 = "\"
//
// M1 and M3 = same roller orientation
// M2 and M4 = opposite roller orientation
//
// ============================================================
//
// BLE / SERIAL COMMANDS
//
// N   = Forward
// S   = Backward
// W   = Left
// E   = Right
//
// NE  = Forward-Right
// NW  = Forward-Left
// SE  = Backward-Right
// SW  = Backward-Left
//
// CW  = Clockwise rotation
// CCW = Counter-clockwise rotation
//
// Q   = Stop
//
// ============================================================
//
// MOTOR CALIBRATION
//
// T1_150              -> M1 forward at 150
// t1_150              -> M1 reverse at 150
//
// T1_150 T4_150       -> M1 and M4 forward
// T1_150 T4_150 t2_150
//
// Motors not mentioned in a calibration line are stopped.
//
// X / Q -> stop everything
// ============================================================


// ============================================================
// BLE
// ============================================================

BLEService motorService(
  "12345678-1234-1234-1234-1234567890AB"
);

BLEStringCharacteristic cmdChar(
  "12345678-1234-1234-1234-1234567890CD",
  BLEWrite | BLEWriteWithoutResponse,
  40
);

BLEStringCharacteristic logChar(
  "12345678-1234-1234-1234-1234567890EF",
  BLERead | BLENotify,
  100
);

bool bleCentralConnected = false;


// ============================================================
// BLE / SERIAL LOGGING
// ============================================================

void bleLog(String msg)
{
  Serial.println(msg);

  if (bleCentralConnected)
  {
    if (msg.length() > 99)
      msg = msg.substring(0, 99);

    logChar.writeValue(msg);
  }
}


// ============================================================
// MOTOR PINS
//
// REN and LEN are connected to VCC.
//
// Therefore Arduino only controls:
// RPWM
// LPWM
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

#define M3_RPWM 7
#define M3_LPWM 6

#define M4_RPWM 9
#define M4_LPWM 8


// ============================================================
// SETTINGS
// ============================================================

const int SPEED_LIMIT = 180;


// ============================================================
// MOTOR DIRECTION CORRECTION
//
// Start with all FALSE.
//
// If a motor rotates opposite to the required direction,
// change only that motor to TRUE.
// ============================================================

bool M1_INVERT = false;
bool M2_INVERT = false;
bool M3_INVERT = false;
bool M4_INVERT = false;


const bool DEBUG = true;


// ============================================================
// STATE
// ============================================================

bool testModeActive = false;


// ============================================================
// STATUS LED / MOSFET
// ============================================================

const int mosfetPin = 13;

const int blinkInterval = 50;
const int fadeInterval = 8;

const unsigned long holdDuration = 2000;
const unsigned long startupBlinkDuration = 5000;

unsigned long ledPreviousMillis = 0;
unsigned long ledPhaseStartMillis = 0;

int ledBrightness = 0;
bool ledState = LOW;


enum SequenceState
{
  STARTUP_BLINK,
  FADE_IN,
  HOLD_ON,
  FADE_OUT
};


SequenceState currentLedState = STARTUP_BLINK;


// ============================================================
// FUNCTION DECLARATIONS
// ============================================================

void handleCommand(String command);

void mixDrive(
  int Vx,
  int Vy,
  int W
);

void moveForward();
void moveBackward();

void strafeLeft();
void strafeRight();

void moveForwardRight();
void moveForwardLeft();

void moveBackwardRight();
void moveBackwardLeft();

void rotateCW();
void rotateCCW();

void stopAll();

void setMotor(
  int motor,
  int speed
);

void setRawMotor(
  int motor,
  int speed
);

void handleSerialLine(
  String line
);

void runMotorTokens(
  String line
);

bool parseMotorToken(
  String token,
  int &motor,
  int &speed,
  bool &reverse
);

void updateStatusLED();


// ============================================================
// SETUP
// ============================================================

void setup()
{
  Serial.begin(115200);


  // ==========================================================
  // MOTOR PWM PINS
  // ==========================================================

  pinMode(M1_RPWM, OUTPUT);
  pinMode(M1_LPWM, OUTPUT);

  pinMode(M2_RPWM, OUTPUT);
  pinMode(M2_LPWM, OUTPUT);

  pinMode(M3_RPWM, OUTPUT);
  pinMode(M3_LPWM, OUTPUT);

  pinMode(M4_RPWM, OUTPUT);
  pinMode(M4_LPWM, OUTPUT);


  // Always start with motors stopped
  stopAll();


  // ==========================================================
  // STATUS MOSFET
  // ==========================================================

  pinMode(mosfetPin, OUTPUT);

  ledPhaseStartMillis = millis();


  // ==========================================================
  // BLE INITIALIZATION
  // ==========================================================

  if (!BLE.begin())
  {
    Serial.println(
      "ERROR: BLE failed to start!"
    );

    while (1)
    {
      stopAll();
      delay(100);
    }
  }


  BLE.setLocalName(
    "UNO_R4_Robot"
  );


  BLE.setAdvertisedService(
    motorService
  );


  motorService.addCharacteristic(
    cmdChar
  );


  motorService.addCharacteristic(
    logChar
  );


  BLE.addService(
    motorService
  );


  BLE.advertise();


  // ==========================================================
  // STARTUP INFORMATION
  // ==========================================================

  Serial.println();

  Serial.println(
    "=========================================="
  );

  Serial.println(
    "   XTRIVE BLE HOLONOMIC CONTROLLER"
  );

  Serial.println(
    "=========================================="
  );


  Serial.println();

  Serial.println(
    "MOTOR MAPPING:"
  );

  Serial.println(
    "  M1 = Front Left"
  );

  Serial.println(
    "  M2 = Rear Left"
  );

  Serial.println(
    "  M3 = Rear Right"
  );

  Serial.println(
    "  M4 = Front Right"
  );


  Serial.println();

  Serial.println(
    "MOVEMENT COMMANDS:"
  );

  Serial.println(
    "  N  = Forward"
  );

  Serial.println(
    "  S  = Backward"
  );

  Serial.println(
    "  W  = Left"
  );

  Serial.println(
    "  E  = Right"
  );

  Serial.println(
    "  NE = Forward-Right"
  );

  Serial.println(
    "  NW = Forward-Left"
  );

  Serial.println(
    "  SE = Backward-Right"
  );

  Serial.println(
    "  SW = Backward-Left"
  );

  Serial.println(
    "  CW = Clockwise"
  );

  Serial.println(
    "  CCW = Counter-clockwise"
  );

  Serial.println(
    "  Q  = Stop"
  );


  Serial.println();

  Serial.println(
    "MOTOR CALIBRATION:"
  );

  Serial.println(
    "  T1_150"
  );

  Serial.println(
    "  t1_150"
  );

  Serial.println(
    "  T1_150 T4_150"
  );

  Serial.println(
    "  X = Stop"
  );


  Serial.println();

  Serial.println(
    "=========================================="
  );
}


// ============================================================
// MAIN LOOP
// ============================================================

void loop()
{
  updateStatusLED();


  // ==========================================================
  // SERIAL INPUT
  // ==========================================================

  if (Serial.available())
  {
    String line =
      Serial.readStringUntil('\n');

    line.trim();

    if (line.length() > 0)
    {
      handleSerialLine(line);
    }
  }


  // ==========================================================
  // BLE
  // ==========================================================

  BLEDevice central =
    BLE.central();


  if (central)
  {
    bleCentralConnected = true;


    Serial.print(
      "Connected to: "
    );

    Serial.println(
      central.address()
    );


    bleLog(
      "Connected: " +
      central.address()
    );


    while (central.connected())
    {
      updateStatusLED();


      // ------------------------------------------------------
      // BLE COMMAND
      // ------------------------------------------------------

      if (cmdChar.written())
      {
        String value =
          cmdChar.value();

        value.trim();


        if (value.length() > 0)
        {
          handleSerialLine(value);
        }
      }


      // ------------------------------------------------------
      // SERIAL WHILE BLE CONNECTED
      // ------------------------------------------------------

      if (Serial.available())
      {
        String line =
          Serial.readStringUntil('\n');

        line.trim();


        if (line.length() > 0)
        {
          handleSerialLine(line);
        }
      }


      BLE.poll();
    }


    // --------------------------------------------------------
    // BLE DISCONNECT SAFETY
    // --------------------------------------------------------

    stopAll();

    bleCentralConnected = false;


    Serial.println(
      "BLE disconnected. Motors stopped."
    );
  }
}


// ============================================================
// SERIAL / BLE COMMAND ROUTER
// ============================================================

void handleSerialLine(String line)
{
  line.trim();


  if (line.length() == 0)
    return;


  // ==========================================================
  // MOTOR CALIBRATION
  //
  // Must be checked BEFORE converting to uppercase because
  // lowercase 't' means reverse.
  // ==========================================================

  char first =
    line.charAt(0);


  if (
    (first == 'T' || first == 't') &&
    line.indexOf('_') != -1
  )
  {
    testModeActive = true;

    runMotorTokens(line);

    return;
  }


  // ==========================================================
  // CONVERT NORMAL COMMAND TO UPPERCASE
  // ==========================================================

  line.toUpperCase();


  // ==========================================================
  // STOP
  // ==========================================================

  if (
    line == "Q" ||
    line == "X" ||
    line == "STOP"
  )
  {
    testModeActive = false;

    stopAll();

    bleLog(
      "CMD: STOP"
    );

    return;
  }


  // ==========================================================
  // MOVEMENT COMMAND
  // ==========================================================

  handleCommand(line);
}


// ============================================================
// COMMAND HANDLER
// ============================================================

void handleCommand(String command)
{
  testModeActive = false;


  // ==========================================================
  // FORWARD
  // ==========================================================

  if (command == "N")
  {
    moveForward();

    bleLog(
      "CMD: FORWARD"
    );

    return;
  }


  // ==========================================================
  // BACKWARD
  // ==========================================================

  if (command == "S")
  {
    moveBackward();

    bleLog(
      "CMD: BACKWARD"
    );

    return;
  }


  // ==========================================================
  // LEFT
  // ==========================================================

  if (command == "W")
  {
    strafeLeft();

    bleLog(
      "CMD: LEFT"
    );

    return;
  }


  // ==========================================================
  // RIGHT
  // ==========================================================

  if (command == "E")
  {
    strafeRight();

    bleLog(
      "CMD: RIGHT"
    );

    return;
  }


  // ==========================================================
  // FORWARD-RIGHT
  // ==========================================================

  if (command == "NE")
  {
    moveForwardRight();

    bleLog(
      "CMD: FORWARD-RIGHT"
    );

    return;
  }


  // ==========================================================
  // FORWARD-LEFT
  // ==========================================================

  if (command == "NW")
  {
    moveForwardLeft();

    bleLog(
      "CMD: FORWARD-LEFT"
    );

    return;
  }


  // ==========================================================
  // BACKWARD-RIGHT
  // ==========================================================

  if (command == "SE")
  {
    moveBackwardRight();

    bleLog(
      "CMD: BACKWARD-RIGHT"
    );

    return;
  }


  // ==========================================================
  // BACKWARD-LEFT
  // ==========================================================

  if (command == "SW")
  {
    moveBackwardLeft();

    bleLog(
      "CMD: BACKWARD-LEFT"
    );

    return;
  }


  // ==========================================================
  // CLOCKWISE
  // ==========================================================

  if (command == "CW")
  {
    rotateCW();

    bleLog(
      "CMD: ROTATE CW"
    );

    return;
  }


  // ==========================================================
  // COUNTER-CLOCKWISE
  // ==========================================================

  if (command == "CCW")
  {
    rotateCCW();

    bleLog(
      "CMD: ROTATE CCW"
    );

    return;
  }
}


// ============================================================
// BASIC MOVEMENTS
// ============================================================

void moveForward()
{
  mixDrive(
    SPEED_LIMIT,
    0,
    0
  );
}


void moveBackward()
{
  mixDrive(
    -SPEED_LIMIT,
    0,
    0
  );
}


void strafeLeft()
{
  mixDrive(
    0,
    SPEED_LIMIT,
    0
  );
}


void strafeRight()
{
  mixDrive(
    0,
    -SPEED_LIMIT,
    0
  );
}


// ============================================================
// DIAGONAL MOVEMENTS
// ============================================================

void moveForwardRight()
{
  mixDrive(
    SPEED_LIMIT,
    -SPEED_LIMIT,
    0
  );
}


void moveForwardLeft()
{
  mixDrive(
    SPEED_LIMIT,
    SPEED_LIMIT,
    0
  );
}


void moveBackwardRight()
{
  mixDrive(
    -SPEED_LIMIT,
    -SPEED_LIMIT,
    0
  );
}


void moveBackwardLeft()
{
  mixDrive(
    -SPEED_LIMIT,
    SPEED_LIMIT,
    0
  );
}


// ============================================================
// ROTATION
// ============================================================

void rotateCW()
{
  mixDrive(
    0,
    0,
    -SPEED_LIMIT
  );
}


void rotateCCW()
{
  mixDrive(
    0,
    0,
    SPEED_LIMIT
  );
}


// ============================================================
// STOP ALL MOTORS
// ============================================================

void stopAll()
{
  setMotor(1, 0);
  setMotor(2, 0);
  setMotor(3, 0);
  setMotor(4, 0);
}


// ============================================================
// HOLONOMIC MECANUM MIXING
//
// Coordinate system:
//
// Vx = forward/backward
// Vy = left/right
// W  = rotation
//
// +Vx = Forward
// -Vx = Backward
//
// +Vy = Left
// -Vy = Right
//
// +W = Counter-clockwise
// -W = Clockwise
//
// Wheel equations:
//
// M1 = Vx - Vy - W
// M4 = Vx + Vy + W
// M2 = Vx + Vy - W
// M3 = Vx - Vy + W
//
// ============================================================

void mixDrive(
  int Vx,
  int Vy,
  int W
)
{
  float M1;
  float M2;
  float M3;
  float M4;


  // ==========================================================
  // MECANUM MIXING
  // ==========================================================

  M1 =
    Vx -
    Vy -
    W;


  M4 =
    Vx +
    Vy +
    W;


  M2 =
    Vx +
    Vy -
    W;


  M3 =
    Vx -
    Vy +
    W;


  // ==========================================================
  // NORMALIZATION
  //
  // Combined movements can produce values greater than
  // SPEED_LIMIT.
  //
  // Scale all four motors proportionally.
  // ==========================================================

  float maxMag =
    max(
      max(
        fabs(M1),
        fabs(M2)
      ),
      max(
        fabs(M3),
        fabs(M4)
      )
    );


  if (
    maxMag >
    SPEED_LIMIT
  )
  {
    float scale =
      SPEED_LIMIT /
      maxMag;


    M1 *= scale;
    M2 *= scale;
    M3 *= scale;
    M4 *= scale;
  }


  // ==========================================================
  // SEND MOTOR COMMANDS
  // ==========================================================

  setMotor(
    1,
    (int)M1
  );


  setMotor(
    2,
    (int)M2
  );


  setMotor(
    3,
    (int)M3
  );


  setMotor(
    4,
    (int)M4
  );


  // ==========================================================
  // DEBUG
  // ==========================================================

  if (DEBUG)
  {
    String line =
      "MIX Vx=" +
      String(Vx) +

      " Vy=" +
      String(Vy) +

      " W=" +
      String(W) +

      " -> M1=" +
      String((int)M1) +

      " M2=" +
      String((int)M2) +

      " M3=" +
      String((int)M3) +

      " M4=" +
      String((int)M4);


    bleLog(line);
  }
}


// ============================================================
// MOTOR CALIBRATION TOKEN PARSER
//
// T1_150 = forward
// t1_150 = reverse
// ============================================================

bool parseMotorToken(
  String token,
  int &motor,
  int &speed,
  bool &reverse
)
{
  if (
    token.length() <
    4
  )
  {
    return false;
  }


  char dirChar =
    token.charAt(0);


  if (
    dirChar != 'T' &&
    dirChar != 't'
  )
  {
    return false;
  }


  reverse =
    (dirChar == 't');


  char motorChar =
    token.charAt(1);


  if (
    motorChar < '1' ||
    motorChar > '4'
  )
  {
    return false;
  }


  motor =
    motorChar - '0';


  if (
    token.charAt(2) != '_'
  )
  {
    return false;
  }


  String speedStr =
    token.substring(3);


  if (
    speedStr.length() == 0
  )
  {
    return false;
  }


  for (
    unsigned int i = 0;
    i < speedStr.length();
    i++
  )
  {
    if (
      !isDigit(
        speedStr.charAt(i)
      )
    )
    {
      return false;
    }
  }


  speed =
    speedStr.toInt();


  speed =
    constrain(
      speed,
      0,
      255
    );


  return true;
}


// ============================================================
// RUN MOTOR TEST TOKENS
// ============================================================

void runMotorTokens(
  String line
)
{
  bool used[5] =
  {
    false,
    false,
    false,
    false,
    false
  };


  int speed[5] =
  {
    0,
    0,
    0,
    0,
    0
  };


  bool rev[5] =
  {
    false,
    false,
    false,
    false,
    false
  };


  String summary = "";

  bool anyValid = false;


  int start = 0;


  while (
    start <
    (int)line.length()
  )
  {
    int spaceIdx =
      line.indexOf(
        ' ',
        start
      );


    String token;


    if (
      spaceIdx == -1
    )
    {
      token =
        line.substring(
          start
        );
    }
    else
    {
      token =
        line.substring(
          start,
          spaceIdx
        );
    }


    token.trim();


    if (
      token.length() > 0
    )
    {
      int m;
      int spd;
      bool rv;


      if (
        parseMotorToken(
          token,
          m,
          spd,
          rv
        )
      )
      {
        used[m] = true;

        speed[m] = spd;

        rev[m] = rv;

        anyValid = true;


        summary +=
          "M" +
          String(m) +
          "=" +
          (
            rv
            ? "-"
            : "+"
          ) +
          String(spd) +
          " ";
      }
      else
      {
        bleLog(
          "[TEST] Bad token: " +
          token
        );
      }
    }


    if (
      spaceIdx == -1
    )
    {
      break;
    }


    start =
      spaceIdx + 1;
  }


  if (!anyValid)
  {
    bleLog(
      "[TEST] No valid tokens."
    );

    return;
  }


  // ==========================================================
  // APPLY TEST COMMANDS
  //
  // Motors not mentioned are stopped.
  // ==========================================================

  for (
    int m = 1;
    m <= 4;
    m++
  )
  {
    int s;


    if (used[m])
    {
      s =
        rev[m]
        ? -speed[m]
        : speed[m];
    }
    else
    {
      s = 0;
    }


    setMotor(
      m,
      s
    );
  }


  bleLog(
    "[TEST] " +
    summary +
    "(others stopped)"
  );
}


// ============================================================
// MOTOR DIRECTION INVERSION
// ============================================================

void setMotor(
  int motor,
  int speed
)
{
  switch (motor)
  {
    case 1:

      if (M1_INVERT)
        speed = -speed;

      break;


    case 2:

      if (M2_INVERT)
        speed = -speed;

      break;


    case 3:

      if (M3_INVERT)
        speed = -speed;

      break;


    case 4:

      if (M4_INVERT)
        speed = -speed;

      break;


    default:
      return;
  }


  setRawMotor(
    motor,
    speed
  );
}


// ============================================================
// RAW IBT-2 / BTS7960 MOTOR CONTROL
//
// Positive:
//
//   RPWM = speed
//   LPWM = 0
//
// Negative:
//
//   RPWM = 0
//   LPWM = abs(speed)
//
// Zero:
//
//   RPWM = 0
//   LPWM = 0
// ============================================================

void setRawMotor(
  int motor,
  int speed
)
{
  int rpwmPin;
  int lpwmPin;


  switch (motor)
  {
    case 1:

      rpwmPin = M1_RPWM;
      lpwmPin = M1_LPWM;

      break;


    case 2:

      rpwmPin = M2_RPWM;
      lpwmPin = M2_LPWM;

      break;


    case 3:

      rpwmPin = M3_RPWM;
      lpwmPin = M3_LPWM;

      break;


    case 4:

      rpwmPin = M4_RPWM;
      lpwmPin = M4_LPWM;

      break;


    default:
      return;
  }


  speed =
    constrain(
      speed,
      -255,
      255
    );


  if (speed > 0)
  {
    analogWrite(
      lpwmPin,
      0
    );

    analogWrite(
      rpwmPin,
      speed
    );
  }

  else if (speed < 0)
  {
    analogWrite(
      rpwmPin,
      0
    );

    analogWrite(
      lpwmPin,
      -speed
    );
  }

  else
  {
    analogWrite(
      rpwmPin,
      0
    );

    analogWrite(
      lpwmPin,
      0
    );
  }
}


// ============================================================
// STATUS LED / MOSFET SEQUENCE
// ============================================================

void updateStatusLED()
{
  unsigned long currentMillis =
    millis();


  switch (currentLedState)
  {
    // --------------------------------------------------------
    // STARTUP BLINK
    // --------------------------------------------------------

    case STARTUP_BLINK:

      if (
        currentMillis -
        ledPhaseStartMillis >=
        startupBlinkDuration
      )
      {
        currentLedState =
          FADE_IN;

        ledBrightness = 0;
      }

      else if (
        currentMillis -
        ledPreviousMillis >=
        blinkInterval
      )
      {
        ledPreviousMillis =
          currentMillis;

        ledState =
          !ledState;

        digitalWrite(
          mosfetPin,
          ledState
        );
      }

      break;


    // --------------------------------------------------------
    // FADE IN
    // --------------------------------------------------------

    case FADE_IN:

      if (
        currentMillis -
        ledPreviousMillis >=
        fadeInterval
      )
      {
        ledPreviousMillis =
          currentMillis;

        ledBrightness++;


        analogWrite(
          mosfetPin,
          ledBrightness
        );


        if (
          ledBrightness >= 255
        )
        {
          currentLedState =
            HOLD_ON;

          ledPhaseStartMillis =
            currentMillis;
        }
      }

      break;


    // --------------------------------------------------------
    // HOLD ON
    // --------------------------------------------------------

    case HOLD_ON:

      if (
        currentMillis -
        ledPhaseStartMillis >=
        holdDuration
      )
      {
        currentLedState =
          FADE_OUT;
      }

      break;


    // --------------------------------------------------------
    // FADE OUT
    // --------------------------------------------------------

    case FADE_OUT:

      if (
        currentMillis -
        ledPreviousMillis >=
        fadeInterval
      )
      {
        ledPreviousMillis =
          currentMillis;

        ledBrightness--;


        analogWrite(
          mosfetPin,
          ledBrightness
        );


        if (
          ledBrightness <= 0
        )
        {
          currentLedState =
            FADE_IN;
        }
      }

      break;
  }
}