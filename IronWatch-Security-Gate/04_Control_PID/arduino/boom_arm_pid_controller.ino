/* =====================================================================
   IronWatch Security Gate - Boom Arm PID Position Controller
   =====================================================================
   Closes the position loop on the motorised swing-arm boom barrier.
   The arm rotates ~90 deg between CLOSED (0 deg) and OPEN (90 deg).

   Hardware (reference wiring - adjust pins to your board):
     A0            : potentiometer / encoder-derived angle feedback
     D5, D6        : motor driver PWM inputs (e.g. L298N IN1/IN2 or a
                     PWM + DIR H-bridge). Here: PWM magnitude + direction.
     D3            : direction pin (HIGH = opening, LOW = closing)
     Trigger input : set targetAngle from the access-control logic
                     (face/RFID authorised -> command OPEN; then auto-close)

   Control law:  u = Kp*e + Ki*integral(e) + Kd*d(measurement)/dt
   Gains from the dynamic model (boom_arm_pid.py / .m):
     Kp = 70, Ki = 60, Kd = 4   (scaled below for an 8-bit PWM actuator)

   NOTE: The model gains are in SI (volts per radian). On the Arduino the
   loop runs on ANGLE IN DEGREES and outputs 0..255 PWM, so the gains are
   rescaled. Re-tune on real hardware; these are validated starting points.
   ===================================================================== */

// ---------- Pin map ----------
const int PIN_FEEDBACK = A0;   // angle sensor (pot or encoder-conditioned)
const int PIN_PWM      = 5;    // motor speed (PWM)
const int PIN_DIR      = 3;    // motor direction

// ---------- Angle calibration ----------
// Map raw ADC (0..1023) to arm angle in degrees (0..90).
const float ADC_AT_CLOSED = 0.0;     // ADC reading at 0 deg
const float ADC_AT_OPEN   = 1023.0;  // ADC reading at 90 deg

// ---------- PID gains (rescaled for degrees -> 0..255 PWM) ----------
float Kp = 6.0;      // ~ model 70 scaled for degree error & PWM span
float Ki = 4.0;
float Kd = 0.8;

// ---------- Motion setpoints ----------
const float ANGLE_CLOSED = 0.0;
const float ANGLE_OPEN   = 90.0;
float targetAngle = ANGLE_CLOSED;    // commanded by access logic

// ---------- Loop timing ----------
const unsigned long DT_MS = 10;      // 100 Hz control loop
unsigned long lastLoop = 0;

// ---------- PID state ----------
float integral = 0.0;
float prevMeas = 0.0;
const float INTEGRAL_LIMIT = 200.0;  // anti-windup clamp
const int   PWM_MAX = 255;
const int   PWM_MIN = 0;

// ---------- Auto-close timer ----------
unsigned long openedAt = 0;
const unsigned long HOLD_MS = 4000;  // stay open 4 s then auto-close
bool isOpenCommand = false;

float readAngle() {
  int raw = analogRead(PIN_FEEDBACK);
  float a = (raw - ADC_AT_CLOSED) * (ANGLE_OPEN - ANGLE_CLOSED)
            / (ADC_AT_OPEN - ADC_AT_CLOSED);
  return a;
}

void driveMotor(float u) {
  // u is signed control effort; sign -> direction, magnitude -> PWM
  bool opening = (u >= 0);
  digitalWrite(PIN_DIR, opening ? HIGH : LOW);
  int pwm = (int)constrain(fabs(u), PWM_MIN, PWM_MAX);
  analogWrite(PIN_PWM, pwm);
}

void commandOpen()  { targetAngle = ANGLE_OPEN;  isOpenCommand = true;  openedAt = millis(); }
void commandClose() { targetAngle = ANGLE_CLOSED; isOpenCommand = false; }

void setup() {
  Serial.begin(9600);
  pinMode(PIN_PWM, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  prevMeas = readAngle();
  Serial.println(F("IronWatch boom-arm PID controller online."));
  Serial.println(F("Send 'O' to open, 'C' to close."));
}

void loop() {
  // --- accept open/close commands (from access-control logic or serial) ---
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'O' || c == 'o') commandOpen();
    if (c == 'C' || c == 'c') commandClose();
  }

  // --- auto-close after hold time ---
  if (isOpenCommand && (millis() - openedAt > HOLD_MS)) commandClose();

  // --- fixed-rate PID update ---
  unsigned long now = millis();
  if (now - lastLoop < DT_MS) return;
  float dt = (now - lastLoop) / 1000.0;
  lastLoop = now;

  float meas = readAngle();
  float error = targetAngle - meas;

  // integral with anti-windup clamp
  integral += error * dt;
  integral = constrain(integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

  // derivative on measurement (no setpoint kick)
  float dMeas = (meas - prevMeas) / dt;
  prevMeas = meas;

  float u = Kp * error + Ki * integral - Kd * dMeas;

  // saturation + conditional integrator freeze (simple anti-windup)
  float u_sat = constrain(u, -PWM_MAX, PWM_MAX);
  if (u != u_sat) integral -= error * dt;

  driveMotor(u_sat);

  // telemetry (throttled)
  static unsigned long lastPrint = 0;
  if (now - lastPrint > 200) {
    lastPrint = now;
    Serial.print(F("target=")); Serial.print(targetAngle, 1);
    Serial.print(F(" angle="));  Serial.print(meas, 1);
    Serial.print(F(" u="));      Serial.println(u_sat, 1);
  }
}
