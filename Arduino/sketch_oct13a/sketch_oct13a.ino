#include <SoftwareSerial.h>

#define WHEEL_DIAMETER_METERS 0.127 //diameter of the wheels
#define TICKS_PER_REVOLUTION 336.0 //7 PPR Encoder * 48:1 Gearbox

// Pre-calculate for efficiency
const float WHEEL_CIRCUMFERENCE = PI * WHEEL_DIAMETER_METERS;

// --- Pin Definitions ---
#define SABERTOOTH_TX_PIN 13
SoftwareSerial SabertoothSerial(10, SABERTOOTH_TX_PIN);

#define LEFT_ENCODER_A_PIN 2
#define LEFT_ENCODER_B_PIN 4
#define RIGHT_ENCODER_A_PIN 3
#define RIGHT_ENCODER_B_PIN 5

// --- Global Variables ---
volatile long leftEncoderTicks = 0;
volatile long rightEncoderTicks = 0;

// Variables for velocity calculation
long prevLeftEncoderTicks = 0;
long prevRightEncoderTicks = 0;

// --- Communication & Timing ---
const unsigned long ENCODER_REPORT_INTERVAL_MS = 50; // 20 Hz
const float ENCODER_REPORT_INTERVAL_S = ENCODER_REPORT_INTERVAL_MS / 1000.0; // Interval in seconds
unsigned long lastReportTime = 0;

void setup() {
  Serial.begin(115200);
  SabertoothSerial.begin(9600);

  pinMode(LEFT_ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(LEFT_ENCODER_B_PIN, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_B_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_A_PIN), readLeftEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_A_PIN), readRightEncoder, RISING);
  
  Serial.println("Arduino ready. Reporting velocity in m/s.");
}

void loop() {
  // Check for motor commands from Serial Monitor/Pi
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    parseCommand(command);
  }

  // Periodically calculate and report velocity
  if (millis() - lastReportTime >= ENCODER_REPORT_INTERVAL_MS) {
    calculateAndReportVelocity();
    lastReportTime = millis();
  }
}

void calculateAndReportVelocity() {
  // Create local copies of the volatile encoder counts for safe calculation
  long currentLeftTicks = leftEncoderTicks;
  long currentRightTicks = rightEncoderTicks;

  // 1. Calculate ticks since the last report
  long deltaLeftTicks = currentLeftTicks - prevLeftEncoderTicks;
  long deltaRightTicks = currentRightTicks - prevRightEncoderTicks;

  // 2. Update the previous tick values for the next calculation
  prevLeftEncoderTicks = currentLeftTicks;
  prevRightEncoderTicks = currentRightTicks;

  // 3. Calculate velocity for each wheel
  float left_revolutions = (float)deltaLeftTicks / TICKS_PER_REVOLUTION;
  float left_distance = left_revolutions * WHEEL_CIRCUMFERENCE;
  float left_velocity_mps = left_distance / ENCODER_REPORT_INTERVAL_S; // m/s

  float right_revolutions = (float)deltaRightTicks / TICKS_PER_REVOLUTION;
  float right_distance = right_revolutions * WHEEL_CIRCUMFERENCE;
  float right_velocity_mps = right_distance / ENCODER_REPORT_INTERVAL_S; // m/s
  
  // 4. Format and send the report string
  // Format: V<left_velocity>,<right_velocity>
  // The '4' specifies printing the float with 4 decimal places
  String report = "V" + String(left_velocity_mps, 4) + "," + String(right_velocity_mps, 4);
  Serial.println(report);
}

void parseCommand(String cmd) {
  // Trim whitespace just in case
  cmd.trim();

  int lPos = cmd.indexOf('L');
  int rPos = cmd.indexOf('R');
  int commaPos = cmd.indexOf(',');

  if (lPos != -1 && rPos != -1 && commaPos != -1) {
    String lSpeedStr = cmd.substring(lPos + 1, commaPos);
    String rSpeedStr = cmd.substring(rPos + 1);

    int left_val = lSpeedStr.toInt();
    int right_val = rSpeedStr.toInt();

    // Call the motor function with the parsed values
    driveMotors(left_val, right_val);
    
  } else {
    // Print an error if the format is wrong
    Serial.print("Command parsing FAILED for: '");
    Serial.print(cmd);
    Serial.println("'");
  }
}

// --- ISRs and Motor Control (Unchanged) ---
void readLeftEncoder() {
  if (digitalRead(LEFT_ENCODER_B_PIN) == HIGH) leftEncoderTicks++; else leftEncoderTicks--;
}
void readRightEncoder() {
  if (digitalRead(RIGHT_ENCODER_B_PIN) == HIGH) rightEncoderTicks--; else rightEncoderTicks++;
}
void driveMotors(int left_val, int right_val) {
  // 1. Constrain values just in case
  left_val = constrain(left_val, -127, 127);
  right_val = constrain(right_val, -127, 127);

  // 2. Map the ROS range [-127, 127] to the Sabertooth's ranges
  // Motor 1 (Left):   -127 to 127  ->  1 (full reverse) to 127 (full forward)
  // Motor 2 (Right):  -127 to 127  ->  128 (full reverse) to 255 (full forward)
  
  byte saber_left_cmd = map(left_val, -127, 127, 1, 127);
  byte saber_right_cmd = map(right_val, -127, 127, 128, 255);

  // 3. Send the commands to the Sabertooth driver
  SabertoothSerial.write(saber_left_cmd);
  SabertoothSerial.write(saber_right_cmd);

  // 4. (Optional) Print final commands to the Serial Monitor for debugging
  Serial.print("Sent -> Saber Left: ");
  Serial.print(saber_left_cmd);
  Serial.print(", Saber Right: ");
  Serial.println(saber_right_cmd);
}