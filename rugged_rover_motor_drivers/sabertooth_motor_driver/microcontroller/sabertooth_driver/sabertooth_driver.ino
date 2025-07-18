#include <QuickPID.h>

#define SABERTOOTH_ADDR 128

#define ENCODER_FRONT_LEFT_A 2
#define ENCODER_FRONT_LEFT_B 3
#define ENCODER_FRONT_RIGHT_A 20
#define ENCODER_FRONT_RIGHT_B 21

#define BATTERY_VOLTAGE_PIN A0

#define SCALE 1000.0
#define START_BYTE 0xFF

bool debugging = true;

// Packet types
const byte PACKET_TYPE_VELOCITY = 0x02;
const byte VELOCITY_PACKET_LENGTH = 0x08;

const byte PACKET_TYPE_FEEDBACK_REQUEST = 0x03;
const byte FEEDBACK_REQUEST_LENGTH = 0x04;
const byte PACKET_TYPE_FEEDBACK_RESPONSE = 0x04;
const byte FEEDBACK_RESPONSE_LENGTH = 0x0E;

const byte PACKET_TYPE_VOLTAGE_FEEDBACK_REQUEST = 0x05;
const byte VOLTAGE_FEEDBACK_LENGTH = 0x04;
const byte PACKET_TYPE_VOLTAGE_FEEDBACK_RESPONSE = 0x06;
const byte VOLTAGE_FEEDBACK_RESPONSE_LENGTH = 0x06;

const byte MAX_PACKET_SIZE = 16;

const float PULSES_PER_REVOLUTION = 12.0;
const float GEAR_RATIO_MULTIPLIER = 27.0;

// Encoder state
volatile long encoderFrontLeft = 0;
volatile long encoderFrontRight = 0;

// Serial packet buffer
byte packetBuffer[MAX_PACKET_SIZE];
byte packetLength = 0;
byte bufferIndex = 0;
bool receivingPacket = false;

// Timing
unsigned long lastEncoderSampleTime = 0;
long lastFrontLeftTicks = 0;
long lastFrontRightTicks = 0;

// Control variables
float frontLeftRadSec = 0;
float frontRightRadSec = 0;
float setpointVelocityFrontLeft = 0;
float setpointVelocityFrontRight = 0;
float outputFrontLeft = 0;



QuickPID frontLeftPID(&frontLeftRadSec, &outputFrontLeft, &setpointVelocityFrontLeft);

void setup() {
  Serial.begin(115200);     // Host communication
  Serial1.begin(9600);      // Sabertooth serial

  pinMode(ENCODER_FRONT_LEFT_A, INPUT);
  pinMode(ENCODER_FRONT_LEFT_B, INPUT);
  pinMode(ENCODER_FRONT_RIGHT_A, INPUT);
  pinMode(ENCODER_FRONT_RIGHT_B, INPUT);
  pinMode(BATTERY_VOLTAGE_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENCODER_FRONT_LEFT_A), onFrontLeftEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_FRONT_RIGHT_A), onFrontRightEncoder, RISING);

  frontLeftPID.SetTunings(3.0, 0.7, 0.1);
  frontLeftPID.SetOutputLimits(-127, 127);
  frontLeftPID.SetMode(QuickPID::Control::automatic);
  frontLeftPID.SetSampleTimeUs(50000);
  frontLeftPID.SetDerivativeMode(QuickPID::dMode::dOnMeas);
}

void loop() {
  readSerial();

  unsigned long now = millis();
  if (now - lastEncoderSampleTime >= 50) {
    sampleEncoders();
    frontLeftPID.Compute();
    updateMotors();
    lastEncoderSampleTime = now;
  }
}

void readSerial() {
  while (Serial.available()) {
    byte b = Serial.read();

    if (!receivingPacket) {
      if (b == START_BYTE) {
        bufferIndex = 0;
        packetBuffer[bufferIndex++] = b;
        receivingPacket = true;
      }
    } else {
      packetBuffer[bufferIndex++] = b;

      if (bufferIndex == 2) {
        packetLength = packetBuffer[1];
        if (packetLength > MAX_PACKET_SIZE || packetLength < 4) {
          resetPacket();
          return;
        }
      }

      if (bufferIndex == packetLength && receivingPacket) {
        receivingPacket = false;
        parsePacket(packetBuffer, packetLength);
        resetPacket();
      }
    }
  }
}

void resetPacket() {
  receivingPacket = false;
  bufferIndex = 0;
  packetLength = 0;
}

void parsePacket(byte* packet, byte length) {
  if (length < 4 || packet[0] != START_BYTE) return;

  byte checksum = 0;
  for (int i = 0; i < length - 1; ++i) checksum ^= packet[i];
  if (checksum != packet[length - 1]) return;

  byte type = packet[2];
  switch (type) {
    case PACKET_TYPE_FEEDBACK_REQUEST:
      if (length == FEEDBACK_REQUEST_LENGTH) sendFeedbackResponse();
      break;
    case PACKET_TYPE_VELOCITY:
      if (length != VELOCITY_PACKET_LENGTH) return;

      int16_t velocityFrontLeftRaw = packet[3] | (packet[4] << 8);
      int16_t velocityFrontRightRaw = packet[5] | (packet[6] << 8);

      setpointVelocityFrontLeft = velocityFrontLeftRaw / SCALE;
      setpointVelocityFrontRight = velocityFrontRightRaw / SCALE;
      break;
  }
}

void updateMotors() {
  if (abs(outputFrontLeft) < 20 && abs(setpointVelocityFrontLeft) > 0.5) {
    outputFrontLeft = (outputFrontLeft > 0) ? 20 : -20;
  }

  sendMotorCommand(Serial1, SABERTOOTH_ADDR, 1, 0); // motor 1 disabled
  sendMotorCommand(Serial1, SABERTOOTH_ADDR, 2, (int)outputFrontLeft);
}

void sendMotorCommand(HardwareSerial& port, byte address, byte motor, int speed) {
  speed = constrain(speed, -127, 127);
  byte command = 0;

  if (motor == 1)
    command = (speed < 0) ? 1 : 0;
  else
    command = (speed < 0) ? 5 : 4;

  byte data = abs(speed);
  byte checksum = (address + command + data) & 0x7F;

  byte packet[] = { address, command, data, checksum };
  port.write(packet, sizeof(packet));
}

void onFrontLeftEncoder() {
  encoderFrontLeft += (digitalRead(ENCODER_FRONT_LEFT_B) ? 1 : -1);
}

void onFrontRightEncoder() {
  encoderFrontRight += (digitalRead(ENCODER_FRONT_RIGHT_B) ? 1 : -1);
}

void sampleEncoders() {
  noInterrupts();
  long leftEncoderNow = encoderFrontLeft;
  long rightEncoderNow = encoderFrontRight;
  interrupts();

  float intervalSec = 0.05;
  long deltaLeft = leftEncoderNow - lastFrontLeftTicks;
  long deltaRight = rightEncoderNow - lastFrontRightTicks;

  frontLeftRadSec = (((deltaLeft / intervalSec) / (PULSES_PER_REVOLUTION * GEAR_RATIO_MULTIPLIER)) * TWO_PI) * -1;
  frontRightRadSec = ((deltaRight / intervalSec) / (PULSES_PER_REVOLUTION * GEAR_RATIO_MULTIPLIER)) * TWO_PI;

  lastFrontLeftTicks = leftEncoderNow;
  lastFrontRightTicks = rightEncoderNow;


}

void sendFeedbackResponse() {
  byte response[FEEDBACK_RESPONSE_LENGTH] = {
    START_BYTE,
    FEEDBACK_RESPONSE_LENGTH,
    PACKET_TYPE_FEEDBACK_RESPONSE
  };

  int16_t leftVelRaw = static_cast<int16_t>(frontLeftRadSec * SCALE);
  int16_t rightVelRaw = static_cast<int16_t>(frontRightRadSec * SCALE);

  int16_t leftPosRaw = 0;   // Placeholder for position
  int16_t rightPosRaw = 0;

  int16_t voltageRaw = analogRead(BATTERY_VOLTAGE_PIN);
  int16_t voltageMv = map(voltageRaw, 0, 1023, 0, 5000);

  response[3] = leftVelRaw & 0xFF;
  response[4] = (leftVelRaw >> 8) & 0xFF;
  response[5] = rightVelRaw & 0xFF;
  response[6] = (rightVelRaw >> 8) & 0xFF;

    // Pack positions
  response[7]  = leftPosRaw & 0xFF;
  response[8]  = (leftPosRaw >> 8) & 0xFF;
  response[9]  = rightPosRaw & 0xFF;
  response[10] = (rightPosRaw >> 8) & 0xFF;

  // Pack battery voltage
  response[11] = voltageMv & 0xFF;
  response[12] = (voltageMv >> 8) & 0xFF;


  byte checksum = 0;
  for (int i = 0; i < FEEDBACK_RESPONSE_LENGTH - 1; ++i)
    checksum ^= response[i];

  response[13] = checksum;
  Serial.write(response, FEEDBACK_RESPONSE_LENGTH);
  Serial.flush();
}

