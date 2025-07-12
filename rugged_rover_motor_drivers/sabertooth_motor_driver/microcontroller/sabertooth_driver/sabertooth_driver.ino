#include <PID_v1.h>

#define SABERTOOTH_ADDR 128

#define ENCODER_FRONT_LEFT_A 2
#define ENCODER_FRONT_LEFT_B 3
#define ENCODER_FRONT_RIGHT_A 20
#define ENCODER_FRONT_RIGHT_B 21

#define SABERTOOTH_UPDATE_RATE_MS 50

volatile long encoderFrontLeft = 0;
volatile long encoderFrontRight = 0;

int targetFrontLeft = 0;
int targetSpeedFrontRight = 0;
int targetSpeedFrontLeft = 0;

const byte START_BYTE = 0xFF;
const byte PACKET_TYPE_VELOCITY = 0x02;
const byte MAX_PACKET_SIZE = 16;
byte packetBuffer[MAX_PACKET_SIZE];
byte packetLength = 0;
byte bufferIndex = 0;
bool receivingPacket = false;

const float PULSES_PER_REVOLUTION = 12;
const float GEAR_RATIO_MULTIPLIER = 27;

const float SCALE = 1000.0; 

unsigned long lastEncoderSampleTime = 0;
long lastFrontLeftTicks = 0;
long lastFrontRightTicks = 0;
double frontLeftRadSec = 0;
float frontRightRadSec = 0;

double setpointFrontLeft = 0;

double outputFrontLeft = 0;

double setpointVelocityFrontLeft = 0;
double setpointVelocityFrontRight = 0;

PID pidFrontLeft(&frontLeftRadSec, &outputFrontLeft, &setpointVelocityFrontLeft, 6.0, 0.7, 0.0, DIRECT);

void setup() {

  // Serial comms to raspberry pi  
  Serial.begin(115200);

  // Serial comms to sabertooth
  Serial1.begin(9600);

  // Set encoder pins
  pinMode(ENCODER_FRONT_LEFT_A, INPUT);
  pinMode(ENCODER_FRONT_LEFT_B, INPUT);
  pinMode(ENCODER_FRONT_RIGHT_A, INPUT);
  pinMode(ENCODER_FRONT_RIGHT_B, INPUT);

  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(ENCODER_FRONT_LEFT_A), onFrontLeftEncoder, RISING );
  attachInterrupt(digitalPinToInterrupt(ENCODER_FRONT_RIGHT_A), onFrontRightEncoder, RISING );

  pidFrontLeft.SetMode(AUTOMATIC);
  pidFrontLeft.SetOutputLimits(-127, 127);
}

void loop() {
  readSerial();
  sampleEncoders();

}

void readSerial() {
  while(Serial.available()) {
    byte b = Serial.read();
    if(!receivingPacket)
    {
      if(b == START_BYTE)
      {
        bufferIndex = 0;
        packetBuffer[bufferIndex++] = b;
        receivingPacket = true;
      }
    } else {
      packetBuffer[bufferIndex++] = b;

      if (bufferIndex == 2)
      {
        packetLength = packetBuffer[1];
        if(packetLength > MAX_PACKET_SIZE || packetLength < 4)
        {
          receivingPacket = false;
          bufferIndex = 0;
          packetLength = 0;
          memset(packetBuffer, 0 , MAX_PACKET_SIZE);
        }
      }
      if (bufferIndex == packetLength && receivingPacket) {
        receivingPacket = false;
        parsePacket(packetBuffer, packetLength);      
      }
    }
  }
}

void parsePacket(byte* packet, byte length) {
  if (length < 4 || packet[0] != START_BYTE) return;

  // Calculate checksum
  byte checksum = 0;
  for (int i = 0; i < length - 1; ++i) {
    checksum ^= packet[i];
  }

  if (checksum != packet[length - 1]) {
    Serial.println("#ERR: Bad checksum");
    return;
  }

  byte type = packet[2];
  Serial.write(packet, sizeof(packet));
  switch (type) {
    case PACKET_TYPE_VELOCITY:
      if (length != 8) return; 
      int16_t velocityFrontLeftRaw = packet[3] | (packet[4] << 8);
      int16_t velocityFrontRightRaw = packet[5] | (packet[6] << 8);

      setpointVelocityFrontLeft = velocityFrontLeftRaw / SCALE;
      setpointVelocityFrontRight = velocityFrontRightRaw / SCALE;
      break;

    // case 0x02: // extend here for other packet types
  }
}

void updateMotors() {

    sendMotorCommand(Serial1, SABERTOOTH_ADDR, 1, targetSpeedFrontRight);
    sendMotorCommand(Serial1, SABERTOOTH_ADDR, 2, (int)outputFrontLeft);
}

void sendMotorCommand(HardwareSerial &port, byte address, byte motor, int speed) {

  // Constrain to Sabertooth valid range
  speed = constrain(speed, -127, 127);

  // Set motor direction
  byte command;
  if(motor == 1)
  {
    // If speed less than 0 set motor 1 command byte to 0x01
    command = (speed < 0) ? 1 : 0;
  } else {
    // If speed less than 0 set motor 2 command byte to 0x04 (Right motor is reversed)
    command = (speed < 0) ? 4 : 5;
  }

  // Get magnitude of speed
  byte data = abs(speed);

  // Calculate checksum as per Sabertooth protocol
  byte checksum = (address + command + data) & 0x7F;

  // Create packet
  byte packet[] = {address, command, data, checksum};

  //Write packet
  port.write(packet, sizeof(packet));
}

void onFrontLeftEncoder() {
  if( digitalRead( ENCODER_FRONT_LEFT_B ) ) {
    encoderFrontLeft++;
  } else {
    encoderFrontLeft--;
  }
}
void onFrontRightEncoder() {
  if( digitalRead( ENCODER_FRONT_RIGHT_B ) ) {
    encoderFrontRight++;
  } else {
    encoderFrontRight--;
  }
}

void sampleEncoders() {
  unsigned long now = millis();

  // Sample encoders every 100ms
  if(now - lastEncoderSampleTime >= 100) {

    // Disable interrupts to safely read encoder values
    noInterrupts();
    long leftEncoderNow = encoderFrontLeft;
    long rightEncoderNow = encoderFrontRight;
    interrupts();

    // Calculate the time since the last sample
    float intervalSec = (now -lastEncoderSampleTime) / 1000.0;
    long deltaFrontLeft  = leftEncoderNow - lastFrontLeftTicks;
    long deltaFrontRight = rightEncoderNow - lastFrontRightTicks;

    // Avoid division by zero
    if (intervalSec <= 0.0) {
      return;
    }
    // Calculate angular velocity in radians per second
    frontLeftRadSec = ((deltaFrontLeft / intervalSec) / (PULSES_PER_REVOLUTION * GEAR_RATIO_MULTIPLIER)) * TWO_PI;
    frontRightRadSec = (((deltaFrontRight / intervalSec) / (PULSES_PER_REVOLUTION * GEAR_RATIO_MULTIPLIER)) * TWO_PI); // Right motor is reversed

    lastFrontLeftTicks = leftEncoderNow;
    lastFrontRightTicks = rightEncoderNow;
    lastEncoderSampleTime = now;

    pidFrontLeft.Compute();
    updateMotors();

  }
}