#include <QuickPID.h>

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
const byte VELOCITY_PACKET_LENGTH = 0x08;
const byte PACKET_TYPE_FEEDBACK_REQUEST = 0x03;
const byte FEEDBACK_REQUEST_LENGTH = 0x04;
const byte PACKET_TYPE_FEEDBACK_RESPONSE = 0x04;
const byte FEEDBACK_RESPONSE_LENGTH = 0x0C;

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
float frontLeftRadSec = 0;
float frontRightRadSec = 0;

double setpointFrontLeft = 0;

float outputFrontLeft = 0;

float setpointVelocityFrontLeft = 0;
double setpointVelocityFrontRight = 0;

float scaledFrontLeftRadSec = 0;
float scaledSetpointVelocityFrontLeft = 0;

QuickPID frontLeftPID(&frontLeftRadSec , &outputFrontLeft, &setpointVelocityFrontLeft);



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

  frontLeftPID.SetTunings(1.0, 0.0, 0.0);
  frontLeftPID.SetOutputLimits(-127, 127);
  frontLeftPID.SetMode(frontLeftPID.Control::automatic);
  frontLeftPID.SetSampleTimeUs(50000);
  //frontLeftPID.SetProportionalMode(frontLeftPID.pMode::pOnMeas); 


}

void loop() {
  readSerial();
  sampleEncoders();
  frontLeftPID.Compute();

}

void readSerial() {
  while(Serial.available()) {

    byte b = Serial.read();

    // If not currently reading a incoming packet
    if(!receivingPacket)
    {
      // If we recieved the start byte
      if(b == START_BYTE)
      {
        // Zero our index
        bufferIndex = 0;
        // Assign the byte to packetBuffer and increment index
        packetBuffer[bufferIndex++] = b;
        receivingPacket = true;
      }
    } else {
      // We have recieved our start byte so add next byte to buffer and increment index
      packetBuffer[bufferIndex++] = b;

      // We have already incremented so if bufferIndex is two we check length byte
      if (bufferIndex == 2)
      {
        // Get the packetLength
        packetLength = packetBuffer[1];

        // If packetLegth bigger that max size or less than min size.
        if(packetLength > MAX_PACKET_SIZE || packetLength < 4)
        {
          // Reset indexes and zero buffer
          receivingPacket = false;
          bufferIndex = 0;
          packetLength = 0;
          memset(packetBuffer, 0 , MAX_PACKET_SIZE);
        }
      }
      // If we recieved thw whole packet parse
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
    return;
  }

  byte type = packet[2];
  // Serial.write(packet, sizeof(packet));
  switch (type) {
    case PACKET_TYPE_FEEDBACK_REQUEST:

      if( length != FEEDBACK_REQUEST_LENGTH)
      {
        return;
      }

      // Send feedback response
      sendFeedbackResponse();
      break;
    case PACKET_TYPE_VELOCITY:
      // If packet not the length it is supposed to be return
      if (length != VELOCITY_PACKET_LENGTH) {
        return;
      }

      // Get the raw velocity from the two bytes 
      int16_t velocityFrontLeftRaw = packet[3] | (packet[4] << 8);
      int16_t velocityFrontRightRaw = packet[5] | (packet[6] << 8);

      // Convert to float using corresponding scale
      // Negate front left velocity to account for motor direction
      setpointVelocityFrontLeft = (velocityFrontLeftRaw / SCALE);
      setpointVelocityFrontRight = velocityFrontRightRaw / SCALE;

      break;
    default:
      Serial.print("Hit Default");
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
    command = (speed < 0) ? 5 : 4;
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
  if(now - lastEncoderSampleTime >= 50) {

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
    // Front left is negated to acount for motor direction.
    frontLeftRadSec = (((deltaFrontLeft / intervalSec) / (PULSES_PER_REVOLUTION * GEAR_RATIO_MULTIPLIER)) * TWO_PI) * -1;
    frontRightRadSec = ((deltaFrontRight / intervalSec) / (PULSES_PER_REVOLUTION * GEAR_RATIO_MULTIPLIER)) * TWO_PI;



    lastFrontLeftTicks = leftEncoderNow;
    lastFrontRightTicks = rightEncoderNow;
    lastEncoderSampleTime = now;
    updateMotors();
    Serial.print("\n");
    Serial.print(frontLeftRadSec);

  }
}

void sendFeedbackResponse() {
  // Create a response packet with the current encoder values
  byte response[FEEDBACK_RESPONSE_LENGTH];
  response[0] = START_BYTE;
  response[1] = FEEDBACK_RESPONSE_LENGTH;
  response[2] = PACKET_TYPE_FEEDBACK_RESPONSE;

  // Pack the encoder values into the response
  int16_t frontLeftVelocity = static_cast<int16_t>(frontLeftRadSec * SCALE);
  int16_t frontRightVelocity = static_cast<int16_t>(frontRightRadSec * SCALE);

  response[3] = frontLeftVelocity & 0xFF;
  response[4] = (frontLeftVelocity >> 8) & 0xFF;
  response[5] = frontRightVelocity & 0xFF;
  response[6] = (frontRightVelocity >> 8) & 0xFF;

  // Add dummy position values for now
  response[7] = 0; // Front left position low byte
  response[8] = 0; // Front left position high byte
  response[9] = 0; // Front right position low byte
  response[10] = 0; // Front right position high byte

  // Calculate checksum
  byte checksum = 0;
  for (int i = 0; i < FEEDBACK_RESPONSE_LENGTH - 1; ++i) {
    checksum ^= response[i];
  }
  
  response[11] = checksum;

  // Send the response packet
  Serial.write(response, FEEDBACK_RESPONSE_LENGTH);
}