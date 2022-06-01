#include <MeAuriga.h>
#include <Wire.h>

MeGyro gyro(0, 0x69);

// For line
MeLineFollower lineFinder(PORT_6);

// For motor
MeEncoderOnBoard rightMotor(SLOT1);
MeEncoderOnBoard leftMotor(SLOT2);

// For UltrasonicSensor
MeUltrasonicSensor ultraSensor(PORT_7);

// Global direction
enum DIRECTION
{
  FORWARD,
  BACK,
  LEFT,
  RIGHT
};

DIRECTION direction = BACK;
int SPEED = 50;
float previousX = 0;
float previousY = 0;
int startdir = 1;
char data;
bool manual = true;

float distanceX = 0;
float distanceY = 0;
int leftPulses = 0;
int rightPulses = 0;

void isr_process_encoder1(void)
{
  if (digitalRead(rightMotor.getPortB()) == 0)
  {
    rightMotor.pulsePosMinus();
  }
  else
  {
    rightMotor.pulsePosPlus();
  }
}

void isr_process_encoder2(void)
{
  if (digitalRead(leftMotor.getPortB()) == 0)
  {
    leftMotor.pulsePosMinus();
  }
  else
  {
    leftMotor.pulsePosPlus();
  }
}

void setup()
{
  Serial.begin(115200);
  gyro.begin();
  attachInterrupt(rightMotor.getIntNum(), isr_process_encoder1, RISING);
  attachInterrupt(leftMotor.getIntNum(), isr_process_encoder2, RISING);

  //Set PWM 8KHz
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);

  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);
}

// OM VÄNSTER ÄR NEGATIV OCH HÖGER POSITIV SÅ GÅR DEN BAKÅT
// 1 är HÖGER
// 2 är VÄNSTER
void driveForward()
{
  leftMotor.setMotorPwm(60);
  rightMotor.setMotorPwm(-60);
  leftMotor.updateSpeed();
  rightMotor.updateSpeed();
  direction = FORWARD;
}

void driveBack()
{
  leftMotor.setMotorPwm(-60);
  rightMotor.setMotorPwm(60);
  leftMotor.updateSpeed();
  rightMotor.updateSpeed();
  direction = BACK;
}

void collisionDetection()
{
  if (ultraSensor.distanceCm() < 10)
  {
    Serial.write("obstacle detected, chainging direction");
    motorStop();
    int randomInt = random(1, 3);
    if (randomInt == 1)
    {
      driveLeft();
    }
    else if (randomInt == 2)
    {
      driveRight();
    }
  }
}

void driveRight()
{
  if (direction == BACK)
  {
    driveForward();
    delay(100);
    rightMotor.setMotorPwm(SPEED);
    rightMotor.updateSpeed();
    leftMotor.setMotorPwm(SPEED);
    leftMotor.updateSpeed();
  }
  else
  {
    rightMotor.setMotorPwm(-SPEED);
    rightMotor.updateSpeed();
    leftMotor.setMotorPwm(-SPEED);
    leftMotor.updateSpeed();
  }
  delay(500);
  driveForward();
}

void driveLeft()
{
  if (direction == BACK)
  {
    driveForward();
    delay(100);
    rightMotor.setMotorPwm(-SPEED);
    rightMotor.updateSpeed();
    leftMotor.setMotorPwm(-SPEED);
    leftMotor.updateSpeed();
  }
  else
  {
    rightMotor.setMotorPwm(SPEED);
    rightMotor.updateSpeed();
    leftMotor.setMotorPwm(SPEED);
    leftMotor.updateSpeed();
  }
  delay(500);
  driveForward();
}
void motorStop()
{
  rightMotor.setMotorPwm(0);
  rightMotor.updateSpeed();
  leftMotor.setMotorPwm(0);
  leftMotor.updateSpeed();
  Serial.println(ultraSensor.distanceCm());
  delay(500);
}

void manualDriveForward()
{
  leftMotor.setMotorPwm(SPEED);
  rightMotor.setMotorPwm(-SPEED);
  leftMotor.updateSpeed();
  rightMotor.updateSpeed();
}

void manualDriveBackwards()
{
  leftMotor.setMotorPwm(-SPEED);
  rightMotor.setMotorPwm(SPEED);
  leftMotor.updateSpeed();
  rightMotor.updateSpeed();
}

void manualDriveLeft()
{
  rightMotor.setMotorPwm(-SPEED);
  rightMotor.updateSpeed();
  leftMotor.setMotorPwm(-SPEED);
  leftMotor.updateSpeed();
}

void manualDriveRight()
{
  rightMotor.setMotorPwm(SPEED);
  rightMotor.updateSpeed();
  leftMotor.setMotorPwm(SPEED);
  leftMotor.updateSpeed();
}

void updatePosition(float angle, int pulseRight, int pulseLeft)
{
  int averagePulses = (pulseLeft + (-pulseRight)) / 2;
  distanceX = -averagePulses * 0.056694 * sin(angle * (3.14 / 180)) + previousX;
  distanceY = -averagePulses * 0.056694 * cos(angle * (3.14 / 180)) + previousY;
  leftMotor.setPulsePos(0);
  rightMotor.setPulsePos(0);
  previousX = distanceX;
  previousY = distanceY;
  Serial.print("(");
  Serial.print(distanceX);
  Serial.print(", ");
  Serial.print(distanceY);
  Serial.println(")");
}

void loop()
{
  gyro.update();
  //delay(10);
  data = ' ';
  if (Serial.available())
  {
    data = Serial.read();
    Serial.println(data);
  }

  if (data == '6' || manual == true)
  {
    switch (data)
    {
    case '1':
      manualDriveForward();
      Serial.write("Driving Forwards");
      delay(50);
      break;
    case '2':
      Serial.write("Turning left");
      manualDriveLeft();
      delay(50);
      leftMotor.setPulsePos(0);
      rightMotor.setPulsePos(0);
      break;
    case '3':
      Serial.write("Turning right");
      manualDriveRight();
      delay(50);
      leftMotor.setPulsePos(0);
      rightMotor.setPulsePos(0);
      break;

    case '4':
      Serial.write("Driving Backwards");
      manualDriveBackwards();
      delay(50);
      break;
    case '5':
      leftPulses = leftMotor.getPulsePos();
      rightPulses = rightMotor.getPulsePos();
      updatePosition(gyro.getAngleZ(), leftPulses, rightPulses);
      Serial.write("breaking");
      motorStop();
      delay(50);
      break;

    case '6':
      Serial.write("switching mode");
      if (manual == false)
      {
        manual = true;
        Serial.write("Manual mode.");
      }
      else
      {
        manual = false;
        Serial.write("Automatic mode.");
      }
      break;

    default:
      break;
    }
  }
  if (manual == false)
  { //everything below is autonimus
    if (startdir == 1)
    {
      driveForward();
      startdir += 1;
    }
    collisionDetection();

    int sensorState = lineFinder.readSensors();
    switch (sensorState)
    {
    case S1_IN_S2_IN:
      if (direction == FORWARD)
      {
        int randomInt = random(1, 3);
        if (randomInt == 1)
        {
          rightMotor.setMotorPwm(-SPEED);
          rightMotor.updateSpeed();
          leftMotor.setMotorPwm(-SPEED);
          leftMotor.updateSpeed();
        }
        else if (randomInt == 2)
        {
          rightMotor.setMotorPwm(SPEED);
          rightMotor.updateSpeed();
          leftMotor.setMotorPwm(SPEED);
          leftMotor.updateSpeed();
        }
        int randomDelay = random(1000, 2001);
        delay(randomDelay);
      }
      else
      {
        driveForward();
        //delay(500);
      }
      break;
    case S1_IN_S2_OUT:
      if (direction == BACK)
      {
        driveRight();
      }
      else
      {
        driveLeft();
      }
      break;
    case S1_OUT_S2_IN:
      if (direction == BACK)
      {
        driveLeft();
      }
      else
      {
        driveRight();
      }
      break;
    case S1_OUT_S2_OUT:
      if (direction == FORWARD)
      {
        driveForward();
      }
      else
      {
        driveBack();
      }
      break;
    default:
      break;
    }
    delay(50);
  }
}
