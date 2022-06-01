#include "robotpacket.h"

#include <stdint.h>

#include <MeAuriga.h>
#include <Wire.h>

#define BUZZER_PORT 45
#define RGB_LED_PORT 44

// For mBot
MeGyro gyro(0, 0x69);
MeLineFollower lineFinder(PORT_6);
MeEncoderOnBoard rightMotor(SLOT1);
MeEncoderOnBoard leftMotor(SLOT2);
MeUltrasonicSensor ultraSensor(PORT_7);
MeBuzzer buzzer;
MeRGBLed led( 0, 12 );

void isr_process_encoder1(void)
{
  if (digitalRead(rightMotor.getPortB()) == 0)
    rightMotor.pulsePosMinus();
  else
    rightMotor.pulsePosPlus();
}

void isr_process_encoder2(void)
{ 
  if (digitalRead(leftMotor.getPortB()) == 0)
    leftMotor.pulsePosMinus();
  else
    leftMotor.pulsePosPlus();
}

int8_t current_acceleration;
int8_t current_steering;

boolean isConnected = false;
boolean isAutomove = false;

void setup() {
  Serial.begin(115200);
  Serial.flush();

  gyro.begin();
  attachInterrupt(rightMotor.getIntNum(), isr_process_encoder1, RISING);
  attachInterrupt(leftMotor.getIntNum(), isr_process_encoder2, RISING);

  //Set PWM 8KHz
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);

  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);

  buzzer.setpin(BUZZER_PORT);
  led.setpin(RGB_LED_PORT);

  //TEST: Sets forward values for PwmMotors in setup
  //on_acceleration_changed(40); 
}

int counter = 0;

void loop() {
  process_message_protocol();

  if (isAutomove && isConnected)
    auto_mode();
    
  delay(10);
}

void on_connected() {
  isConnected = true;
  play_hanshake();  
}

void on_disconnected() {
  isConnected = false;
}

void on_acceleration_changed(int8_t acceleration) {
  //Serial.println("on_acceleration_changed triggered"); 
  //Serial.println(String(acceleration));
  
  current_acceleration = acceleration;
  update_motion(current_acceleration, current_steering);
}


void on_steering_changed(int8_t steering) {
  //Serial.println("on_steering_changed triggered"); 
  //Serial.println(String(steering));
  
  current_steering = steering;
  update_motion(current_acceleration, current_steering);
}
void on_mode_changed(){
   isAutomove = !isAutomove;

   if (isAutomove)
    on_acceleration_changed(60);
   else
    on_acceleration_changed(0);
}


bool collision(int distance)
{
  return (ultraSensor.distanceCm() < distance);
}

void update_motion(int8_t acceleration, int8_t steering) { 
  float normalized = 1.0f - ((float)abs(steering) / 100.0f);
  int turnAcceleration = (normalized > 0.2f) ? (int)(acceleration * normalized) : acceleration * -1;
  
  // Right turn
  if (steering > 0) {
    rightMotor.setMotorPwm(-turnAcceleration);
    leftMotor.setMotorPwm(acceleration);
  }
  // Left turn
  else if (steering < 0) {
    rightMotor.setMotorPwm(-acceleration);
    leftMotor.setMotorPwm(turnAcceleration);
  }
  // Forward/backwards
  else {
    rightMotor.setMotorPwm(-acceleration);
    leftMotor.setMotorPwm(acceleration);
  }

  rightMotor.updateSpeed();
  leftMotor.updateSpeed();
}



void collision_handler(int triggerDistance) {
  int8_t distance = static_cast<int8_t>(ultraSensor.distanceCm());
  if (distance < triggerDistance) {
    RobotPacket<int8_t> packet = RobotPacket<int8_t>(RobotCommand::COLLISION, distance);
    int8_t packetBuffer[packet.get_length()];
    
    packet.to_bytes(packetBuffer);
    write_n_bytes(packetBuffer, packet.get_length());
  }
}

void coordinate_handler(int interval) {
  static unsigned long startTime = millis();
  static float lastX = 0;
  static float lastY = 0;
  static float startRightPosition = rightMotor.getPulsePos();
  static float startLeftPosition = leftMotor.getPulsePos();

  if (isnan(lastX) || isnan(lastY)) {
    lastX = 0;
    lastY = 0;
  }
  
  if (millis() - startTime > interval) {   
    // Update the angle of the gyroscope
    gyro.update();
    float angle = 360 - (gyro.getAngleZ() + 180);
    
    float rightDistance = ((rightMotor.getPulsePos() - startRightPosition) / 360) * 20.41;
    float leftDistance = ((leftMotor.getPulsePos() - startLeftPosition) / 360) * 20.41;

    float maxDistance = max(abs(rightDistance), abs(leftDistance));

    float factor = (((-rightDistance + leftDistance) / 2) / maxDistance);
    float distance = maxDistance * factor;

    float x = distance * cos(radians(angle));
    float y = distance * sin(radians(angle));

    lastX = lastX + x;
    lastY = lastY + y;

    startTime = millis();
    startRightPosition = rightMotor.getPulsePos();
    startLeftPosition = leftMotor.getPulsePos();
  }
}



void auto_mode() {

  switch (lineFinder.readSensors()) {
    case S1_IN_S2_IN:
      if (current_acceleration > 0)
        ((random(0, 2) == 0) ? &turn_left : &turn_right)(1500);
      break;
    case S1_IN_S2_OUT:
      ((current_acceleration < 0) ? &turn_left : &turn_right)(500);    
      break;
    case S1_OUT_S2_IN:
      ((current_acceleration < 0) ? &turn_right : &turn_left)(500);           
      break;
    case S1_OUT_S2_OUT:
      break;
  }
  if (collision(10)) {
    ((random(0, 2) == 0) ? &turn_left : &turn_right)(500);
  }
}

void turn_right(int duration) {
  on_steering_changed(100);
  delay(duration);
  on_steering_changed(0);
}

void turn_left(int duration) {
  on_steering_changed(-100);
  delay(duration);
  on_steering_changed(0);
}






void read_n_bytes(int8_t* packetBuffer, const int n) {
  unsigned long startTime = millis();
  while ((Serial.available() < n) && (millis() - startTime < 2000)){}
  
  for (size_t i = 0; i < n; i++) {  
    int b = Serial.read();
    if (b < 0) break;   

    *(packetBuffer+i) = static_cast<int8_t>(b);
  }
}

void write_n_bytes(int8_t* packetBuffer, const int n) {
  for (size_t i = 0; i < n; i++)
    Serial.write(packetBuffer[i]);
}


void process_message_protocol() {
  int8_t packetBuffer[64];

  if (Serial.available() > 0) {
    RobotCommand command = static_cast<RobotCommand>(Serial.read());

    if (command == CONNECTED && !isConnected) {
      on_connected();
      return;
    }
  
    switch(command) {
    case DISCONNECTED:
      on_disconnected();
      break;
    case ACCELERATION:
      read_n_bytes(packetBuffer, sizeof(int8_t));
      on_acceleration_changed(RobotPacket<int8_t>(command, packetBuffer).get_parameter());
      break;
    case STEERING:
      read_n_bytes(packetBuffer, sizeof(int8_t));
      on_steering_changed(RobotPacket<int8_t>(command, packetBuffer).get_parameter());
      break;
    case MODE:
      on_mode_changed();
      break;
    }
  }
}


void play_hanshake() 
{
  int pitch = 2;
  
  // Van Halen - Talkin' Bout Love - Guitar Solo
  led.setColorAt(0, 100, 0, 0);
  led.setColorAt(6, 100, 0, 0);
  led.show();
  on_acceleration_changed(100);
  on_steering_changed(100);
  buzzer.tone(330 * pitch, 250); // E
  
  led.setColorAt(1, 0, 0, 100);
  led.setColorAt(7, 0, 0, 100);
  led.show();
  on_acceleration_changed(-100);
  buzzer.tone(392 * pitch, 250); // G
  
  led.setColorAt(2, 0, 100, 0);
  led.setColorAt(8, 0, 100, 0);
  led.show();
  on_acceleration_changed(100);
  buzzer.tone(440 * pitch, 250); // A
  
  led.setColorAt(3, 100, 0, 100);
  led.setColorAt(9, 100, 0, 100);
  led.show();
  on_acceleration_changed(-100);
  buzzer.tone(440 * pitch, 100); // A
  
  led.setColorAt(4, 0, 100, 100);
  led.setColorAt(10, 0, 100, 100);
  led.show();
  on_acceleration_changed(100);
  buzzer.tone(440 * pitch, 250); // A
  
  led.setColorAt(5, 100, 100, 0);
  led.setColorAt(11, 100, 100, 0);
  led.show();
  on_acceleration_changed(-100);
  buzzer.tone(440 * pitch, 100); // A
  
  for (int i = 0; i < 12; i++) {
    led.setColorAt(i, 175, 175, 175);
    led.show();
  }
  on_acceleration_changed(100);
  buzzer.tone(494 * pitch, 250); // B
  
  for (int i = 0; i < 12; i++) {
    led.setColorAt(i, 255, 255, 255);
    led.show();
  }
  on_acceleration_changed(-100);
  buzzer.tone(392 * pitch, 500); // G
  
  on_acceleration_changed(0);
  on_steering_changed(0);
  
  for (int i = 0; i < 12; i++) {
    led.setColorAt(i, 0, 255, 0);
    buzzer.tone(50 * i, 50);
    led.show();
  }

  for (int i = 0; i < 12; i++) {
    led.setColorAt(i, 0, 0, 0);
    buzzer.tone(550 - (i * 50), 50);
    led.show();
  }

  buzzer.noTone();
} 
