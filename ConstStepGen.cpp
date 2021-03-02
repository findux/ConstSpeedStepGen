#include "ConstStepGen.h"
#include <EEPROM.h>

ConstStepGen::ConstStepGen(uint8_t step_pin , uint8_t dir_pin, uint8_t EepromAddress) {

  _pin[0] = step_pin;  // step
  _pin[1] = dir_pin;  //dir
  _currentPos = 0; //
  _targetPos = 0;
  _speed = 0.0;
  _maxSpeed = 1.0;
  _stepInterval = 0;
  _minPulseWidth = 1;
  _lastStepTime = 0;
  _direction = DIRECTION_CCW;
  _EepromAddress = EepromAddress;

  int i;
  for (i = 0; i < 2; i++)
    _pinInverted[i] = 0;

  initOutputPin();

}

void ConstStepGen::initOutputPin() {
  pinMode(_pin[0], OUTPUT);
  pinMode(_pin[1], OUTPUT);
}

void ConstStepGen::moveTo(long absolute) {
  if (_targetPos != absolute) {
    _direction = ((absolute - _targetPos) > 0.0) ? DIRECTION_CW : DIRECTION_CCW;
    _targetPos = absolute;
  }
#if debug
  Serial.println("moveTo------ _direction/_targetPos");
  Serial.println(_direction);
  Serial.println(_targetPos);
  Serial.println("-----");
#endif
}

void ConstStepGen::move(long relative) {
  moveTo(_currentPos + relative);
}

void ConstStepGen::setMaxSpeed(float speed) {

  if (_maxSpeed != speed) {
    _maxSpeed = speed;
    _stepMinInterval = 1000000.0 / speed;
  }
#if debug
  Serial.println("setMaxSpeed------ _maxSpeed/_stepMinInterval");
  Serial.println(_maxSpeed);
  Serial.println(_stepMinInterval);
  Serial.println("-----");
#endif
}

float ConstStepGen::maxSpeed() {
  return _maxSpeed;
}

void ConstStepGen::setSpeed(float speed) {
  if (speed == _speed)
    return;

  speed = constrain(speed, -_maxSpeed, _maxSpeed);
  if (speed == 0.0)
    _stepInterval = 0;
  else {
    _stepInterval = fabs(1000000.0 / speed);
  }
  _speed = speed;

#if debug
  Serial.println("setSpeed edildi------_speed/_stepInterval");
  Serial.println(_speed);
  Serial.println(_stepInterval);
  Serial.println("-----");
#endif
}

long ConstStepGen::distanceToGo()
{
  return _targetPos - _currentPos;
}

boolean ConstStepGen::runSpeed() {
  // Dont do anything unless we actually have a step interval
  if (!_stepInterval)
    return false;

  unsigned long time = micros();
  if (time - _lastStepTime >= _stepInterval) {
    if (_direction == DIRECTION_CW) {
      // Clockwise
      _currentPos += 1;
    }
    else {
      // Anticlockwise
      _currentPos -= 1;
    }
    //step(_currentPos);
    step();
    
#if debug    
    Serial.println("STEP");
#endif

    _lastStepTime = time;
    return true;
  }
  else {
    return false;
  }
}

void ConstStepGen::reCalculate() {
  long distanceTo = distanceToGo();

  if (distanceTo == 0) {
    _stepInterval = 0;
    _speed = 0.0;
    return;
  }

  _direction = (distanceTo > 0) ? DIRECTION_CW : DIRECTION_CCW;

#if debug
  Serial.println("reCalculate------_direction/distanceTo/");
  Serial.println(_direction);
  Serial.println(distanceTo);
  Serial.println("-----");
#endif
}

boolean ConstStepGen::run() {

  if (runSpeed())
    reCalculate();

#if debug
  Serial.println("run------distanceToGo()/_targetPos/_currentPos");
  Serial.println(distanceToGo());
  Serial.println(_targetPos);
  Serial.println(_currentPos);
  Serial.println("-----");
#endif

  return distanceToGo() != 0;

}

void ConstStepGen::step() {

  // _pin[0] is step, _pin[1] is direction
  setOutputPins(_direction ? 0b10 : 0b00);
  // Set direction first else get rogue pulses
  setOutputPins(_direction ? 0b11 : 0b01); // step HIGH
  // Caution 200ns setup time
  // Delay the minimum allowed pulse width
  delayMicroseconds(_minPulseWidth);
  setOutputPins(_direction ? 0b10 : 0b00); // step LOW

}

void ConstStepGen::setOutputPins(uint8_t mask) {
  uint8_t numpins = 2;
  uint8_t i;
  for (i = 0; i < numpins; i++)
    digitalWrite(_pin[i], (mask & (1 << i)) ? (HIGH ^ _pinInverted[i]) : (LOW ^ _pinInverted[i]));
}

void ConstStepGen::setPinsInverted(bool stepInvert, bool directionInvert) {
  _pinInverted[0] = stepInvert;
  _pinInverted[1] = directionInvert;
}

void ConstStepGen::stop() {
  setSpeed(0); // hız sıfır
  _targetPos = _currentPos;
  EEPROM_write_CurrentPos();
}

void ConstStepGen::EEPROM_write_CurrentPos()
{
  byte* p = (byte*)(void*)&_currentPos;
  for (int i = 0; i < sizeof(_currentPos); i++)
    EEPROM.write(_EepromAddress++, *p++);
}

void ConstStepGen::EEPROM_read_CurrentPos()
{
  unsigned long value = 0.0;
  byte* p = (byte*)(void*)&value;
  for (int i = 0; i < sizeof(value); i++)
    *p++ = EEPROM.read(_EepromAddress++);
  setCurrentPosition(value);
}

void ConstStepGen::setCurrentPosition(unsigned long position) {
  _targetPos = _currentPos = position;
  _stepInterval = 0;
  _speed = 0.0;
}

bool ConstStepGen::isRunning() {
  return !(_targetPos == _currentPos);
}
