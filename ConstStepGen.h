#define debug 1

#ifndef ConstStepGen_h
#define ConstStepGen_h

#include <stdlib.h>
#if ARDUINO >= 100
#include <Arduino.h>
#else
#include <WProgram.h>
#include <wiring.h>
#endif

class ConstStepGen {
  public:
    ConstStepGen(uint8_t step_pin = 4, uint8_t dir_pin = 5 , uint8_t EepromAddress = 0);
    void    moveTo(long absolute);
    void    move(long relative);
    boolean run();
    boolean runSpeed();
    void    setMaxSpeed(float speed);
    float   maxSpeed();
    void    setSpeed(float speed);
    void    initOutputPin();
    long    distanceToGo();
    void    setPinsInverted(bool stepInvert = false, bool directionInvert = false);
    void    stop();
    void    setCurrentPosition(unsigned long position);
    void    EEPROM_write_CurrentPos();
    void    EEPROM_read_CurrentPos();
    bool    isRunning();

  protected:
    typedef enum
    {
      DIRECTION_CCW = 0,
      DIRECTION_CW  = 1
    } Direction;

    virtual void   setOutputPins(uint8_t mask);
    void reCalculate();
    void step();

  private:
    /// step motor driverları için step dir pinleri
    uint8_t _pin[2];

    /// eğer pinler invet edilmişse
    uint8_t _pinInverted[2];

    long _currentPos; //Stepler

    long _targetPos;//Stepler

    float _speed; // saniye başına step

    float _maxSpeed;// saniye başına maxSpeed;

    unsigned long _stepInterval;//stepler arası heseplanan zaman

    float _stepMinInterval;// max hızdaki step aralığı

    /// son atılan stepin microsaniye cinsinden değeri
    unsigned long  _lastStepTime;

    unsigned int _minPulseWidth; //step tick zamanı

    boolean _direction;//yön

    uint8_t _EepromAddress;//acil durumda eeprom yazılacak current pozisyonun başlangıç adresi

};
#endif
