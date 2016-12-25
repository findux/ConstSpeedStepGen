// ConstSpeedStepGen.h
/// \yazat Eyüp Fındıklı (eyupfindikli@gmail.com) DO NOT CONTACT THE AUTHOR DIRECTLY: USE THE LISTS
// Copyright (C) 2016 Eyüp Fındıklı
#define debug 1
#ifndef ConstSpeedStepGen_h
#define ConstSpeedStepGen_h

#include <stdlib.h>
#if ARDUINO >= 100
#include <Arduino.h>
#else
#include <WProgram.h>
#include <wiring.h>
#endif

// These defs cause trouble on some versions of Arduino
#undef round

class ConstSpeedStepGen
{
public:
	//EepromAddress verilirken diğer leri ile çakışmamalı
     ConstSpeedStepGen( uint8_t step_pin = 4, uint8_t dir_pin = 5,uint8_t EepromAddress = 0,bool enable = true);
    void    moveTo(long absolute); 
    void    move(long relative);
    boolean run();
    boolean runSpeed();
    void    setMaxSpeed(float speed);
    float   maxSpeed();
    void    setSpeed(float speed);
    float   speed();
    long    distanceToGo();
    long    targetPosition();
    long    currentPosition();  
    void    setCurrentPosition(unsigned long position);  
    void    runToPosition(); // istenilen pozisyona gidene kadar diğerleri çalışmaz

    /// Runs at the currently selected speed until the target position is reached
    /// Does not implement accelerations.
    /// \return true if it stepped
    boolean runSpeedToPosition();

    /// Moves the motor (with acceleration/deceleration)
    /// to the new target position and blocks until it is at
    /// position. Dont use this in event loops, since it blocks.
    /// \param[in] position The new target position.
    void    runToNewPosition(long position);

    /// Sets a new target position that causes the stepper
    /// to stop as quickly as possible, using the current speed and acceleration parameters.
    void stop();
	
	/// Disable motor pin outputs by setting them all LOW
    /// Depending on the design of your electronics this may turn off
    /// the power to the motor coils, saving power.
    /// This is useful to support Arduino low power modes: disable the outputs
    /// during sleep and then reenable with enableOutputs() before stepping
    /// again.
    /// If the enable Pin is defined, sets it to OUTPUT mode and clears the pin to disabled.
    virtual void    disableOutputs();

    /// Enable motor pin outputs by setting the motor pins to OUTPUT
    /// mode. Called automatically by the constructor.
    /// If the enable Pin is defined, sets it to OUTPUT mode and sets the pin to enabled.
    virtual void    enableOutputs();

    /// Sets the minimum pulse width allowed by the stepper driver. The minimum practical pulse width is 
    /// approximately 20 microseconds. Times less than 20 microseconds
    /// will usually result in 20 microseconds or so.
    /// \param[in] minWidth The minimum pulse width in microseconds. 
    void    setMinPulseWidth(unsigned int minWidth);

    /// Sets the enable pin number for stepper drivers.
    /// 0xFF indicates unused (default).
    /// Otherwise, if a pin is set, the pin will be turned on when 
    /// enableOutputs() is called and switched off when disableOutputs() 
    /// is called.
    /// \param[in] enablePin Arduino digital pin number for motor enable
    /// \sa setPinsInverted
    void    setEnablePin(uint8_t enablePin = 0xff);

    /// Sets the inversion for stepper driver pins
    /// \param[in] directionInvert True for inverted direction pin, false for non-inverted
    /// \param[in] stepInvert      True for inverted step pin, false for non-inverted
    /// \param[in] enableInvert    True for inverted enable pin, false (default) for non-inverted
    void    setPinsInverted(bool directionInvert = false, bool stepInvert = false, bool enableInvert = false);

    /// Checks to see if the motor is currently running to a target
    /// \return true if the speed is not zero or not at the target position
    bool    isRunning();
	void EEPROM_write_CurrentPos();
	void EEPROM_read_CurrentPos();

protected:

/*     // / \brief Direction indicator
    // / Symbolic names for the direction the motor is turning
    typedef enum
    {
	DIRECTION_CCW = 0,  ///< Clockwise
        DIRECTION_CW  = 1   ///< Counter-Clockwise
    } Direction;

    // / Low level function to set the motor output pins
    // / bit 0 of the mask corresponds to _pin[0]
    // / bit 1 of the mask corresponds to _pin[1]
    // / You can override this to impment, for example serial chip output insted of using the
    // / output pins directly
    virtual void   setOutputPins(uint8_t mask);

    // / Called to execute a step. Only called when a new step is
    // / required. Subclasses may override to implement new stepping
    // / interfaces. The default calls step1(), step2(), step4() or step8() depending on the
    // / number of pins defined for the stepper.
    // / \param[in] step The current step phase number (0 to 7) */
	    typedef enum
    {
	DIRECTION_CCW = 0,  ///< Clockwise
        DIRECTION_CW  = 1   ///< Counter-Clockwise
    } Direction;
	
	virtual void   setOutputPins(uint8_t mask);
	
	void           computeNewSpeed();

    virtual void   step(long step);
	
	/// Called to execute a step on a stepper driver (ie where pins == 1). Only called when a new step is
    /// required. Subclasses may override to implement new stepping
    /// interfaces. The default sets or clears the outputs of Step pin1 to step, 
    /// and sets the output of _pin2 to the desired direction. The Step pin (_pin1) is pulsed for 1 microsecond
    /// which is the minimum STEP pulse width for the 3967 driver.
    /// \param[in] step The current step phase number (0 to 7)
    virtual void   step1(long step);
	
private:

    /// Arduino pin number assignments for the 2 or 4 pins required to interface to the
    /// stepper motor or driver
    uint8_t        _pin[2];

    /// Whether the _pins is inverted or not
    uint8_t        _pinInverted[2];
     /// The current absolution position in steps.
    unsigned long           _currentPos;    // Steps

    /// The target position in steps. The AccelStepper library will move the
    /// motor from the _currentPos to the _targetPos, taking into account the
    /// max speed, acceleration and deceleration
    unsigned long           _targetPos;     // Steps

    /// The current motos speed in steps per second
    /// Positive is clockwise
    float          _speed;         // Steps per second

    /// The maximum permitted speed in steps per second. Must be > 0.
    float          _maxSpeed;

    /// The current interval between steps in microseconds.
    /// 0 means the motor is currently stopped with _speed == 0
    unsigned long  _stepInterval;
	
	    /// The last step time in microseconds
    unsigned long  _lastStepTime;

    /// The minimum allowed pulse width in microseconds
    unsigned int   _minPulseWidth;
	
	    /// Is the enable pin inverted?
    bool           _enableInverted;

    /// Enable pin for stepper driver, or 0xFF if unused.
    uint8_t        _enablePin;
	
	/// EepromAddress diğer bir ConstSpeedStepGen objesininkinden çakışmamalı
	/// eeprom 4 byte yazılacağı için diğeri ConstSpeedStepGen eepromaddresi +4 byte ötelenmeli
	uint8_t _EepromAddress;
	
    /// The step counter for speed calculations
    long _n;

    /// Initial step size in microseconds
    float _c0;

    /// Last step size in microseconds
    float _cn;

    /// Min step size in microseconds based on maxSpeed
    float _cmin; // at max speed

    /// Current direction motor is spinning in
    boolean _direction; // 1 == CW

};
#endif 
