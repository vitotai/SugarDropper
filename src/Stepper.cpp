#include <Arduino.h>
#include "Stepper.h"

// a implementation of "timed" pump
// Not good programming pratice, but using static member is tidious.


// using 1ms as "step", but 10ms (100Hz) interrupt period
// let flow rate @ 150ml, 10ms resolution =  150/60/100 = 0.025ml far more than enough
// [ 16,000,000Hz/ (prescaler * desired interrupt frequency) ] - 1
// 16000000/(1024*100) -1 = 16000000/(1024*100) -1= 155.25

#define OverflowCompare 155

void Stepper::_powerOn(){
    _running=true;
    digitalWrite(_actuatorPin,LOW);
    _startTime=millis();
}

void Stepper::_powerOff(){
    digitalWrite(_actuatorPin,HIGH);
    _accRunTime += millis()  - _startTime;
    _running=false;
}

void Stepper::timerTick(){
    if(_autoStop){
        if(millis() >= _autoStopTime){
            _autoStop=false;
            _powerOff();
        }
    }
}
static Stepper* _stepers[2]={NULL,NULL};
static uint8_t _stepperCount=0;
ISR (TIMER1_COMPA_vect) 
{
    if(_stepers[0]) _stepers[0]->timerTick();
    if(_stepers[1]) _stepers[1]->timerTick();
}

static void _setupTimer()
{
	// set Timer 1 prescaler to 1024
	// 64 us, CTC set
/*	TCCR1B = (TCCR1B & 0b11110000) |0x08 |0x04;
	OCR1A = _overflowCompare;
	// enable timer 1
	TIMSK1 |= (1<<OCIE1A);
*/
	noInterrupts();
    TCCR1A = 0;// set entire TCCR1A register to 0
    TCCR1B = 0;// same for TCCR1B
    TCNT1  = 0;//initialize counter value to 0
    // set compare match register for 1000000hz increments with 8 bits prescaler
    OCR1A = OverflowCompare;//   [ 16,000,000Hz/ (prescaler * desired interrupt frequency) ] - 1
    // turn on CTC mode
    TCCR1B |= (1 << WGM12);
    // Set CS11 bit for 8 prescaler. Each timer has a different bit code to each prescaler
    TCCR1B |= (1 << CS12); 
    TCCR1B |= (1 << CS10);  
    // enable timer compare interrupt
    TIMSK1 |= (1 << OCIE1A);
    interrupts();
}

uint32_t Stepper::startTime(){
    return _startTime;
}

Stepper::Stepper(byte stepPin,byte dirPin){
    _actuatorPin = stepPin;
}

void Stepper::begin(){
    _accRunTime =0;
    pinMode(_actuatorPin, OUTPUT);
    _powerOff();

    if(_stepperCount < 2){
        _stepers[_stepperCount] = this;
        _stepperCount ++;
    }

    if(_stepperCount==1) _setupTimer();
}

uint32_t Stepper::run(){
    _powerOn();
    return _accRunTime;
}
void Stepper::stop(){
    _powerOff();
}

bool Stepper::running(){
    return _running;
}

uint32_t Stepper::runSteps(unsigned long step){
    // in miniseconds
    _autoStopTime = millis() + step;
    _powerOn();
    _autoStop= true;
    return _accRunTime;
}

uint32_t Stepper::steps(){
    return _accRunTime + (_running? (millis()-_startTime):0);
}

/* not used */
void Stepper::setDirectiionForward(bool forward){}
void Stepper::setRPM(unsigned long rpm){}

