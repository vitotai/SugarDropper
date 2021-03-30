#define LongPressedTime 1000

typedef enum{
  RotaryEncoderStatusNone,
  RotaryEncoderStatusPushed,
  RotaryEncoderStatusDepushed,
  RotaryEncoderStatusLongPressed,
  RotaryEncoderStatusFordward,
  RotaryEncoderStatusBackward
}RotaryEncoderStatus;

// the following code is from BrewPi. Good one.

#define DIR_NONE 0x0
// Clockwise step.
#define DIR_CW 0x10
// Anti-clockwise step.
#define DIR_CCW 0x20

#define R_START 0x0
// half steps
// Use the half-step state table (emits a code at 00 and 11)
#define HS_R_CCW_BEGIN 0x1
#define HS_R_CW_BEGIN 0x2
#define HS_R_START_M 0x3
#define HS_R_CW_BEGIN_M 0x4
#define HS_R_CCW_BEGIN_M 0x5

const uint8_t PROGMEM hs_ttable[7][4] = {
	// R_START (00)
	{HS_R_START_M,            HS_R_CW_BEGIN,     HS_R_CCW_BEGIN,  R_START},
	// HS_R_CCW_BEGIN
	{HS_R_START_M | DIR_CCW, R_START,        HS_R_CCW_BEGIN,  R_START},
	// HS_R_CW_BEGIN
	{HS_R_START_M | DIR_CW,  HS_R_CW_BEGIN,     R_START,      R_START},
	// HS_R_START_M (11)
	{HS_R_START_M,            HS_R_CCW_BEGIN_M,  HS_R_CW_BEGIN_M, R_START},
	// HS_R_CW_BEGIN_M
	{HS_R_START_M,            HS_R_START_M,      HS_R_CW_BEGIN_M, R_START | DIR_CW},
	// HS_R_CCW_BEGIN_M
	{HS_R_START_M,            HS_R_CCW_BEGIN_M,  HS_R_START_M,    R_START | DIR_CCW},
	{R_START, R_START, R_START, R_START}
};
#if 0
// full steps
#define R_CW_FINAL 0x1
#define R_CW_BEGIN 0x2
#define R_CW_NEXT 0x3
#define R_CCW_BEGIN 0x4
#define R_CCW_FINAL 0x5
#define R_CCW_NEXT 0x6
const uint8_t PROGMEM ttable[7][4] = {
	// R_START
	{R_START,    R_CW_BEGIN,  R_CCW_BEGIN, R_START},
	// R_CW_FINAL
	{R_CW_NEXT,  R_START,     R_CW_FINAL,  R_START | DIR_CW},
	// R_CW_BEGIN
	{R_CW_NEXT,  R_CW_BEGIN,  R_START,     R_START},
	// R_CW_NEXT
	{R_CW_NEXT,  R_CW_BEGIN,  R_CW_FINAL,  R_START},
	// R_CCW_BEGIN
	{R_CCW_NEXT, R_START,     R_CCW_BEGIN, R_START},
	// R_CCW_FINAL
	{R_CCW_NEXT, R_CCW_FINAL, R_START,     R_START | DIR_CCW},
	// R_CCW_NEXT
	{R_CCW_NEXT, R_CCW_FINAL, R_CCW_BEGIN, R_START},
};
#endif


class RotaryEncoder
{
  private:
    uint8_t _state;
        int _step;
        byte _output;
	byte _position;
        bool _pushed;
        bool _skipUp;
        unsigned long _pressedTime;
    int _encoderPinA;
    int _encoderPinB;
    int _encoderPinP;

  public:
	RotaryEncoder(int pinA,int pinB, int pinSW) 
	{ 
        _encoderPinA = pinA;
        _encoderPinB = pinB;
        _encoderPinP = pinSW;
		pinMode(_encoderPinA, INPUT);
		digitalWrite(_encoderPinA, HIGH);
		pinMode(_encoderPinB, INPUT);
		digitalWrite(_encoderPinB, HIGH);
		pinMode(_encoderPinP, INPUT);
		digitalWrite(_encoderPinP, HIGH);

		_position = 0; 
                _pushed = false;
                _step=0;
                _output=2;
                _skipUp = false;
        _state=R_START;
	}

	RotaryEncoderStatus read(void)
	{ 

        uint8_t currPinA = ! digitalRead(_encoderPinA);
	    uint8_t currPinB = ! digitalRead(_encoderPinB);

        unsigned char pinstate = (currPinB << 1) | currPinA;


		_state = pgm_read_byte(&(hs_ttable[_state & 0xf][pinstate]));
		//_state = pgm_read_byte(&(ttable[_state & 0xf][pinstate]));
    	// Get emit bits, ie the generated event.

	    uint8_t dir = _state & 0x30;

	    if(dir){
		    return (dir==DIR_CW)? RotaryEncoderStatusBackward:RotaryEncoderStatusFordward;
	    }

        bool pushed=!digitalRead(_encoderPinP);
		if (_pushed !=  pushed){
                   //DBG_println("sw");
            _pushed= pushed;
            if(pushed){
                if((millis() - _pressedTime) <  200)  return RotaryEncoderStatusNone; // debounce
                _pressedTime = millis();
            }
            if(_skipUp && !_pushed){
                    _skipUp = false;
            }else{
                return pushed?  RotaryEncoderStatusPushed:RotaryEncoderStatusDepushed;
            }
        }else{
            if(_pushed && !_skipUp){
                if((millis() - _pressedTime) >= LongPressedTime){
                    _skipUp = true;
                    return RotaryEncoderStatusLongPressed;
                }
            }
        }
        return RotaryEncoderStatusNone;
	}

};
