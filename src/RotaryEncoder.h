#define LongPressedTime 1000

typedef enum{
  RotaryEncoderStatusNone,
  RotaryEncoderStatusPushed,
  RotaryEncoderStatusDepushed,
  RotaryEncoderStatusLongPressed,
  RotaryEncoderStatusFordward,
  RotaryEncoderStatusBackward
}RotaryEncoderStatus;

class RotaryEncoder
{
  private:
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
	}

	RotaryEncoderStatus read(void)
	{ 
		byte pos = (digitalRead(_encoderPinB) * 2) + digitalRead(_encoderPinA);;
		if (pos != _position)
		{
			bool isFwd = ((_position == 0) && (pos == 1)) || ((_position == 1) && (pos == 3)) || 
				((_position == 3) && (pos == 2)) || ((_position == 2) && (pos == 0));
		        _position = pos;
                        if(isFwd) 
                       {
                          _step ++;
                          if(_step >= _output)
                          {
                              _step =0;
                              return RotaryEncoderStatusFordward;
                          }
                       }
                       else
                      {
                         _step--;
                         if(0-_step >= _output)
                         {
                             _step=0;
                             return RotaryEncoderStatusBackward;
                         }
                      }
		}

                bool pushed=!digitalRead(_encoderPinP);
		if (_pushed !=  pushed)
                {
                   //DBGPrintln("sw");
                    _pushed= pushed;
                    if(pushed)
                    {
                        if((millis() - _pressedTime) <  200)  return RotaryEncoderStatusNone; // debounce
                       _pressedTime = millis();
                    }
                    if(_skipUp && !_pushed)
                    {
                        _skipUp = false;
                    }
                    else
                    return pushed?  RotaryEncoderStatusPushed:RotaryEncoderStatusDepushed;
                }else
                {
                    if(_pushed && !_skipUp)
                    {
                        if((millis() - _pressedTime) >= LongPressedTime)
                        {
                           _skipUp = true;
                           return RotaryEncoderStatusLongPressed;
                        }
                    }
                }
                return RotaryEncoderStatusNone;
	}

};
