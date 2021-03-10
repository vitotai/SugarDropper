// maybe interrupt mode...

#define DebounceTime 15

void switchPressed ();

class SwitchButton{
    static int _pin;
    static bool _pressed; // physical status
    
    static bool _logicalPressed;  // current status

    static uint32_t _changedTime;
public:
    SwitchButton(){}
    ~SwitchButton(){}

    static void begin(int pin){ 
        _pin = pin;
		pinMode(_pin, INPUT_PULLUP);
		digitalWrite(_pin, HIGH);
        attachInterrupt(digitalPinToInterrupt(_pin), switchPressed, CHANGE);
    }

    static bool pressed(){ return _logicalPressed; }

    static bool statusChanged(){
        if( _logicalPressed == _pressed){
            // status unchanged. fine
            return false;
        }
        // button status changed
        uint32_t present = millis();
        if(present - _changedTime > DebounceTime){
            // ok. over debounce time.
            _logicalPressed = _pressed;
            return true;
        }
        DBGPrint("logical:");
        DBGPrint(_logicalPressed);
        DBGPrint("physical:");
        DBGPrint(_pressed);
        DBGPrint("time:");
        DBGPrintln(_changedTime);
        // not long enough. wait..
        return false;
    }

    static void changeIsrHandler(){
        _pressed = digitalRead(_pin) == LOW;
        _changedTime= millis();
    }
};

void switchPressed ()
{
    SwitchButton::changeIsrHandler();
}  // end of switchPressed

int SwitchButton::_pin;
bool SwitchButton::_pressed=false;
bool SwitchButton::_logicalPressed=false;
uint32_t SwitchButton::_changedTime=0;