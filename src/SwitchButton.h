// maybe interrupt mode...

#define DebounceTime 15

void switchPressed1 ();
void switchPressed2 ();

class SwitchButton{
    int _pin;
    bool _pressed; // physical status
    
    bool _logicalPressed;  // current status

    uint32_t _changedTime;

    static SwitchButton* objs[2];
    static uint8_t isrCount;
public:
    SwitchButton(){}
    ~SwitchButton(){}

    uint8_t alloctIsr(){
        if(isrCount < 2){
            objs[isrCount]=this;
            isrCount ++;
            return isrCount;
        }
        // should alert. ....
        return isrCount;
    }

    void begin(int pin){ 
        _pressed=false;
        _logicalPressed=false;
        _changedTime=0;
        _pin = pin;
		pinMode(_pin, INPUT_PULLUP);
		digitalWrite(_pin, HIGH);

        if(alloctIsr() == 1){
            attachInterrupt(digitalPinToInterrupt(_pin), switchPressed1, CHANGE);
        }else{
            attachInterrupt(digitalPinToInterrupt(_pin), switchPressed2, CHANGE);
        }
    }

    bool pressed(){ return _logicalPressed; }

    bool statusChanged(){
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

    void isrHandler(){
        _pressed = digitalRead(_pin) == LOW;
        _changedTime= millis();
    }
    static void changeIsrHandler(uint8_t idx){
        objs[idx]->isrHandler();
    }
};

void switchPressed1()
{
    SwitchButton::changeIsrHandler(0);
}  // end of switchPressed

void switchPressed2()
{
    SwitchButton::changeIsrHandler(1);
}  // end of switchPressed

uint8_t SwitchButton::isrCount=0;
SwitchButton* SwitchButton::objs[2];

SwitchButton switchButton1;

SwitchButton switchButton2;