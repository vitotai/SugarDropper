
#pragma once
#include <Arduino.h>
#include <EEPROM.h>
#include <avr/wdt.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>


#define DEBUG_OUT false

#if DEBUG_OUT
#define DBGPrint(...) Serial.print(__VA_ARGS__)
#define DBGPrintln(...) Serial.println(__VA_ARGS__)
#else
#define DBGPrint(...)
#define DBGPrintln(...) 
#endif

#include "RotaryEncoder.h"
#include "SwitchButton.h"
#include "Stepper.h"
#include "mystrlib.h"

#define AdjustUnit 0.01
#define DoseAdjustUnit 0.05
#define MaximumAmount 99.0
#define MinimumAmount 0.5
#define MaximumCalibrationCount 100

#define RA_CLK_PIN 9
#define RB_DT_PIN 10
#define SW_PIN 11

#define BUTTON_PIN 2

#define STEP_PIN 5
#define DIR_PIN 6

#define SCL_PIN A5
#define SDA_PIN A4

#define BUZZ_PIN 12

#define MinimumGapBetweenDrop 1000
#define CalibrationDropDelay 3000

#define BeepButton 100
#define BeepDoseStart 300
#define BeepDoseEnd 550

#define BlinkingShowTime 600
#define BlinkingHideTime 400

typedef enum _SugarAppId{
    SugarAppMenu,
    SugarAppAutomatic,
    SugarAppManual,
    SugarAppDoseCalibration,
    SugarAppCalibration,
    SugarAppDosingSettingMode,
    SugarAppSoundSetting,
    SugarAppUnitSetting,
    SugarAppPriming
} SugarAppId;


/*********************************************************************************/
// LCD routines
//
#define DrosingSymbolChar 1
#define RevDrosingSymbolChar 2
#define DosingSymbolNone ' '
#define DegreeChar 0b11011111
LiquidCrystal_I2C *lcd;



#define LcdColNumber 16

byte DropBitmap[8] = {
	0b00100,
	0b00100,
	0b01110,
	0b01110,
	0b11111,
	0b11111,
	0b01110,
	0b00000
};

byte EmptyDropBitmap[8] = {
	0b00100,
	0b00100,
	0b01010,
	0b01010,
	0b10001,
	0b10001,
	0b01110,
	0b00000
};


void lcdInitialize(){
    	DBGPrintln("Scanning I2C\n");
	    Wire.begin();
        byte lcdaddress = 0x27;
	    byte error, address;
        //DBGPrintln("Scan LCD Address...\n");

 	    for(address = 127; address > 0; address-- )
  	    {
    	    // The i2c_scanner uses the return value of
    	    // the Write.endTransmisstion to see if
    	    // a device did acknowledge to the address.

    		Wire.beginTransmission(address);
        	error = Wire.endTransmission();

        	if (error == 0)
        	{			
      	    	DBGPrint(F("LCD found at address:0x"));
                DBGPrintln(address,16);
                lcdaddress= address;
                break;
    	    }
        }

        lcd = new LiquidCrystal_I2C(lcdaddress,20,4,YWROBOT);
        lcd->begin(20,4);
        lcd->clear();
        lcd->createChar(DrosingSymbolChar,DropBitmap);
        lcd->createChar(RevDrosingSymbolChar,EmptyDropBitmap);
        lcd->noCursor();
}
void lcdPrintFixedSpace(byte col, byte row,byte lead,const char* str){
    lcd->setCursor(col,row);
    for(byte i=0;i<lead;i++) lcd->write(' ');
    const char* p=str;
    while(*p){
        lcd->write(*p);
        p++;
    }
}
void lcdWriteAt(byte col, byte row,char ch){
    lcd->setCursor(col,row);
    lcd->write(ch);
}
void lcdPrintAt(byte col, byte row,float value,byte space, byte precision){
    char buff[16];
    byte len=sprintFloat(buff,value,precision);
    buff[len]='\0';
    lcdPrintFixedSpace(col,row,space - len,buff);
}

void lcdPrintAt(byte col,byte row, int val, byte space){
    char buff[16];
    byte len=sprintInt(buff,val);
    buff[len]='\0';
    lcdPrintFixedSpace(col,row,space - len,buff);
}

byte lcdPrint_P(byte col,byte row,const char* p,bool clear2End=false)
{
    lcd->setCursor(col, row);
	byte i=0;
	char ch;
	while((ch=(char)pgm_read_byte_near(p + i))!=0)
	{
  		lcd->write(ch);
  		i++;
  	}
    if(clear2End){
        while(i< LcdColNumber){
            lcd->write(' ');
            i++;
        }
    }
	
    return i;
}

byte lcdClear(byte col,byte row,byte number){
    lcd->setCursor(col,row);
    for(byte i=0;i<number;i++) lcd->write(' ');
    return number;
}
void  lcdClearLine(byte row){
    lcdClear(0,row,LcdColNumber);
}

void lcdDosingSymbol(char sym){
    lcd->setCursor(15,0);
    lcd->write(sym);
}
inline void lcdSetCursor(byte col, byte row){
    lcd->setCursor(col,row);
}
inline void lcdWrite(char ch){
    lcd->write(ch);
}
/**********************************************************************************/
// Blinking Text
class EditingTextClass{
public:
    EditingTextClass():_blinking(false){}
    ~EditingTextClass(){}

    void setNumber(uint8_t col, uint8_t row,int number,uint8_t space){
         _col=col; _row = row;
        _len=sprintInt(_number,number);
        _number[_len]='\0';
        _space = space;
        _isText=false;
    }

    void setNumber(uint8_t col, uint8_t row,float number,uint8_t space,uint8_t precision){
         _col=col; _row = row;
        _len=sprintFloat(_number,number,precision);
        _number[_len]='\0';
        _space =space;
        _isText=false;
    }

    void setText_P(uint8_t col, uint8_t row,const char* text){
         _col=col; _row = row;
        _isText=true;
        _text = text;
        _space = strlen_P(text);
    }

    void blink(){
        if(_blinking) return;
        _hiding =false;
        _blinking=true;
    }
    
    void noblink(){
        if(!_blinking) return;
        _blinking=false;
        if(_hiding) show();
    }
    
    void show(){
        if(_isText){
            lcdPrint_P(_col,_row,_text);
        }else{

            lcdSetCursor(_col,_row);
            uint8_t lead = _space - _len;
            for(int i=0;i<lead;i++) lcdWrite(' ');
            const char* p=_number;
            while(*p){
                lcd->write(*p);
                p++;
            }
        }
        _hiding =false;
    }
    
    void hide(){
        lcdSetCursor(_col,_row);
        for(int i=0;i<_space;i++) lcdWrite(' ');
        _hiding =true;
    }

    void loop(){
        if(!_blinking) return;
        if(_hiding){
            if(millis() - _blinkTime > BlinkingHideTime){
                show();                
                _blinkTime = millis();
            }
        }else{
            if(millis() - _blinkTime > BlinkingShowTime){
                hide();
                _blinkTime = millis();
            }
        }

    }
protected:
    uint32_t _blinkTime;
    bool _blinking;
    bool _hiding;
    uint8_t _col;
    uint8_t _row;

    bool _isText;
    const char *_text;
    char _number[17];
    uint8_t _space;
    uint8_t _len;
}EditingText;

/*********************************************************************************/
// Priming sugar calculator
// PS = 15.195 VB (CD- 3.0378 + 5.0062 X 1o·2 T - 2.6555 X 1o·4 T2) 
// PS: in gram
// VB: volume of beer, in gallon
// CD: Carbonation Desired
// T:  temberature of beer, in Farenheight

// input CD= volume
//  beetAmount = beer volume in liter
//  temperature = beer temperature in Celisus
float calculatePrimingSugar(float CD, float beerAmount, float temperature){
    
    float VB = 0.264172052 * beerAmount;  // to Gallon
    float T = 32 + temperature * 9/5; ; // to F
     
    return 15.195 * VB * ( CD - 3.0378 + 0.050062 * T - 0.00026555 * T * T);
}
/********************************************************************************/
//     SG = (Brix / (258.6-((Brix / 258.2)*227.1))) + 1

float brix2SG(float Brix){
    return (Brix / (258.6-((Brix / 258.2)*227.1))) + 1;
}
/*********************************************************************************/
// Settings

#define FootprintPattern 0x5B5B5B5B


struct _Settings{
    uint32_t footPrint;
    float stepPerMl;
    float shotAdjustment;
    uint8_t trigerType;
    uint8_t delayTime;  
    uint8_t coolTime;
    // the following fields musht not changed orders.
    uint8_t beepButton;
    uint8_t beepDoseStart;
    uint8_t beepDoseEnd;
    uint8_t useWeight;
    // for priming sugar calculation
    uint8_t inputBeer;
    uint8_t sugarRatio; // in brix
    int8_t   beerTemperature;
    uint8_t  carbonation; // in volume * 10
} Settings;

#define Set2CV(v) (float)(v)/10.0
#define CV2Set(v) (uint8_t)((v)*10)


class SettingManagerClass{
public:
    SettingManagerClass(){}
    ~SettingManagerClass(){}

    bool begin(){
        EEPROM.get(0,Settings);
        if(Settings.footPrint != FootprintPattern){
            Settings.footPrint = FootprintPattern;
            Settings.shotAdjustment =0;
            Settings.trigerType =0;
            Settings.delayTime =1;
            Settings.coolTime = 2;

            DBGPrintln("uninitialized data.");
            return false;
        }
        return true;
    }
    void save(){
        EEPROM.put(0,Settings);
    }
};

SettingManagerClass SettingManager;

/*********************************************************************************/
//  Buzzer

class BuzzerClass{
public:
    BuzzerClass(uint8_t pin){
        _pin=pin;
    }
    ~BuzzerClass(){}
    
    void begin(){
        _buzzing = false;
        pinMode(_pin, OUTPUT);
        digitalWrite(_pin,LOW);
    }

    void loop(){
        if(_buzzing){
            if(millis() >= _stop){
                 digitalWrite(_pin,LOW);
                 _buzzing = false;                 
            }
        }
    }

    void buzz(uint32_t duration){
        _stop = millis() + duration;
        _buzzing = true;
        digitalWrite(_pin,HIGH);
    }
protected:
    uint8_t _pin;
    bool    _buzzing;
    uint32_t _stop;
}Buzzer(BUZZ_PIN);

/*********************************************************************************/
//  Doser control
class SugarDoser{
public:
    SugarDoser():_stepper(STEP_PIN,DIR_PIN){}
    ~SugarDoser(){}

    void begin(){
//        _stepper.setRPM(100);
        _stepper.begin();
    }
    
    bool running(){
        return _stepper.running();
    }

    void dose(float amount){
        if(_stepper.running()) return;

        float adjustedValue =amount + Settings.shotAdjustment;
        uint32_t steps =(uint32_t)(adjustedValue * Settings.stepPerMl);

        _startDosingPos=_stepper.runSteps( steps );

        DBGPrint("\toutput");
        DBGPrint(amount,2);
        DBGPrint(F(" corrected:"));
        DBGPrint(adjustedValue);
        DBGPrint(F("ml as steps:"));
        DBGPrintln(steps);
    }

    void run(){
        _startDosingPos=_stepper.run();
        DBGPrintln(F("\t flow start"));
    }

    void stop(){
        _stepper.stop();
        DBGPrintln(F("\t flow stop"));
    }
    void startCalibrate(){
        _startCalPos = _stepper.steps();
        DBGPrint(F("*start steps: "));
        DBGPrintln(_startCalPos);

    }

    void calibrate(float amount){
        uint32_t steps = _stepper.steps();
        Settings.stepPerMl = (float)(steps - _startCalPos) / amount;
        SettingManager.save();

        DBGPrint(F("*stop steps: "));
        DBGPrintln(steps);

        DBGPrint(F("*Calibrate to "));
        DBGPrint(amount,2);
        DBGPrint(F("ml steps/ml:"));
        DBGPrintln(Settings.stepPerMl ,2);
    }
    void resetDoseAdjustment(){
        Settings.shotAdjustment = 0;
    }
    void setDoseAdjustment(float value){
        Settings.shotAdjustment = value;
        SettingManager.save();
        DBGPrint(F("update shot adjustment:"));
        DBGPrintln(value);
    }
    // for statistic information
    float getDosingVolume(){
        // calculate running since latest start
        uint32_t steps = _stepper.steps();
//        DBGPrint(F("run steps:"));
//        DBGPrintln(steps - _startDosingPos);
        return (float)(steps - _startDosingPos) / Settings.stepPerMl;
    }

protected:
    Stepper _stepper;
    uint32_t _startCalPos;

    uint32_t _startDosingPos;
};

enum DosingMode{
DosingModeDisabled,
DosingModeSingleShot,
DosingModeManual
};

enum DosingState{
DSIdle,
DSPrepareToDose,
DSDosing,
DSCoolTime
};

class DosingController{
public:
    DosingController():_dosing(false){}
    ~DosingController(){}

    void setDosage(float dosage){
        _dosage = dosage;
    }

    void setMode(DosingMode mode){
        _mode = mode;
        if(_mode == DosingModeDisabled) {
            if(_doser.running()){
                _doser.stop();
            }
        }else{
            _state = DSIdle;
        }
    }
    
    void begin(){
        _mode = DosingModeDisabled;
        loadSetting();
        _doser.begin();
    }
    
    void loadSetting(){
        _dosingDelay = Settings.delayTime * 500;
        _coolTime = Settings.coolTime * 500;
        _isPositionSensor = Settings.trigerType !=0;
    }

    // most be call every loop
    bool isDosingStateChanged(){
        bool ret=false;
        if(_isPositionSensor){
            if(SwitchButton::statusChanged()){
                _handleSensorState(SwitchButton::pressed());
            }
        }else{
            if(_mode != DosingModeDisabled){
                if(SwitchButton::statusChanged()){
                     _handleButtonAction(SwitchButton::pressed());
                }
            }
        }

        if(_dosing != _doser.running()){
            _dosing = _doser.running();
            _processStateForDropingStateChange(_dosing);
            if(_dosing){
                if(Settings.beepDoseStart) Buzzer.buzz(BeepDoseStart);
            }else{
                if(Settings.beepDoseEnd)  Buzzer.buzz(BeepDoseEnd);
            }
            ret =true;
        }

        _processStateForTime();
        return ret;
    }
    bool isDosing(){
        return _dosing;
    }
    // for calibration/programming purpose
    void dose(){
        lcdDosingSymbol(DrosingSymbolChar);
        _doser.dose(_dosage);
        _state = DSDosing;
    }

    inline void resetDoseAdjustment(){
        _doser.resetDoseAdjustment();
    }
    
    inline void setDoseAdjustment(float val){
        _doser.setDoseAdjustment(val);
    }

    inline void startCalibrate(){
        _doser.startCalibrate();
    }

    inline void calibrate(float vol){
        _doser.calibrate(vol);
    }

    inline float getDosingVolume(){
        return _doser.getDosingVolume();
    }
protected:
    SugarDoser _doser;

    bool _dosing;
    float _dosage;
    DosingMode _mode;
    DosingState _state;
    bool       _isPositionSensor;
    uint32_t   _dosingDelay;
    uint32_t   _coolTime;

    uint32_t   _timeToAction;

    void _handleSensorState(bool inPosition){
        if(inPosition){
            if(_mode == DosingModeSingleShot || _mode == DosingModeManual){
                if(_state==DSIdle){
                   if(Settings.beepButton) Buzzer.buzz(BeepButton);
                    lcdDosingSymbol(RevDrosingSymbolChar);
                    _timeToAction = millis() + _dosingDelay;
                    _state = DSPrepareToDose;
                }
            }
        }else{
            //out of position. stop any way
            
            if(_doser.running()){
                _doser.stop();
                DBGPrintln(F("Forced Stop"));
            }else if (_state==DSPrepareToDose){
                _state=DSIdle;
                lcdDosingSymbol(DosingSymbolNone);
                DBGPrintln(F("Forced Stop"));
            }
        }
    }

    void _handleButtonAction(bool pressed){
            // care only about pressed.
        if(!pressed) return;

        if(_state==DSIdle){
            if(_mode == DosingModeSingleShot || _mode == DosingModeManual){
                if(Settings.beepButton) Buzzer.buzz(BeepButton);
                lcdDosingSymbol(RevDrosingSymbolChar);

                _timeToAction = millis() + _dosingDelay;
                _state = DSPrepareToDose;
            }
        }else if(_state==DSDosing){
            if(_mode == DosingModeManual){
               if(Settings.beepButton) Buzzer.buzz(BeepButton);
                // stop
                _doser.stop();
                // state change will be process when
                // dosing end dectected
            }
        }
    }

    void _processStateForDropingStateChange(bool dosing){
        if(! dosing){ // dosing end
            _timeToAction = millis() + _coolTime;
            if(_mode == DosingModeDisabled){
                lcdDosingSymbol(DosingSymbolNone);
                _state = DSIdle;
            }else{
                lcdDosingSymbol(RevDrosingSymbolChar);
                _state = DSCoolTime;
            }
            
        }
    }

    void _processStateForTime(){
        uint32_t present = millis();
        if(_state == DSPrepareToDose){
            if(present >= _timeToAction){
                if(_mode == DosingModeSingleShot){
                    lcdDosingSymbol(DrosingSymbolChar);
                    _doser.dose(_dosage);
                }else{
                    lcdDosingSymbol(DrosingSymbolChar);
                    _doser.run();
                }
                _state = DSDosing;
            }
        }else if(_state == DSCoolTime){
            if(present >= _timeToAction){
                _state = DSIdle;
                lcdDosingSymbol(DosingSymbolNone);
            }
        }
    }

} dosingController;




/*********************************************************************************/
// Apps

class SugarBaby{
public:
    SugarBaby(){}
    virtual ~SugarBaby(){}

    virtual void show(){}
    virtual void loop(){}
    // UI related
    virtual void rotateForward(){}
    virtual void rotateBackward(){}
    virtual bool switchPushed(){return true;}
    // on exit
    virtual SugarAppId nextApp(){return SugarAppMenu;}

//    virtual void buttonStatus(bool){}
//    virtual void dropEndded(){}
    virtual void dosingStateChanged(bool){}
};

/*********************************************************************************/
// Manu handler
const char strAutomatic[] PROGMEM  ="Automatic";
const char strManual[] PROGMEM ="Manual";
const char strCalibration[] PROGMEM ="Calibration";
const char strShotCalibration[] PROGMEM ="Dose calibrate";
const char strDropSettings[] PROGMEM ="Triger Settings";
const char strBack[]  PROGMEM ="Back";
const char strSoundSetting[] PROGMEM="Sound Setting";
const char strSetting[]  PROGMEM="Setting";
const char strMl[] PROGMEM = "ml";
const char strUnitSetting[] PROGMEM ="Unit";
const char strFunction[] PROGMEM ="Function";
const char strAutoDoseSettings[] PROGMEM="Priming";
struct MenuList;

struct MenuItem{
    const char * const title;
    bool isSubMenu;
    union {   
        SugarAppId mode;
        const MenuList *subMenu;
    };
} ;

struct MenuList{
    const char * const title;
    uint8_t numberOfItem;
    const MenuItem *menuItems;
};

extern const MenuList MainMenu;

const MenuItem SettingMenuItems[]={
    {strCalibration, false,{ .mode=SugarAppCalibration }},
    {strShotCalibration,false,{ .mode=SugarAppDoseCalibration}},
    {strDropSettings, false, { .mode=SugarAppDosingSettingMode }},
    {strAutoDoseSettings, false, { .mode=SugarAppPriming }},
    {strSoundSetting, false, {.mode=SugarAppSoundSetting}},
    {strUnitSetting, false, {.mode=SugarAppUnitSetting}},
    {strBack, true, { .subMenu= &MainMenu }}
};

const MenuList SettingMenu={
    strSetting,
    sizeof(SettingMenuItems)/sizeof(MenuList),
    SettingMenuItems
};

const MenuItem mainMenuItems[]  ={
    {strAutomatic,false, {.mode = SugarAppAutomatic}},
    {strManual, false, {.mode =SugarAppManual}},
    {strSetting, true, {.subMenu = &SettingMenu}}
};

const MenuList MainMenu={
    strFunction,
    sizeof(mainMenuItems)/sizeof(MenuItem),
    mainMenuItems
};

/*
Display
0123456789012345
Function:
   Inject
   Cleaning
   Calibration
   Setup
*/

class MenuHandler:public SugarBaby{
private:
    byte _selected;
public:
    MenuHandler(const MenuList *menu):_selected(0){
        _menu = menu;
    }
    ~MenuHandler(){}

    void show(){
        // print display
        lcdPrint_P(0,0,_menu->title,true);
        _printItem();
    }

    void rotateBackward(){
        
        if(_selected < _menu->numberOfItem -1){
            _selected ++;
            _printItem();
        }
    }

    void rotateForward(){
        if(_selected >0){
             _selected --;
            _printItem();
        }
    }
    
    bool switchPushed(){
        if (_menu->menuItems[_selected].isSubMenu){
            _menu = _menu->menuItems[_selected].subMenu;
            _selected=0;
            show();
            return false;
        }
        return true;
    }
        
    SugarAppId nextApp(){ 
        return _menu->menuItems[_selected].mode; 
    }
    
protected:
    const MenuList *_menu;

    void _printItem(){
        int i=2;
        i+= lcdPrint_P(i,1,_menu->menuItems[_selected].title,true);
    }
};


/*********************************************************************************/
// Automatic dosing
/*
0123456789012345
Automatic   999x 
 999.25  199.25ml
*/

/*
0123456789012345
Automatic   999x 
A:999.25  99.25ml


v2
0123456789012345
# 999    1330mlX 
A:999.25 99.25g

0123456789012345
# 999          X 
A:999.25 99.25g

0123456789012345
XQ99.99  1330mlX          
099/999  99.25g
*/

const char strTotal[]  PROGMEM="A:";

#define DosageAmountRow 1
#define DosageAmountCol 9
#define DosageAmountSpace 5

#define BeerVolumeRow 0
#define BeerVolumeCol 9
#define BeerVolumeSpace 4

#define DosageCountRow 0
#define DosageCountCol 2
#define DosageCountSymbolCol 0
#define DosageCountSapce 3

#define AccumulatedOutputRow 1
#define AccumulatedOutputCol 2
#define AccumulatedOutputStringCol 0
#define AccumulatedOutputSpace 6

#define MinimumBeerVolume 50
#define DefaultBeerVolume  330
#define MaximumBeerVolume 5000

class AutoDoser:public SugarBaby{
public:
    AutoDoser(){
        _totalAmount = 0;
        _count =0;
    }

    ~AutoDoser(){}

    void show(){
        if(Settings.inputBeer){
            if(_beerVolume < MinimumBeerVolume ||_beerVolume > MaximumBeerVolume) _beerVolume = DefaultBeerVolume;
        }else{
            if(_amount < MinimumAmount || _amount > MaximumAmount) _amount = 5;
        }

        // "A:"
        lcdPrint_P(AccumulatedOutputStringCol,AccumulatedOutputRow,strTotal);
        // dosage unit
        if(Settings.useWeight) lcdWriteAt(DosageAmountCol + DosageAmountSpace,DosageAmountRow,'g');
        else lcdPrint_P(DosageAmountCol + DosageAmountSpace,DosageAmountRow,strMl);
        // "#"
        lcdWriteAt(DosageCountSymbolCol,DosageCountRow,'#');

        if(Settings.inputBeer){
            lcdPrint_P(BeerVolumeCol + BeerVolumeSpace,BeerVolumeRow,strMl);
            _updateBeerVolume();
            _calPrimingSugar();
        }

        _updateDosage();
        _updateCount();
        _updateTotal();

        dosingController.setMode(DosingModeSingleShot);
    }

    void rotateForward(){
        if(Settings.inputBeer){
            if( (_beerVolume +10) < MaximumBeerVolume){
                _beerVolume +=10;

                _updateBeerVolume();
                _calPrimingSugar();
                _updateDosage();
            }
        }else{
           _amount += DoseAdjustUnit;
            if(_amount > MaximumAmount) _amount = MaximumAmount;
            _updateDosage();
        }
    }
    
    void rotateBackward(){
        if(Settings.inputBeer){
            if( (_beerVolume -10) > MinimumBeerVolume){
                _beerVolume -=10;

                _updateBeerVolume();
                _calPrimingSugar();
                _updateDosage();
            }
        }else{
            _amount -= DoseAdjustUnit;
            if(_amount <MinimumAmount) _amount = MinimumAmount;
            _updateDosage();
        }
    }

    void dosingStateChanged(bool dosing){
        if(dosing){
            _count ++;
            _updateCount();
        }else{
            // udpateTotal, suppose it _amount ++, but the amount might changed. so...
            _totalAmount += dosingController.getDosingVolume();
            _updateTotal();
        }
    }

protected:
    float _amount;
    float _totalAmount;
    uint16_t _count;
    uint16_t _beerVolume;

    void _calPrimingSugar(){
        //
        DBGPrint(F("CD:"));
        DBGPrint(Set2CV(Settings.carbonation));
        DBGPrint(F(" beerTemperature:"));
        DBGPrintln(Settings.beerTemperature);

        float ps=calculatePrimingSugar(Set2CV(Settings.carbonation),(float)_beerVolume/1000.0,(float)Settings.beerTemperature);
        float weight = ps / (float)Settings.sugarRatio * 100.0;
        // if the unit is volume, calculate SG and derive volume
        // however, temperature might be a problem. ignore that for now
        DBGPrint(F("PS:"));
        DBGPrint(ps);
        DBGPrint(F(" weight:"));
        DBGPrintln(weight);

        if(Settings.useWeight){
            _amount = weight;
        }else{
            DBGPrint(F("SG:"));
            DBGPrintln(brix2SG((float)Settings.sugarRatio));

            _amount = weight / brix2SG((float)Settings.sugarRatio);
        }
    }

    // LCD display
    void _updateCount(){
        lcdPrintAt(DosageCountCol,DosageCountRow,_count,DosageCountSapce);
    }
    void _updateTotal(){
        lcdPrintAt(AccumulatedOutputCol,AccumulatedOutputRow,_totalAmount,AccumulatedOutputSpace,2);
    }
    void _updateDosage(){
        DBGPrint(F("Sugar amount:"));
        DBGPrintln(_amount);
        lcdPrintAt(DosageAmountCol,DosageAmountRow,_amount,DosageAmountSpace,2);
        dosingController.setDosage(_amount);
    }
    void _updateBeerVolume(){
        DBGPrint(F("Beer Vol:"));
        DBGPrintln(_beerVolume);
        lcdPrintAt(BeerVolumeCol,BeerVolumeRow,_beerVolume,BeerVolumeSpace);
    }
};


/*********************************************************************************/
// manual dosing

/*
0123456789012345
Manual
100.1s  999.12ml
*/

#define MinimumUpdateTime 100

class ManualDoser:public SugarBaby{
public:
    ManualDoser(){}
    ~ManualDoser(){}

    void show(){
        lcdPrint_P(0,0,strManual,true);
        if(Settings.useWeight) lcdWriteAt(14,1,'g');
        else lcdPrint_P(14,1,strMl);
        lcd->setCursor(5,1);
        lcd->write('s');
        _accumulatedTime=0;
        _updateInfo(0,0);

        dosingController.setMode(DosingModeManual);
        _doserRunning = false;
        _accumulatedVolume = 0;
    }

    void loop(){
        if(_doserRunning){
            if( millis() - _lastUpdate > MinimumUpdateTime){
                _lastUpdate = millis();
                _updateInfo(_accumulatedTime +  millis() - _startedTime,
                    _accumulatedVolume + dosingController.getDosingVolume());
            }
        }
    }

    void dosingStateChanged(bool dosing){
        if(dosing){
            _startedTime = millis();
            _doserRunning =true;
        }else{
            _accumulatedVolume += dosingController.getDosingVolume();
            _doserRunning = false;
            _accumulatedTime += millis() - _startedTime;
            _updateInfo(_accumulatedTime,_accumulatedVolume);
        }
     }

protected:
    uint32_t _lastUpdate;
    uint32_t _accumulatedTime;
    uint32_t _startedTime;
    float    _accumulatedVolume;
    bool     _doserRunning;

    void _updateInfo(uint32_t duration,float volume){
        lcdPrintAt(0,1,(float) duration/1000.0,5,1);        
//        DBGPrint("accvol:");
//        DBGPrintln(volume);
        lcdPrintAt(7,1,volume ,7,2);
    }
};


/*********************************************************************************/
// initial calibration/setup
//  to derive steps/ml
typedef enum _CalibrationMode{
Cal_SelectVolume,
Cal_RunDoser,
Cal_InputVolume,
Cal_DisplayResult
}CalibrationMode;

/*
0123456789012345
Setup
cal by 100.24 ml
*/

const char strCalBy[] PROGMEM="cal by";
const char strAdjust[] PROGMEM="adjust";
const char strRunDoser[] PROGMEM="Run Doser";
const char strEnter[] PROGMEM="Continue";
const char strRate[] PROGMEM="Rate";
const char strMlPerSec[] PROGMEM="ml/s";
const char strGramPerSec[] PROGMEM="g/s";

class SugarCalibrator:public SugarBaby{
public:
    SugarCalibrator(){}
    ~SugarCalibrator(){}

    void show(){
        _mode =Cal_SelectVolume;
        _calVolume = 10;
        lcdPrint_P(0,0,strCalibration,true);
        lcdPrint_P(0,1,strCalBy);
        if(Settings.useWeight) lcdWriteAt(14,1,'g');
        else lcdPrint_P(14,1,strMl);

        _updateVolumeDisplay(_calVolume);
    }
    

    void rotateForward(){
        if(_mode == Cal_SelectVolume){
            if(_calVolume >=10) _calVolume += 10;
            else _calVolume += 5;
            _updateVolumeDisplay(_calVolume);
        }if(_mode == Cal_InputVolume){
            _realVolume += AdjustUnit;
            _updateVolumeDisplay(_realVolume);
        }
    }
    
    void rotateBackward(){
        if(_mode == Cal_SelectVolume){
            if(_calVolume >10) _calVolume -= 10;
            else _calVolume -= 5;
            if(_calVolume < 5) _calVolume = 5;
            _updateVolumeDisplay(_calVolume);            
        }else if(_mode == Cal_InputVolume){
            _realVolume -= AdjustUnit;
            _updateVolumeDisplay(_realVolume);
        }
    }

    bool switchPushed(){
        if(_mode == Cal_SelectVolume){
            _mode = Cal_RunDoser;
            _enterCalibratingState();
        }else if(_mode == Cal_RunDoser){
            if(_dosed && ! _dosing){
              _mode = Cal_InputVolume;
              _enterInputVolumeState();
              // what if "running now"?
            }
        }else if(_mode == Cal_InputVolume){            
            EditingText.noblink();
            _finishCalibrate();
            SettingManager.save();
            // display the result
            _showResult();
            _mode =Cal_DisplayResult;
        } else if(_mode == Cal_DisplayResult){
            return true;
        }
        return false;
    }

    void dosingStateChanged(bool dosing){
        if(_mode != Cal_RunDoser) return;

        if(dosing){
            _startTime = millis();
            _dosed=true;
            _dosing = true;
        }else{
            EditingText.noblink();
            _dosing =false;
            _accTime += millis() - _startTime;
            lcdPrint_P(0,1,strEnter,true);
        }
    }
protected:
    CalibrationMode _mode;
    bool _dosed;
    bool _dosing;
    float _calVolume;
    float _realVolume;
    uint32_t _startTime;
    uint32_t _accTime;

    void _enterCalibratingState(){
        EditingText.noblink();
        _dosed=false;
        dosingController.resetDoseAdjustment();
        dosingController.startCalibrate();
        //lcdPrint_P(0,1,strRunDoser,true);
        lcdClearLine(1);
        EditingText.setText_P(0,1,strRunDoser);
        EditingText.blink();
        dosingController.setMode(DosingModeManual);
        _accTime =0;
    }

    void _enterInputVolumeState(){
        _realVolume = _calVolume;
        lcdPrint_P(0,1,strAdjust,true);
        if(Settings.useWeight) lcdWriteAt(14,1,'g');
        else    lcdPrint_P(14,1,strMl);

        _updateVolumeDisplay(_realVolume);
         dosingController.setMode(DosingModeDisabled);
    }

    void _updateVolumeDisplay(float vol){
        //lcdPrintAt(7,1,vol,6,2);
        EditingText.setNumber(7,1,vol,6,2);
        EditingText.blink();
    }

    void _finishCalibrate(){
        dosingController.calibrate(_realVolume);
    }
    //0123456789012345
    //Rate  099.34ml/s
    void _showResult(){
        lcdPrint_P(0,1,strRate,true);
        float rate =_realVolume / float(_accTime) * 1000;
        lcdPrintAt(6,1,rate,6,2);
        if(Settings.useWeight) lcdPrint_P(12,1,strGramPerSec);
        else lcdPrint_P(12,1,strMlPerSec);
    }
};

/*********************************************************************************/
// calibration
// micro adjustment
// to calculate Hysteresis by average
typedef enum _DoseCalibrationState{
    CS_SelectAmout,
    CS_SelectCount,
    CS_ReadyToRun,
    CS_Running,
    CS_Adjust,
    CS_Result
} DoseCalibrationState;

/*
0123456789012345
Calibrate by
 10.12 ml x  100
0123456789012345
Amount   10.12ml
Count        100

0123456789012345
Calibrating
       100 / 100

0123456789012345
Calibrated to
       200.11 ml
*/
const char strCalibrateBy[] PROGMEM="Calibrate by";
const char strCalibrating[] PROGMEM="Calibrating..";
const char strCalibratedTo[] PROGMEM="Calibrated to";

const char strAmount[] PROGMEM="Amount";
const char strCount[] PROGMEM="Count";
const char strRun[] PROGMEM="Run";
//const char strAdjust[] PROGMEM="Adjust";

class DoseCalibration:public SugarBaby{
public:
    DoseCalibration(){}
    ~DoseCalibration(){}

    void show(){
        _state = CS_SelectAmout;
        _amount = 5;
        _count = 10;
        lcdPrint_P(0,0,strCalibrateBy,true);
        // show Amount
        lcdPrint_P(0,1,strAmount);
        if(Settings.useWeight) lcdWriteAt(14,1,'g');
        else lcdPrint_P(14,1,strMl);

        _showCalAmount();
    }

    void rotateForward(){
        if(_state == CS_SelectAmout){
            _amount += AdjustUnit;
            _showCalAmount();
        }else if(_state == CS_SelectCount){
            if(_count < MaximumCalibrationCount){
                _count ++;
                _showTotalCount();
            }
        }else if(_state == CS_Adjust){
            _realAmount += AdjustUnit;
            _showAjustedAmount();
        }
    }

    void rotateBackward(){
        if(_state == CS_SelectAmout){
            if(_amount > AdjustUnit){
                _amount -= AdjustUnit;
                _showCalAmount();
            }
        }else if(_state == CS_SelectCount){
            if(_count >1 ){
                _count --;
                _showTotalCount();
            }
        }else if(_state == CS_Adjust){
            if(_realAmount > AdjustUnit){
                _realAmount -= AdjustUnit;
                _showAjustedAmount();
            }
        }
    }

    bool switchPushed(){
        if(_state == CS_SelectAmout){
            // show
            // Count    100
            EditingText.noblink();
            lcdPrint_P(0,1,strCount,true); //rare case            
            _showTotalCount();
            _state = CS_SelectCount;
        }else if(_state == CS_SelectCount){
            EditingText.noblink();
            // show
            //0123456789012345
            //Run 10.12ml *100
            // the "count" numer shold already be there
            // show extra volume
            lcdPrint_P(0,1,strRun);
            lcdPrintAt(3,1,_amount,6,2);
            if(Settings.useWeight) lcdWriteAt(9,1,'g');
            else lcdPrint_P(9,1,strMl);
            lcd->setCursor(12,1);
            lcd->write('*');

            _state = CS_ReadyToRun;
        }else if(_state == CS_ReadyToRun){
            _displayCalibrating();
           _state = CS_Running;
            _dropCount = 0;

            _dosing = true;
            dosingController.resetDoseAdjustment();
            dosingController.setDosage(_amount);
            dosingController.dose();
        }else if(_state == CS_Running){
            /* do nothing */
        }else if(_state == CS_Adjust){
            EditingText.noblink();
            _state = CS_Result;
            //  realAmount =  (_amount + x ) * _count
            // x = realAmount/_count - _amount
            // 
            //
            float adjust =_amount - _realAmount/(float)_count;
            dosingController.setDoseAdjustment( adjust );
            //0123456789012345
            //Adjsut   -0.12ml
            lcdPrint_P(0,1,strAdjust,true);
            lcdPrintAt(9,1,adjust,5,2);
            if(Settings.useWeight) lcdWriteAt(14,1,'g');
            else lcdPrint_P(14,1,strMl);
        }else if(_state == CS_Result){
            return true;
        }
        return false;
    }

    void dosingStateChanged(bool dosing){
        if(_state != CS_Running) return;
        if(dosing) return;

        _dropEndTime = millis();
        _dropCount ++;
        _showProgress();
        _dosing = false;
        if(_dropCount == _count ){
            _state = CS_Adjust;
            _realAmount = _amount * _count;
            _displayAdjusting();
        }
    }

    void loop(){
        if(_state != CS_Running) return;
        if(_dosing) return;
        // loop() might be called before dropEnded() is called.
        //  result in "wrong" _dropEndTime 

        if(millis() - _dropEndTime > CalibrationDropDelay ){
            _dosing = true;
            dosingController.dose();
        }
    }

protected:

/*
0123456789012345
Calibrate by
 10.12 ml x  100
*/
    void _showCalAmount(){
        //lcdPrintAt(8,1,_amount,6,2);
        EditingText.setNumber(8,1,_amount,6,2);
        EditingText.blink();
    }
    void _showTotalCount(){
        //lcdPrintAt(13,1,(int)_count,3);
        EditingText.setNumber(13,1,(int)_count,3);
        EditingText.blink();
    }

/*
0123456789012345
Calibrating
999.99ml 100/100
*/


    void _displayCalibrating(){
        lcdPrint_P(0,0,strCalibrating,true);
        lcdClear(0,1,11);
        lcd->setCursor(12,1);
        lcd->write('/');
        if(Settings.useWeight) lcdWriteAt(6,1,'g');
        else lcdPrint_P(6,1,strMl);
        lcdPrintAt(13,1,_count,3);
        _showProgress();
    }
    
    void _showProgress(){
        lcdPrintAt(0,1,_dropCount * _amount,6,2);
        lcdPrintAt(9,1, _dropCount,3);
    }
/*
0123456789012345
Calibrated to
       200.11 ml
*/

    void _displayAdjusting(){
        lcdPrint_P(0,0,strCalibratedTo,true);
        lcdClearLine(1);
        if(Settings.useWeight) lcdWriteAt(14,1,'g');
        else lcdPrint_P(14,1,strMl);
        _showAjustedAmount();
    }

    void _showAjustedAmount(){
        //lcdPrintAt(7,1,_realAmount,6,2);
        EditingText.setNumber(7,1,_realAmount,6,2);
        EditingText.blink();
    }
    uint32_t _dropEndTime;
    uint16_t _count;
    uint16_t _dropCount;
    float _amount;
    float _realAmount;
    DoseCalibrationState _state;
    bool _dosing;
};

/*********************************************************************************/
// dose setting, including
// delay start, cool time, trigger type
#define MaximumDelayTime 10
#define MaximumCoolTime 10   // in 0.5 seconds

const char strDosingControl[] PROGMEM ="Dosing Control";
const char strControl[] PROGMEM ="Control";
const char strDelay[] PROGMEM ="Delay";
const char strCoolTime[] PROGMEM ="CoolTime";

const char strButton[] PROGMEM ="Button";
const char strSensor[] PROGMEM ="Sensor";

enum TSState{
    TS_TrigerType=0,
    TS_DelayTime,
    TS_CoolTime,
    TS_Back
};
class TriggerSettings:public SugarBaby{
public:
    TriggerSettings(){}
    ~TriggerSettings(){}

    void show(){
        _editing =false;
        _state = TS_TrigerType;
        _displayItems();
    }
    
    void rotateForward(){
        if(_editing){
            if(_state == TS_TrigerType){
                if(Settings.trigerType==0){
                    Settings.trigerType=1;
                    _displayTrigerType();
                }
            }else if(_state == TS_DelayTime){
                if(Settings.delayTime < MaximumDelayTime){
                    Settings.delayTime ++;
                    _displayTime(Settings.delayTime);
                }
            }else if(_state == TS_CoolTime){
                if(Settings.coolTime < MaximumCoolTime){
                    Settings.coolTime ++;
                    _displayTime(Settings.coolTime);
                }
            }
        }else{
            // non editing
            if((int)_state > (int)TS_TrigerType){
                _state =(TSState) ((int)_state -1);
                _displayItems();
            }
        }
    }

    void rotateBackward(){
        if(_editing){
            if(_state == TS_TrigerType){
                if(Settings.trigerType!=0){
                    Settings.trigerType=0;
                    _displayTrigerType();
                }
            }else if(_state == TS_DelayTime){
                if(Settings.delayTime > 0){
                    Settings.delayTime --;
                    _displayTime(Settings.delayTime);
                }
            }else if(_state == TS_CoolTime){
                if(Settings.coolTime > 0){
                    Settings.coolTime --;
                    _displayTime(Settings.coolTime);
                }
            }

        }else{
            // non editing
            if((int)_state < (int)TS_Back){
                _state =(TSState) ((int)_state +1);
                _displayItems();
            }
        }

    }

    bool switchPushed(){
        if(_editing){
            _editing=false;
            EditingText.noblink();
        }else{
            if(_state == TS_Back){
                SettingManager.save();
                dosingController.loadSetting();
                return true;
            }else{
                _editing=true;
                EditingText.blink();
            }
        }
        return false;
    }
protected:
    TSState _state;
    bool _editing;

    void _displayItems(){

        if(_state == TS_TrigerType){
            lcdPrint_P(0,0,strDosingControl,true);
            lcdPrint_P(0,1,strControl,true);
            _displayTrigerType();

        }else if(_state == TS_DelayTime){
            lcdPrint_P(0,1,strDelay,true);
            lcd->setCursor(1,15);
            lcd->write('s');
            _displayTime(Settings.delayTime);            
        }else if(_state == TS_CoolTime){
            lcdPrint_P(0,1,strCoolTime,true);
            lcd->setCursor(1,15);
            lcd->write('s');
            _displayTime(Settings.coolTime);            
        }else{ // _state == TS_Back
            lcdPrint_P(0,1,strBack,true);
        }

    }

    void _displayTrigerType(){
        if(Settings.trigerType){
            // sensor
            EditingText.setText_P(10,1,strSensor);
        }else{
            EditingText.setText_P(10,1,strButton);
        }
        EditingText.show();
    }
//0123456789012345
//Delay       5.0s
    void _displayTime(uint8_t val){
        float fvalue = (float)val * 0.5;
        EditingText.setNumber(12,1,fvalue,3,1);
        EditingText.show();
    }
    
};

/*********************************************************************************/
// Sound Setting

enum SoundSettinState{
    SSS_Button=0,
    SSS_DoseStart=1,
    SSS_DoseEnd=2
};

//const char strButton[] PROGMEM="Button";
const char strDoseStart[] PROGMEM="Dose Start";
const char strDoseEnd[] PROGMEM="Dose End";

const char strOn[] PROGMEM="ON";
const char strOff[] PROGMEM="OFF";

const char* const SoundSettingLabels[]={
    strButton,
    strDoseStart,
    strDoseEnd
};

class SoundSetting:public SugarBaby{
public:
    SoundSetting(){}
    ~SoundSetting(){}


    void show(){
        _state = SSS_Button;
        _on = Settings.beepButton !=0;
        lcdPrint_P(0,0,strSoundSetting,true);
        _displayLabel();
        _displayOnOff();
    }
    
    void rotateForward(){
        if(!_on){
            _on = true;
            _displayOnOff();
        }
    }

    void rotateBackward(){
        if(_on){
            _on = false;
            _displayOnOff();
        }
    }

    bool switchPushed(){
        bool ret=false;
        if(_state == SSS_DoseEnd) ret=true;

        *(((uint8_t*)&Settings.beepButton) + _state) = _on? 1:0;
        
        _state = (SoundSettinState)( (uint8_t)_state + 1);

        _on = *(((uint8_t*)&Settings.beepButton) + _state) != 0;

        _displayLabel();
        _displayOnOff();

        if(ret){
            SettingManager.save();
        }
        return ret;
    }

protected:
    SoundSettinState _state;
    bool _on;

    void _displayLabel(){
        lcdPrint_P(0,1,SoundSettingLabels[_state],true);
    }
    void _displayOnOff(){
        if(_on){
            lcd->setCursor(13,1);
            lcd->write(' ');
            lcdPrint_P(14,1,strOn);
        }
        else  lcdPrint_P(13,1,strOff);
    }

};


/*********************************************************************************/
// Unit Setting

const char strUse[] PROGMEM="Use";
const char strWeight[] PROGMEM="Weight";
const char strVolume[] PROGMEM="Volume";

class UnitSetting:public SugarBaby{
public:
    UnitSetting(){}
    ~UnitSetting(){}


    void show(){
        _idx=0;
        _editing = false;
        lcdPrint_P(0,0,strUnitSetting,true);
        _showItem();
    }
    
    void rotateForward(){
        if(_editing){
            if(Settings.useWeight ==0){
                Settings.useWeight =1;
                _printValue();
            }
        }else{
            if(_idx >0){
                _idx --;
                _showItem();
            }
        }
    }

    void rotateBackward(){
        if(_editing){
            if(Settings.useWeight !=0){
                Settings.useWeight =0;
                _printValue();
            }
        }else{
            if(_idx <1){
                _idx ++;
                _showItem();
            }
        }
    }

    bool switchPushed(){
        if(_editing){
            _editing=false;
            EditingText.noblink();
        }else{
            if(_idx ==0){
                _editing=true;
                EditingText.blink();
            }else{
                SettingManager.save();
                return true;
            }
        }
        return false;
    }

protected:
    bool _editing;
    uint8_t _idx;
    void _showItem(){
        if(_idx ==0){
            lcdPrint_P(0,1,strUse,true);
            _printValue();
        }else{
             lcdPrint_P(0,1,strBack,true);
        }

    }
    void _printValue(){
        EditingText.setText_P(10,1, Settings.useWeight? strWeight:strVolume);
        EditingText.show();
    }
};

/*************************************************************************/
// Carbonation Settings
/*************************************************************************/
// setting of 
//  - Input Beer/Sugar  0
//  - Sugar Brix         1
//  - Co2 Volume         2
//  - Beer Temp          3
#define IndexInput 0
#define IndexSugar 1
#define IndexCo2Volume 2
#define IndexBeerTemp 3

#define LowestCarbonation 15
#define HighestCarbonation 45
#define MinBeerTemp 0
#define MaxBeerTemp 40


const char strCarbonation[] PROGMEM="Primimg";
const char strInput[] PROGMEM="Input";
const char strBeerVol[] PROGMEM="Beer Vol";
const char strXSugar[] PROGMEM="   Sugar";
const char strBrix[] PROGMEM="Brix";
const char strCo2Vol[] PROGMEM="CO2 Vol.";
const char strBeerTemp[] PROGMEM="Beer Temp";

/*
0123456789012345
 Input  Beer vol
           Sugar
 Brix     012.Bx
 Co2 Vol.    3.2
 BeerTemp   12.C
*/
class PrimingSetting:public SugarBaby{
public:
    PrimingSetting(){}
    ~PrimingSetting(){}
      void show(){
        _setIdx =0;
        lcdPrint_P(0,0,strUnitSetting,true);
        lcdPrint_P(0,1,strInput);
        _printInputValue();
    }

    void rotateForward(){
        if(_setIdx ==IndexInput){
            if(Settings.inputBeer) Settings.inputBeer=0;
            _printInputValue();
        }else if(_setIdx ==IndexSugar){
            if(Settings.sugarRatio<100) Settings.sugarRatio ++;
            _printSugarRatio();
        }else if(_setIdx ==IndexCo2Volume){
            if(Settings.carbonation < HighestCarbonation) Settings.carbonation += 1;
            _printCo2Volume();
        }else if(_setIdx ==IndexBeerTemp){
            if(Settings.beerTemperature < MaxBeerTemp) Settings.beerTemperature ++;
            _printBeerTemp();
        }

    }
    
    void rotateBackward(){
        if(_setIdx ==IndexInput){
            if(Settings.inputBeer ==0) Settings.inputBeer=1;
            _printInputValue();
        } if(_setIdx ==IndexSugar){
            if(Settings.sugarRatio>0) Settings.sugarRatio --;
            _printSugarRatio();
        }else if(_setIdx ==IndexCo2Volume){
            DBGPrint(F("Carbonation:"));
            DBGPrintln(Settings.carbonation);
            if(Settings.carbonation > LowestCarbonation) Settings.carbonation -= 1;
            _printCo2Volume();
        }else if(_setIdx ==IndexBeerTemp){
            if(Settings.beerTemperature > MinBeerTemp) Settings.beerTemperature --;
            _printBeerTemp();
        }
    }
    bool switchPushed(){
        if(_setIdx ==IndexInput){
            _setIdx = IndexSugar;
            lcdPrint_P(0,1,strBrix,true);
            lcd->setCursor(13,1);
            lcd->write(DegreeChar);
            lcd->write('B');
            lcd->write('x');
            if(Settings.sugarRatio>100) Settings.sugarRatio=100;
            _printSugarRatio();
        }else if(_setIdx ==IndexSugar){
            _setIdx = IndexCo2Volume;
            lcdPrint_P(0,1, strCo2Vol, true);
            if(Settings.carbonation < LowestCarbonation) Settings.carbonation = LowestCarbonation;
            else if(Settings.carbonation > HighestCarbonation) Settings.carbonation = HighestCarbonation;
            _printCo2Volume();
        }else if(_setIdx ==IndexCo2Volume){    
            _setIdx =IndexBeerTemp;
            lcdPrint_P(0,1,strBeerTemp,true);
            lcd->setCursor(14,1);
            lcd->write(DegreeChar);
            lcd->write('C');
            _printBeerTemp();
        }else if(_setIdx == IndexBeerTemp){
            SettingManager.save();
            return true;
        }
        return false;
    }
protected:
    uint8_t _setIdx;
    void _printInputValue(){
        if(Settings.inputBeer){
            lcdPrint_P(8,1,strBeerVol);
        }else{
            lcdPrint_P(8,1,strXSugar);
        }
    }

    void _printSugarRatio(){
        lcdPrintAt(10,1,Settings.sugarRatio,3);
    }
    void _printCo2Volume(){
        lcdPrintAt(13,1,Set2CV(Settings.carbonation),3,1);
    }

    void _printBeerTemp(){
        lcdPrintAt(12,1,Settings.beerTemperature,2);
    }

};
/*************************************************************************/
// Main
/*************************************************************************/
RotaryEncoder encoder(RA_CLK_PIN,RB_DT_PIN,SW_PIN);

class SugarDaddy{
public:
    SugarDaddy():_menuHandler(&MainMenu){

    }
    ~SugarDaddy(){}

    void begin(){
        bool initialized=SettingManager.begin();
        Buzzer.begin();
        lcdInitialize();

        SwitchButton::begin(BUTTON_PIN);
        
        dosingController.begin();

        if(initialized) _running = &_menuHandler;
        else _running = &_sugarCalibrate;
   
        _running->show();
    }

    void loop(){
        bool changed;
		RotaryEncoderStatus status=encoder.read();
  		switch(status)
  		{
    		case RotaryEncoderStatusPushed:
    			changed = _running->switchPushed();                
                if(changed){
                    switchTo(_running->nextApp());
                }
    			break;
    
    		case RotaryEncoderStatusFordward:
    			_running->rotateForward();
    			break;

    		case RotaryEncoderStatusBackward:
    			_running->rotateBackward();
    			break;
            
            default:
                break;
    	}
        _running->loop();
        // dosingController.loop(); // call ed in isDosingStateChange
        // drawing symobol
        // isDosingStateChanged() must be called in the loop
        if(dosingController.isDosingStateChanged()){
            _running->dosingStateChanged(dosingController.isDosing());
        }

        Buzzer.loop();
        EditingText.loop();
    }

    void switchTo(SugarAppId mode){
        switch(mode){
            case SugarAppMenu:
                _running = &_menuHandler;
                break;
            case SugarAppAutomatic:
                _running = & _autoDoser;
                break;
            case SugarAppManual:
                _running = & _manualDoser;
                break;
            case SugarAppDoseCalibration:
                _running = & _doseCalibrator;
                break;
            case SugarAppCalibration:
                _running = & _sugarCalibrate;
                break;
            case SugarAppDosingSettingMode:
                _running = & _dropSetting;
                break;
            case SugarAppSoundSetting:
                _running = & _soundSetting;
                break;
            case SugarAppUnitSetting:
                _running = & _unitSetting;
                break;
            case SugarAppPriming:
                _running=& _priming;
                break;
        }
        dosingController.setMode(DosingModeDisabled);
        lcd->clear();
        _running->show();
    }
protected:

    SugarBaby *_running;
    MenuHandler _menuHandler;
    DoseCalibration _doseCalibrator;
    SugarCalibrator _sugarCalibrate;
    AutoDoser _autoDoser;
    ManualDoser _manualDoser;
    TriggerSettings _dropSetting;
    SoundSetting _soundSetting;
    UnitSetting _unitSetting;
    PrimingSetting _priming;
};