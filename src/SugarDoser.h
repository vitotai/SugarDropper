
#pragma once
#include <Arduino.h>
#include <EEPROM.h>
#include <avr/wdt.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>


//#define DEBUG_OUT true

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
#define KrausenDoseAdjustUnit 0.5

#define MaximumAmount 99.0
#define MinimumAmount 0.5
#define MinimumDoseAmount 0.0
#define MaximumDoseAmount 999.0

#define MaximumCalibrationCount 100

#define RA_CLK_PIN 9
#define RB_DT_PIN 10
#define SW_PIN 11

#define BUTTON_PIN 2
#define BUTTON2_PIN 3

#define PUMP_PIN 5
#define PUMP2_PIN 6

#define SCL_PIN A5
#define SDA_PIN A4

#define BUZZ_PIN 12

#define DOSED_INDICATOR_PIN A1
#define DOSED_INDICATOR2_PIN A2
#define INVALID_PIN 0xFF

#define MinimumGapBetweenDrop 1000
#define CalibrationDropDelay 3000


#define BlinkingShowTime 600
#define BlinkingHideTime 150

#define MaximumDoserRatioInteger 300

#define RationIncStepBeer 0.001

typedef enum _SugarAppId{
    SugarAppMenu,
    SugarAppAutomatic,
    SugarAppManual,
    SugarAppDoseCalibration,
    SugarAppCalibration,
    SugarAppDosingSettingMode,
    SugarAppSoundSetting,
    SugarAppUnitSetting,
    SugarAppPriming,
    SugarAppBottleVolume,
    SugarAppSecondarySetting,
    SugarAppRunDoser
} SugarAppId;


/*********************************************************************************/
// LCD routines
//
#define DrosingSymbolChar 1
#define RevDrosingSymbolChar 2
#define DosingSymbolNone ' '
#define DegreeChar 0b11011111
#define SecondarySymbolChar 3
#define DosedSymbol 4
#define WaitSymbol 5

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

byte Rev2ndBitmap[8] = {
	0b11111, 
    0b11011, 
    0b10101, 
    0b11101, 
    0b11011, 
    0b10111, 
    0b10001, 
    0b11111
};
byte WaitBitmap[8]={
    0b00000,
    0b11111,
    0b01010,
    0b01010,
    0b00100,
    0b01010,
    0b01010,
    0b11111
};

byte DosedBitmap[8]={
    0b00000,
	0b01010,
	0b01010,
	0b10001,
	0b10001,
	0b11001,
	0b10111,
	0b11111
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
        lcd->createChar(SecondarySymbolChar,Rev2ndBitmap);
        lcd->createChar(DosedSymbol,DosedBitmap);
        lcd->createChar(WaitSymbol,WaitBitmap);

        lcd->noCursor();
}
void lcdPrintFixedSpace(byte col, byte row,byte lead,const char* str,const char leading=' '){
    lcd->setCursor(col,row);
    for(byte i=0;i<lead;i++) lcd->write(leading);
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
    lcdPrintFixedSpace(col,row,(len > space)? 0:(space - len),buff);
}

void lcdPrintAt(byte col,byte row, int val, byte space){
    char buff[16];
    byte len=sprintInt(buff,val);
    buff[len]='\0';
    lcdPrintFixedSpace(col,row,(len > space)? 0:(space - len),buff);
}

void lcdPrintAtZero(byte col,byte row, int val, byte space){
    char buff[16];
    byte len=sprintInt(buff,val);
    buff[len]='\0';
    lcdPrintFixedSpace(col,row,(len > space)? 0:(space - len),buff,'0');
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

void lcdDosingSymbol(char sym,uint8_t id){
    if(id ==0) lcd->setCursor(15,0);
    else lcd->setCursor(0,0);

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
        setNumberLeading(col,row,number,space,' ');
    }
    void setNumberLeading(uint8_t col, uint8_t row,int number,uint8_t space,const char lead){
         _col=col; _row = row;
        _len=sprintInt(_number,number);
        if(_len > space) _len=space;
        _number[_len]='\0';
        _space = space;
        _isText=false;
        _lead = lead;
    }

    void setNumber(uint8_t col, uint8_t row,float number,uint8_t space,uint8_t precision){
        _lead = ' ';
         _col=col; _row = row;
        _len=sprintFloat(_number,number,precision);
        if(_len > space) _len=space;
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
            for(int i=0;i<lead;i++) lcdWrite(_lead);
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
    uint8_t _lead;

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

/*********************************************************************************/
// krausening calculator
//  Vk = 1.95 * Vt * Ck / (GUk - GUb)
// Vk: volume of krausening beer needed, in quart
// Vt: Total Volume including krausening beer, in quart
// Ck:  carbonation level from krausening in volumes.
// GUk: Current Gravity of krausening Beer, in Points. ( 1052 instead of 1.052)
// GUb: Final Gravity of krausening Beer, in Points. ( 1012 instead of 1.012)

// input
//CD: target carbonation in volume
//  beetAmount = beer volume in liter
//  temperature = beer temperature in Celisus
// sg: gravity in 10xx form
// fg: gravity in 10xx form
float calculateKrauseningBeer(float CD, float total, float temperature,uint16_t sg, uint16_t fg){
    
    float Vt =  total;  // use liter. so don't need to convert back to quart
    float T = 32 + temperature * 9/5; ; // to F
    float Ck= CD - 3.0378 + 0.050062 * T - 0.00026555 * T * T;
     
    return 1.95 * Vt * Ck / (float)(sg - fg);
}

/********************************************************************************/
//     SG = (Brix / (258.6-((Brix / 258.2)*227.1))) + 1

float brix2SG(float Brix){
    return (Brix / (258.6-((Brix / 258.2)*227.1))) + 1;
}
/*********************************************************************************/
// Settings

#define FootprintPattern 0x5B5B5B5B

typedef struct _DosingControlleretting{
    float stepPerMl;
    float shotAdjustment;
    uint8_t triggerType;
    uint8_t delayTime;  
    uint8_t coolTime;
    // the following fields musht not changed orders.
    uint8_t beepButton;
    uint8_t beepDoseStart;
    uint8_t beepDoseEnd;
}DosingControllerSetting; // 14/16

#define SecondaryDoserDisabled 0
#define SecondaryDoserSync 1
#define SecondaryDoserIndependent 2


struct _Settings{
    uint32_t footPrint;

    uint8_t useWeight;
    // for priming sugar calculation
    uint8_t inputBeer;
    uint8_t sugarRatio; // in brix
    int8_t   beerTemperature;
    uint8_t  carbonation; // in volume * 10
    // Dosing machine
    uint8_t  secondaryDoserSet;
    float    secondaryDosageRatio;
    DosingControllerSetting doser[2];
    uint8_t     reverseRotataryDirection;
    uint8_t  krausening;
    uint16_t krausenGravity;
    uint16_t krausenFG;
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
            Settings.doser[0].shotAdjustment =0;
            Settings.doser[0].triggerType =0;
            Settings.doser[0].delayTime =0;
            Settings.doser[0].coolTime = 2;
            Settings.doser[1].shotAdjustment =0;
            Settings.doser[1].triggerType =0;
            Settings.doser[1].delayTime =0;
            Settings.doser[1].coolTime = 2;
            Settings.secondaryDoserSet =SecondaryDoserDisabled;
            Settings.reverseRotataryDirection = false;
            Settings.secondaryDosageRatio = 1;
            Settings.beerTemperature = 20;
            Settings.carbonation = 2.5;
            Settings.inputBeer = 0;
            Settings.sugarRatio = 50;
            Settings.useWeight = 1;
            Settings.krausening = 0;
            Settings.krausenGravity=1050;
            Settings.krausenFG = 1012;
            EEPROM.put(0,Settings);
            DBGPrintln("uninitialized data.");
            return false;
        }
        return true;
    }
    void save(){
        //EEPROM.put(0,Settings);
    }
    template <class T> int EEPROM_writeAnything(int ee, const T& value)
    {
        const byte* p = (const byte*)(const void*)&value;
        unsigned int i;
        for (i = 0; i < sizeof(value); i++)
            EEPROM.write(ee++, *p++);
        return i;
    }

    void write(uint8_t addr, uint8_t val){
        EEPROM.update(addr,val);
    }
    void setValueAt(uint8_t offset, uint8_t val){
        *(((uint8_t*)&Settings) + offset)=val;
        EEPROM.update(offset,val);
    }

    template <class T>  void writeDoserSetting(uint8_t id,int subaddr, const T& value){
        int addr = offsetof(_Settings,doser[0]) +  sizeof(DosingControllerSetting) * id + subaddr;
        const byte* p = (const byte*)(const void*)&value;
        unsigned int i;
        for (i = 0; i < sizeof(value); i++)
            EEPROM.write(addr++, *p++);

    }
};
SettingManagerClass SettingManager;


#define ReadSetting(a) Settings.a
#define WriteSetting(a,v) { Settings.a=v; SettingManager.EEPROM_writeAnything(offsetof(_Settings,a) ,v);}

#define DoserSetting(i,a) Settings.doser[i].a
#define WriteDoserSetting(i,a,v) SettingManager.writeDoserSetting(i,offsetof(DosingControllerSetting,a) ,v)

#define UpdateDoserSetting(i,a,v) Settings.doser[i].a=v,SettingManager.writeDoserSetting(i,offsetof(DosingControllerSetting,a) ,v)


#define ReadSettingAddress(a) *(((uint8_t*)&Settings) + (a))
//#define WriteSettingAddress(a,v) *(((uint8_t*)&Settings) + (a))=v
#define WriteSettingAddress(a,v) SettingManager.setValueAt(a,v)

#define AddressOfSetting(a) offsetof(_Settings,a)

/************************************************************************************************/
// Bottle list
/************************************************************************************************/

#define MaxVolume 5500
#define MinVolume 0


#define BottleVolumeBase 100
#define NumberOfBottles 10

class BottleListClass{
private:
    uint8_t _idx;
public:
    BottleListClass(){
        _idx=0;
    }
    ~BottleListClass(){}


    int getBottle(uint8_t idx){
        int16_t value;
        EEPROM.get(BottleVolumeBase + idx*2, value);
        if(value > MaxVolume || value<MinVolume) value=0;
        return value;
    }

    void setBottle(uint8_t idx, int volume){
        EEPROM.put(BottleVolumeBase + idx*2, (int16_t) volume);
    }

    int currentBottle(){
        return getBottle(_idx);
    }
    
    int next(){
        int vol;
        int idx = _idx;
        do{
            idx ++;
            if(idx == NumberOfBottles) idx=0;
            if(idx == _idx){
                return currentBottle();
            }
            vol=getBottle(idx);
        }while(vol ==0);
        _idx = idx;
        return vol;
    }

    int previous(){
        int vol;
        int idx = _idx;
        DBGPrint("previous:");
        DBGPrintln(idx);
        do{
            if(idx == 0) idx=NumberOfBottles-1;
            else idx --;
            DBGPrint("idx:");
            DBGPrint(idx);

            if(idx == _idx){
                // avoid inifinate loop if NONE set
                return currentBottle();
            }
            vol=getBottle(idx);
            DBGPrint(" vol:");
            DBGPrintln(vol);

        }while(vol ==0);
        _idx = idx;
        return vol;
    }
};
BottleListClass BottleList;
/*********************************************************************************/
//  Buzzer
const uint16_t BeepButton[] PROGMEM ={100,0};
const uint16_t BeepDoseStart[] PROGMEM ={300,0};
const uint16_t BeepDoseEnd[] PROGMEM ={550,0};
const uint16_t BeepError[] PROGMEM ={100,100,100,100,100,0};

class BuzzerClass{
public:
    BuzzerClass(uint8_t pin){
        _pin=pin;
    }
    ~BuzzerClass(){}
    
    void begin(){
        _buzzing = false;
        _playing = false;
        pinMode(_pin, OUTPUT);
        digitalWrite(_pin,LOW);
    }

    void loop(){
        if(_playing){
            if(millis() >= _stop){
                _play();
            }
        }
    }
    void buzz(const uint16_t* song){
        if(_playing) _stopBuzz();
        _playing = true;
        _ptrCurrentNote = song;
        _play();
    }
protected:
    uint8_t _pin;
    bool    _buzzing;
    uint32_t _stop;
    bool    _playing;
    const uint16_t *_ptrCurrentNote;

    void _play(){
        uint32_t duration=(uint32_t)pgm_read_word(_ptrCurrentNote);
        _ptrCurrentNote ++;
        if(duration ==0){
            _playing =false;
            _stopBuzz();
        }else{
            _stop = millis() + duration;            
            if(_buzzing){
                _stopBuzz();
            }else{
                _startBuzz();
            }
        }
    }
    void _startBuzz(){
        _buzzing = true;
        digitalWrite(_pin,HIGH);
    }
    void _stopBuzz(){
        digitalWrite(_pin,LOW);
         _buzzing = false;                 
    }
}Buzzer(BUZZ_PIN);

/*********************************************************************************/
//  Doser converts volume or gram to "steps"
/* 

  R: real Flow Rate
  L: Latency or Error cuased by start and stop of pump
  T: Dosing time. 
  S: total ouput
  
  R * T + L  = S, assuming L is relative small, when R*T is way larger than L, ignore L
  R*T = S,   R= S/T. 
  Then, in Dose calibration

  N: number of dosages.

  N * ( R * T + L ) = S
  L = (S -  R*T*N )/N

  Alternative way, run Twice calibration with different time

 R * T1 + L  = S1
 R * T2 + L  = S2
R *(T1 - T2) = S1 - S2, R = (S1 - S2)/(T1 - T2)
L = S1 - R * T1

*/
//******************************************************************

class Doser{
public:
    Doser(){}
    ~Doser(){}

    void begin(byte pin,float stepPerMl, float shotAdjustment){
        _stepper.begin(pin);
        _stepPerMl = stepPerMl;
        _shotAdjustment = shotAdjustment;

        DBGPrint(F("step per ml:"));
        DBGPrint(_stepPerMl);
        DBGPrint(F(" shotAdjustment:"));
        DBGPrintln(_shotAdjustment);
    }
    
    bool running(){
        return _stepper.running();
    }

    void dose(float amount){
         if(_stepper.running()) return;

        float adjustedValue =amount + _shotAdjustment;
        uint32_t steps =(uint32_t)(adjustedValue * _stepPerMl);

        _startDosingPos=_stepper.runSteps( steps );

        DBGPrint("\toutput");
        DBGPrint(amount,2);
        DBGPrint(F(" corrected:"));
        DBGPrint(adjustedValue);
        DBGPrint(F("ml as steps:"));
        DBGPrintln(steps);
    }
    
    uint32_t dosingDuration(float amount){
        float adjustedValue =amount + _shotAdjustment;
        return (uint32_t)(adjustedValue * _stepPerMl);
    }
    
    void run(){
        _startDosingPos=_stepper.run();
        DBGPrintln(F("\t flow start"));
    }

    void stop(){
        _stepper.stop();
        DBGPrintln(F("\t flow stop"));
    }

    uint32_t steps(){
        return _stepper.steps();
    }

    void setStepPerMl(float amount){
        _stepPerMl = amount;
    }

    void setDoseAdjustment(float value){
        _shotAdjustment =value;
        DBGPrint(F("update shot adjustment:"));
        DBGPrintln(value);
    }
    // for statistic information
    float getDosingVolume(){
        // calculate running since latest start
        uint32_t steps = _stepper.steps();
//        DBGPrint(F("steps:"));
//        DBGPrintln(steps - _startDosingPos);
        return (float)(steps - _startDosingPos) / _stepPerMl;
    }
    
protected:
    Stepper _stepper;
    float _stepPerMl;
    float _shotAdjustment;

    uint32_t _startDosingPos;
};

enum DosingMode{
DosingModeDisabled,
DosingModeSingleShot,
DosingModeManual,
DosingModeForced
};

enum DosingState{
DSIdle,
DSPrepareToDose,
DSDosing,
DSDosed, // for sensor
DSCoolTime
};
//******************************************************************
// controls Dosing operation.

class DosingController{
public:
    DosingController(uint8_t id):_id(id),_buttonPressed(false),_dosing(false){}
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
    
    void begin(SwitchButton& switchButton,byte pin, byte dosedIndicatorPin=INVALID_PIN){
        loadSetting();
        _mode = DosingModeDisabled;
        _switchButton = & switchButton;
        _doser.begin(pin, _setting->stepPerMl,_setting->shotAdjustment);
        _dosedIndPin = dosedIndicatorPin;
        if(_dosedIndPin != INVALID_PIN){
            pinMode(_dosedIndPin,OUTPUT);
            digitalWrite(_dosedIndPin,LOW);
        }
    }
    
    void setButton(SwitchButton& switchButton){
        _switchButton = & switchButton;
    }

    void setParameter(DosingControllerSetting *doseSetting){
        _dosingDelay = doseSetting->delayTime * 500;
        _coolTime = doseSetting->coolTime * 500;
        _isPositionSensor = doseSetting->triggerType !=0;
        _beepButton = doseSetting->beepButton;
        _beepDoseStart = doseSetting->beepDoseStart;
        _beepDoseEnd = doseSetting->beepDoseEnd;
        DBGPrint(F("set "));
        DBGPrint(_id);
        DBGPrint(F(" _dosingDelay "));
        DBGPrintln(_dosingDelay);
    }

    void loadSetting(){
        _setting = & Settings.doser[_id];
        setParameter(_setting);
    }

    // most be call every loop
    bool isDosingStateChanged(){
        bool ret=false;
        if(_mode == DosingModeSingleShot || _mode == DosingModeManual){
            if(_isPositionSensor){
                if(_switchButton->statusChangedFrom(_buttonPressed)){
                    _buttonPressed=_switchButton->pressed();
                    _handleSensorState(_buttonPressed);
                }
            }else{
                if(_switchButton->statusChangedFrom(_buttonPressed)){
                    _buttonPressed=_switchButton->pressed();
                    _handleButtonAction(_buttonPressed);
                } 
            }
        }

        if(_dosing != _doser.running()){
            _dosing = _doser.running();
            _processStateForDropingStateChange(_dosing);
            if(_dosing){
                if(_beepDoseStart) Buzzer.buzz(BeepDoseStart);
                _abnormalStop = false;
            }else{
                if(_beepDoseEnd){
                    if(_abnormalStop) Buzzer.buzz(BeepError);
                    else Buzzer.buzz(BeepDoseEnd);
                }
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
        if(! _doser.running()){
            if(_dosage >0){
                lcdDosingSymbol(DrosingSymbolChar,_id);
                _doser.dose(_dosage);
                _state = DSDosing;
            }
        }
    }

    void resetDoseAdjustment(){
        setDoseAdjustment(0);
    }
    
    void setDoseAdjustment(float val){
        _doser.setDoseAdjustment(val);
        _setting->shotAdjustment = val;
        // save to EEPROM
        WriteDoserSetting(_id,shotAdjustment,val);
    }

    void startCalibrate(){
       _calStarSteps= _doser.steps();
    }

    void calibrate(float vol){
        uint32_t currentSteps = _doser.steps();
        float spm = (float)(currentSteps - _calStarSteps) / vol;
        _doser.setStepPerMl(spm);
        // save
        _setting->stepPerMl = spm;
        // update to EEPROM
        WriteDoserSetting(_id,stepPerMl,spm);
    }

    inline float getDosingVolume(){
        return _doser.getDosingVolume();
    }

    inline uint32_t estDosingTime(){
        return _doser.dosingDuration(_dosage);
    }

    void run(){
        _doser.run();
        _dosing=true;
    }
    void stop(){
        _doser.stop();
        _dosing = false;
    }
protected:
    Doser _doser;
    uint8_t    _id;
    DosingControllerSetting *_setting;
    SwitchButton* _switchButton;
    bool  _buttonPressed;
    bool _dosing;
    float _dosage;
    DosingMode _mode;
    DosingState _state;
    bool       _isPositionSensor;
    bool       _inPosition;
    uint32_t   _dosingDelay;
    uint32_t   _coolTime;

    uint32_t   _waitToAction;
    uint32_t   _calStarSteps;
    bool _beepDoseStart;
    bool _beepDoseEnd;
    bool _beepButton;
    byte _dosedIndPin;
    bool _abnormalStop;

    void _handleSensorState(bool inPosition){
        _inPosition = inPosition;
        if(inPosition){
            if(_mode == DosingModeSingleShot || _mode == DosingModeManual){
                if(_state==DSIdle){
                   if(_beepButton) Buzzer.buzz(BeepButton);
                    lcdDosingSymbol(RevDrosingSymbolChar,_id);
                    _waitToAction = millis();
                    _state = DSPrepareToDose;
                }
            }
        }else{
            //out of position. stop any way
            
            if(_doser.running()){
                _doser.stop();
                DBGPrintln(F("Forced Stop"));
                // doser state changed will be called
                // and state change will be done there.
            }else if (_state==DSPrepareToDose){
                _state=DSIdle;
                lcdDosingSymbol(DosingSymbolNone,_id);
                DBGPrintln(F("Forced Stop"));
            }else if(_state == DSDosed){
                // dosed bottle removed, enter cooling time
                _enterWaitCoolingState();
            }
        }
    }

    void _handleButtonAction(bool pressed){
            // care only about pressed.
        if(!pressed) return;

        if(_state==DSIdle){
            if((_mode == DosingModeSingleShot && _dosage >0 )|| _mode == DosingModeManual){
                    if(_beepButton) Buzzer.buzz(BeepButton);
                    lcdDosingSymbol(RevDrosingSymbolChar,_id);

                    _waitToAction = millis(); 
                    _state = DSPrepareToDose;
                    DBGPrint(F("PrepareToDose:"));
                    DBGPrintln(_waitToAction);
            }
        }else if(_state==DSDosing){
            if(_mode == DosingModeManual){
               if(_beepButton) Buzzer.buzz(BeepButton);
                // stop
                _doser.stop();
                // state change will be process when
                // dosing end dectected
            }
        }
    }
    void _enterWaitCoolingState(){
        lcdDosingSymbol(WaitSymbol,_id);
        _state = DSCoolTime;
        _waitToAction = millis(); // + _coolTime;
    }
    void _enterDosedState(){
        lcdDosingSymbol(DosedSymbol,_id);
        _state = DSDosed;
        if(_dosedIndPin != INVALID_PIN){
            digitalWrite(_dosedIndPin,HIGH);
        }
    }

    void _processStateForDropingStateChange(bool dosing){
        if(! dosing){ // dosing end
            if(_mode == DosingModeDisabled){
                lcdDosingSymbol(DosingSymbolNone,_id);
                _state = DSIdle;
            }else{
                if(_isPositionSensor){
                    if(!_inPosition){                        
                        if( _mode == DosingModeSingleShot){
                            _abnormalStop =true;
                        }
                        _enterWaitCoolingState();
                    }else{
                        _enterDosedState();
                    }
                } else{
                    _enterWaitCoolingState();
                }
            }
        }
    }

    void _processStateForTime(){
        uint32_t present = millis();
        if(_state == DSPrepareToDose){
            if( (present - _waitToAction) >=  _dosingDelay){
                    if(_mode == DosingModeSingleShot && _dosage >0){
                        lcdDosingSymbol(DrosingSymbolChar,_id);
                        _doser.dose(_dosage);
                         _state = DSDosing;
                    }else{
                        lcdDosingSymbol(DrosingSymbolChar,_id);
                        _doser.run();
                         _state = DSDosing;
                    }                   
            }
        }else if(_state == DSCoolTime){
            if( (present - _waitToAction) >= _coolTime){
                _state = DSIdle;
                lcdDosingSymbol(DosingSymbolNone,_id);
                if(_dosedIndPin != INVALID_PIN){
                    digitalWrite(_dosedIndPin,LOW);
                }
            }
        }
    }

} dosingController(0), dosingController2(1);




/*********************************************************************************/
// Apps

class SugarApe{
public:
    SugarApe(){}
    virtual ~SugarApe(){}

    virtual void show(){}
    virtual void loop(){}
    // UI related
    virtual void rotateForward(){}
    virtual void rotateBackward(){}
    virtual bool switchPushed(){return true;}
    virtual bool switchLongPressed(){return true;}
    // on exit
    virtual SugarAppId nextApp(){return SugarAppMenu;}

    virtual void dosingStateChanged(uint8_t,bool){}
};

/*********************************************************************************/
// Manu handler
const char strAutomatic[] PROGMEM  ="Automatic";
const char strManual[] PROGMEM ="Manual";
const char strCalibration[] PROGMEM ="Calibration";
const char strShotCalibration[] PROGMEM ="Dose calibrate";
const char strDropSettings[] PROGMEM ="Dosing Control";
const char strBack[]  PROGMEM ="Back";
const char strSoundSetting[] PROGMEM="Sound Setting";
const char strSetting[]  PROGMEM="Setting";
const char strMl[] PROGMEM = "ml";
const char strCl[] PROGMEM = "cl";
const char strUnitSetting[] PROGMEM ="Unit";
const char strFunction[] PROGMEM ="Function";
const char strAutoDoseSettings[] PROGMEM="Priming";
const char str2ndDoser[] PROGMEM="2nd Doser";
const char strBottles[] PROGMEM="Bottles";

const char strRunDoser[] PROGMEM="Run Doser";

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
    {str2ndDoser, false, {.mode = SugarAppSecondarySetting}},
    {strUnitSetting, false, {.mode=SugarAppUnitSetting}},
    {strDropSettings, false, { .mode=SugarAppDosingSettingMode }},
    {strSoundSetting, false, {.mode=SugarAppSoundSetting}},
    {strCalibration, false,{ .mode=SugarAppCalibration }},
    {strShotCalibration,false,{ .mode=SugarAppDoseCalibration}},
    {strBottles,false,{ .mode=SugarAppBottleVolume}},
    {strBack, true, { .subMenu= &MainMenu }}
};

const MenuList SettingMenu={
    strSetting,
    sizeof(SettingMenuItems)/sizeof(MenuList),
    SettingMenuItems
};

const MenuItem mainMenuItems[]  ={
    {strAutomatic,false, {.mode = SugarAppAutomatic}},
    {strAutoDoseSettings, false, { .mode=SugarAppPriming }},
    {strRunDoser, false, {.mode =SugarAppRunDoser}},
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

class MenuHandler:public SugarApe{
private:
    byte _selected;
public:
    MenuHandler(const MenuList *menu):_selected(0){
        _menu = menu;
    }
    ~MenuHandler(){}

    void show(){
        // print display
        lcdPrint_P(1,0,_menu->title,true);
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
// Synchronized two doser
/*
  One other solution is to use one "controller" that reads buttons and control
  both doser. However, that would duplicate or complicate things when seperated control
  is needed.
*/
void loadDosingControlParameter(){
    if(ReadSetting(secondaryDoserSet) == SecondaryDoserIndependent){
        dosingController.setParameter(& Settings.doser[0]);
        dosingController2.setParameter(& Settings.doser[1]);
    }else if(ReadSetting(secondaryDoserSet) == SecondaryDoserSync){
        // using doser1 setting, but need to adjust end "beep"
        DosingControllerSetting set1,set2;
        set1 = Settings.doser[0];
        set2 = Settings.doser[0];
        // let controller1 "beep" when ncecessary
        set2.beepButton = false;
        set2.beepDoseStart = false;
        set2.beepDoseEnd = false;
        dosingController.setParameter(& set1);
        dosingController2.setParameter(& set2);
    }else{
        dosingController.setParameter(& Settings.doser[0]);
    }
}

/*********************************************************************************/
// Automatic dosing
/*********************************************************************************/

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
#999     1330mlX 
A:999.25 99.25g

*/

const char strTotal[]  PROGMEM="A:";
const char str2nd[]  PROGMEM="\3";

const char strResetExit_0[] PROGMEM=" RESET >EXIT";
const char strResetExit_1[] PROGMEM=">RESET  EXIT";

#define DosageAmountRow 1
#define DosageAmountCol 9
#define DosageAmountSpace 5

#define BeerVolumeRow 0
#define BeerVolumeCol 10
#define BeerVolumeSpace 3

#define DosageCountRow 0
#define DosageCountCol 1
#define DosageCountSymbolCol 0
#define DosageCountSapce 3


#define AccBeerVolCol 5
#define AccBeerVolRow 0
#define AccBeerVolSpace 4
#define AccBeerVolSymbolCol 9

#define AccumulatedOutputRow 1
#define AccumulatedOutputCol 2
#define AccumulatedOutputStringCol 0
#define AccumulatedOutputSpace 6

#define MinimumBeerVolume 50
#define DefaultBeerVolume  330
#define MaximumBeerVolume 5000

#define ACC_BEER_AMOUNT true


/*
0123456789012345
#999 00.0L100clX 
A:999.25 99.25g

0123456789012345
X099/999  133clX
99x99.25 99.25g

*/    


class AutoDoser:public SugarApe{
public:
    AutoDoser(){
        _reset();
    }

    ~AutoDoser(){}
    

    void show(){
        _exitOrReset = false;
        _changeSecondary=false;
        _inputBeer = ReadSetting(inputBeer);
        _useSecondary = ReadSetting(secondaryDoserSet) != SecondaryDoserDisabled;

        if(_inputBeer){
            _beerVolume = BottleList.currentBottle();
        }else{
            if(_amount < MinimumDoseAmount || _amount > MaximumDoseAmount) _amount = 5;
        }
        
        if(_useSecondary){
            lcdWriteAt(4,0,'/');
            EditingText.setText_P(2,1,str2nd);
            EditingText.show();
            _doser2Ratio = ReadSetting(secondaryDosageRatio);
            if(_doser2Ratio ==0){
                // manual input
                _dosage2 =0;
            }else{
                _dosage2 = _doser2Ratio * (_inputBeer? _beerVolume:_amount);
            }
            #if ! ACC_BEER_AMOUNT
            _updateCount2();
            #endif
            _updateDosage2();
        }else{
            // "A:"
            lcdPrint_P(AccumulatedOutputStringCol,AccumulatedOutputRow,strTotal);
            // "#"
            lcdWriteAt(DosageCountSymbolCol,DosageCountRow,'#');
            // 'L'
            lcdWriteAt(AccBeerVolSymbolCol,AccBeerVolRow,'L');
        }

        // dosage unit
        if(ReadSetting(useWeight)) lcdWriteAt(DosageAmountCol + DosageAmountSpace,DosageAmountRow,'g');
        else lcdPrint_P(DosageAmountCol + DosageAmountSpace,DosageAmountRow,strCl);


        if(_inputBeer){
            lcdPrint_P(BeerVolumeCol + BeerVolumeSpace,BeerVolumeRow,strCl);
            _updateBeerVolume();
            _calPrimingSugar();
        }

        _updateDosage();
        _updateCount();
        _updateTotal();
        
        loadDosingControlParameter();
        dosingController.setMode(DosingModeSingleShot);
        if(_useSecondary){
            dosingController2.setButton( (ReadSetting(secondaryDoserSet) == SecondaryDoserSync)? switchButton1:switchButton2);
            dosingController2.setMode(DosingModeSingleShot);
        }
    }

    void rotateForward(){
        if(_changeSecondary){
            _dosage2 += DoseAdjustUnit;
            if(_dosage2 > MaximumAmount) _dosage2 = MaximumAmount;
            _updateDosage2();
        }else if (_exitOrReset){
            _exitResetSelection=(_exitResetSelection ==0)? 1:0;
            _showExitReset();
        }else if(_inputBeer){
            int vol = BottleList.next();
            if( _beerVolume != vol){
                _beerVolume =vol;

                _updateBeerVolume();
                _calPrimingSugar();
                _updateDosage();

                if(_useSecondary){
                    if(_doser2Ratio > 0){
                        _dosage2 = _doser2Ratio * _beerVolume;
                        _updateDosage2();
                    }
                }
            }
        }else{
        
           _amount += ReadSetting(krausening)? KrausenDoseAdjustUnit:DoseAdjustUnit;
            if(_amount > MaximumDoseAmount) _amount = MaximumDoseAmount;
            _updateDosage();
            if(_useSecondary){
                if(_doser2Ratio > 0){
                    _dosage2 = _doser2Ratio * _beerVolume;
                    _updateDosage2();
                }
            }

        }
    }
    
    void rotateBackward(){
        if(_changeSecondary){            
            if( (_dosage2 -DoseAdjustUnit)  >= MinimumDoseAmount){
                _dosage2 -= DoseAdjustUnit;
                _updateDosage2();
            }
        }else if (_exitOrReset){
            _exitResetSelection=(_exitResetSelection ==0)? 1:0;
            _showExitReset();
        }else if(_inputBeer){
            int vol = BottleList.previous();
            if( _beerVolume != vol){
                _beerVolume =vol;

                _updateBeerVolume();
                _calPrimingSugar();
                _updateDosage();

                if(_useSecondary){
                    if(_doser2Ratio > 0){
                        _dosage2 = _doser2Ratio * _beerVolume;
                        _updateDosage2();
                    }
                }

            }
        }else{
            _amount -=  ReadSetting(krausening)? KrausenDoseAdjustUnit:DoseAdjustUnit;
            if(_amount <MinimumDoseAmount) _amount = MinimumDoseAmount;
            _updateDosage();

            if(_useSecondary){
                if(_doser2Ratio > 0){
                    _dosage2 = _doser2Ratio * _beerVolume;
                    _updateDosage2();
                }
            }

        }
    }

    void dosingStateChanged(uint8_t doserId,bool dosing){
        if(dosing){
            if(doserId==0){
                _count ++;
                _updateCount();
            }else{
                _count2 ++;
                #if ! ACC_BEER_AMOUNT
                _updateCount2();
                #endif
            }
        }else{
            // udpateTotal, suppose it _amount ++, but the amount might changed. so...
             if(doserId==0){
                _totalAmount += dosingController.getDosingVolume();
                _totalBeer += _beerVolume;
                _updateTotal();
             }
        }
    }
    
    bool switchPushed(){
        if(_changeSecondary){
            _changeSecondary=false;
            EditingText.noblink();
            return false;
         }else if (_exitOrReset){
            if(_exitResetSelection ==0){
                return true;
            }else{
                _reset();
                show();
            }
         }else{
            _exitResetSelection = 0;
            _exitOrReset=true;
            _showExitReset();
         }
        return false;
    }

    bool switchLongPressed(){
        if(_useSecondary && _doser2Ratio==0){
            if(_changeSecondary){
                _changeSecondary=false;
                EditingText.noblink();
            }else{
                _changeSecondary=true;
                EditingText.blink();
            }
        }
        return true;
    }

protected:
    float _amount;
    float _totalAmount;
    uint16_t _count;
    uint16_t _count2;
    int16_t _beerVolume;
    float _totalBeer;
    bool _inputBeer;
    bool _useSecondary;
    float _dosage2;
    float _doser2Ratio;
    bool  _changeSecondary;
    bool _exitOrReset;
    uint16_t _exitResetSelection;

    void _reset(){
        _totalAmount = 0;
        _count =0;
        _count2 =0;
        _beerVolume =0;
        _totalBeer =0.0;
    }
    
    void _showExitReset(){
        lcdClearLine(1);
        if(_exitResetSelection ==0){
            lcdPrint_P(1,1,strResetExit_0);
        }else{
            lcdPrint_P(1,1,strResetExit_1);
        }
    }

    void _calPrimingSugar(){
        //
        DBGPrint(F("CD:"));
        DBGPrint(Set2CV(ReadSetting(carbonation)));
        DBGPrint(F(" beerTemperature:"));
        DBGPrintln(ReadSetting(beerTemperature));
        if(ReadSetting(krausening)){
            _amount=calculateKrauseningBeer(Set2CV(ReadSetting(carbonation)),(float)_beerVolume/1000.0,(float)ReadSetting(beerTemperature),
                    ReadSetting(krausenGravity),ReadSetting(krausenFG) );
            _amount = _amount * 1000.0; 
            DBGPrint(F("Krausening:"));
            DBGPrintln(_amount);
        
        }else{
            float ps=calculatePrimingSugar(Set2CV(ReadSetting(carbonation)),(float)_beerVolume/1000.0,(float)ReadSetting(beerTemperature));
            float weight = ps / (float)ReadSetting(sugarRatio) * 100.0;
            // if the unit is volume, calculate SG and derive volume
            // however, temperature might be a problem. ignore that for now
            DBGPrint(F("PS:"));
            DBGPrint(ps);
            DBGPrint(F(" weight:"));
            DBGPrintln(weight);

//            if(ReadSetting(useWeight)){
                _amount = weight;
#if 0                
            }else{
                DBGPrint(F("SG:"));
                DBGPrintln(brix2SG((float)ReadSetting(sugarRatio)));

                _amount = weight / brix2SG((float)ReadSetting(sugarRatio));
            }
#endif            
        }
    }

    // LCD display
/*
0123456789012345
#999 99.9L 33clX 
A:999.25 99.25g

0123456789012345
X099/999  133clX
99x99.25 99.25g

count/ACC beervol
c2xACC2    Amount

*/    
    void _updateCount(){
        lcdPrintAtZero(DosageCountCol,DosageCountRow,_count,DosageCountSapce);
    }
    #if! ACC_BEER_AMOUNT
    void _updateCount2(){
        lcdPrintAtZero(0,1,_count2,2);
    }
    #endif

    void _updateTotal(){
        if(_useSecondary){
            int rounded =(int) round(_totalAmount);
            lcdPrintAtZero(5,0,rounded,3);
            #if ACC_BEER_AMOUNT
            float inLiter = _totalBeer/1000.0;
            if(inLiter > 100.0) inLiter=inLiter - floor(inLiter/100)*100; // don't have space for over 100liter
            if(inLiter < 1.0){
                lcdWriteAt(0,1,'.');
                lcdPrintAtZero(1,1,(uint16_t) round(inLiter*10),1);
            }else{                
                lcdPrintAtZero(0,1,(uint16_t) round(inLiter),2);
            }
            #endif
        }else{
            if(_totalAmount>=1000.0){
                lcdPrintAt(AccumulatedOutputCol,AccumulatedOutputRow,_totalAmount,AccumulatedOutputSpace,1);
            }else{
                lcdPrintAt(AccumulatedOutputCol,AccumulatedOutputRow,_totalAmount,AccumulatedOutputSpace,2);
            }


            float inLiter = _totalBeer/1000.0;
            if(inLiter >= 100.0){
                lcdPrintAt(AccBeerVolCol,AccBeerVolRow,(uint16_t) round(inLiter),AccBeerVolSpace);
            }else{
                lcdPrintAt(AccBeerVolCol,AccBeerVolRow,inLiter,AccBeerVolSpace,1);
            }
        }
    }
    void _updateDosage(){
        DBGPrint(F("Sugar amount:"));
        DBGPrintln(_amount);
        if(_amount >= 100.0){
            lcdPrintAt(DosageAmountCol,DosageAmountRow,_amount,DosageAmountSpace,1);
        }else{
            lcdPrintAt(DosageAmountCol,DosageAmountRow,_amount,DosageAmountSpace,2);
        }
        dosingController.setDosage(_amount);
    }
    
    void _updateDosage2(){
        lcdPrintAt(3,1,_dosage2,5,2);
        dosingController2.setDosage(_dosage2);
    }    
    
    void _updateBeerVolume(){
        DBGPrint(F("Beer Vol:"));
        DBGPrintln(_beerVolume);
        lcdPrintAt(BeerVolumeCol,BeerVolumeRow,_beerVolume/10,BeerVolumeSpace);
    }
};


/*********************************************************************************/
// manual dosing
/*********************************************************************************/

/*
0123456789012345
Manual
100.1s  999.12ml
*/

#define MinimumUpdateTime 100

class ManualDoser:public SugarApe{
public:
    ManualDoser(){}
    ~ManualDoser(){}

    void show(){
        loadDosingControlParameter();
        if(ReadSetting(secondaryDoserSet) != SecondaryDoserDisabled){
            _primaryRow =0;
            dosingController2.setButton((ReadSetting(secondaryDoserSet) == SecondaryDoserSync)? switchButton1:switchButton2);

            dosingController2.setMode(DosingModeManual);
            _doserRunning[1] = false;
            _accumulatedVolume[1] = 0;
            _accumulatedTime[1]=0;
        
            _printUnit(1);
            _updateInfo(1,0,0);

        }else{
            lcdPrint_P(1,0,strManual,true);

            _primaryRow = 1;
        }
        
        _printUnit(_primaryRow);
        _updateInfo(_primaryRow,0,0);

        dosingController.setMode(DosingModeManual);
        _doserRunning[0] = false;
        _accumulatedVolume[0] = 0;
        _accumulatedTime[0]=0;
    }

    void loop(){
        _updateRunningInfo(0);
        if(ReadSetting(secondaryDoserSet) != SecondaryDoserDisabled){
            _updateRunningInfo(1);
        }
    }

    void dosingStateChanged(uint8_t doserId,bool dosing){
        if(dosing){
            _startedTime[doserId] = millis();
            _doserRunning[doserId] =true;
        }else{
            _accumulatedVolume[doserId] += doserId==0? dosingController.getDosingVolume():dosingController2.getDosingVolume();
            _doserRunning[doserId] = false;
            _accumulatedTime[doserId] += millis() - _startedTime[doserId];
            _updateInfo(doserId==0? _primaryRow:1,_accumulatedTime[doserId],_accumulatedVolume[doserId]);
        }
     }

protected:
    uint32_t _lastUpdate[2];
    uint32_t _accumulatedTime[2];
    uint32_t _startedTime[2];
    float    _accumulatedVolume[2];
    bool     _doserRunning[2];

    uint8_t  _primaryRow;

    void _updateInfo(uint8_t row,uint32_t duration,float volume){
        lcdPrintAt(1,row,(float) duration/1000.0,5,1);        
        DBGPrint("accvol:");
        DBGPrintln(volume);
        lcdPrintAt(7,row,volume ,7,2);
    }
    void _printUnit(uint8_t row){
        if(ReadSetting(useWeight)) lcdWriteAt(14,row,'g');
        else lcdPrint_P(14,row,strMl);
        lcd->setCursor(6,row);
        lcd->write('s');        
    }
    void _updateRunningInfo(uint8_t idx){
        if(_doserRunning[idx]){
            if( millis() - _lastUpdate[idx] > MinimumUpdateTime){
                DBGPrint(F("_accumulatedTime[idx] running, idx:"));
                DBGPrint(idx);
                DBGPrint(F(" acc:"));
                DBGPrintln(_accumulatedVolume[idx]);

                _lastUpdate[idx] = millis();

                _updateInfo(idx==0? _primaryRow:1,
                    _accumulatedTime[idx] +  millis() - _startedTime[idx],
                    _accumulatedVolume[idx] + 
                    (idx==0? dosingController.getDosingVolume():dosingController2.getDosingVolume()));
            }
        }

    }
};



/*********************************************************************************/
// Run dosing
/*********************************************************************************/

/*

0123456789012345
 Run Primary
     Secondary
 Run/Stop/Exit
*/

const char strExit[] PROGMEM="Exit";
const char strRun[] PROGMEM="Run";
const char strStop[] PROGMEM=">Stop";
const char strPrimary[] PROGMEM =   "Primary";
const char strSecondary[] PROGMEM = "Secondary";
const char strDoser[] PROGMEM = "Run";

const char strRunOrExitRun[] PROGMEM = ">Run  Exit";
const char strRunOrExitExit[] PROGMEM= " Run >Exit";

class RunDoser:public SugarApe{
public:
    RunDoser(){}
    ~RunDoser(){}

    void show(){


        _doserId = 0;

        if(ReadSetting(secondaryDoserSet) != SecondaryDoserDisabled){
            lcdPrint_P(1,0,strDoser);
            _displayDoserSelection();
            _doserChosen = false;
        }else{
             _controller = &dosingController;
            lcdPrint_P(1,0,strRunDoser);
            _doserChosen = true;
            _exit=false;
            _printAction();
            _controller->setMode(DosingModeForced);
        }

    }

    void loop(){
    }

    void rotateForward(){
        if(! _doserChosen){
            if(_doserId ==1 ){
                _doserId = 0;
                _displayDoserSelection();
            }
        }else{
            if(_controller->isDosing()) return;
            // only handle when not running.
            if(_exit){ 
                _exit= false;
                _printAction();
            }
        }
    }
    
    void rotateBackward(){
        if(! _doserChosen){
            if(_doserId ==0 ){
                _doserId = 1;
                _displayDoserSelection();
            }
        }else{
            if(_controller->isDosing()) return;
            // only handle when not running.
            if(!_exit){ 
                _exit= true;
                _printAction();
            }
        }
    }
    
    bool switchPushed(){
        if(! _doserChosen){
            _controller = _doserId? &dosingController2:&dosingController;
            EditingText.noblink();
            _doserChosen = true;
            _exit = false;
            _printAction();
        }else{
            if(_exit){
                EditingText.noblink();
                return true;
            }
            if(_controller->isDosing()){
                DBGPrint(F("Stop Doser\n"));
                _controller->stop();
            }else{
                DBGPrint(F("Run Dorser\n"));
                _controller->run();
            }
            _printAction();
        }
        return false;
    }

protected:
    bool _doserChosen;
    bool _exit;
    uint8_t _doserId;
    DosingController *_controller;

    void _displayDoserSelection(){
        lcdClear(5,0,9);
        EditingText.setText_P(5,0,_doserId? strSecondary:strPrimary);
        EditingText.blink();
    }

    void _printAction(){
        lcdClear(1,1,4);
        if(_exit){
            // EditingText.setText_P(1,1,strExit);
            lcdPrint_P(1,1,strRunOrExitExit);
        }else{
            DBGPrint(F("Is Dosing:"));
            DBGPrintln(_controller->isDosing());

            //EditingText.setText_P(1,1,_controller->isDosing()? strStop:strRun);
            if(_controller->isDosing()){
                lcdPrint_P(1,1,strStop,true);       
            }else{
                lcdPrint_P(1,1,strRunOrExitRun);       
            }
        }
        //EditingText.blink();
    }
};



/*********************************************************************************/
// initial calibration/setup
//  to derive steps/ml
typedef enum _CalibrationMode{
Cal_ViewParameter_1,
Cal_ViewParameter_2,
Cal_Back,

Cal_SelectVolume,
Cal_RunDoser,
Cal_InputVolume
}CalibrationMode;

/*
0123456789012345
Setup
cal by 100.24 ml
*/

const char strCalBy[] PROGMEM="Vol";
const char strAdjust[] PROGMEM="adjust";
//const char strRunDoser[] PROGMEM="Run Doser";
const char strEnter[] PROGMEM="Next";
const char strRate[] PROGMEM="Rate";
const char strD1[] PROGMEM="D#1";
const char strD2[] PROGMEM="D#2";

const char strMlPerSec[] PROGMEM="ml/s";
const char strGramPerSec[] PROGMEM="g/s";

#define TitleRow 0
#define TitleCol 1

class SugarCalibrator:public SugarApe{
public:
    SugarCalibrator(){}
    ~SugarCalibrator(){}

    void show(){
        lcdPrint_P(TitleCol,TitleRow,strCalibration,true);

        _mode =Cal_ViewParameter_1;
        _showParameter();
    }

    void rotateForward(){
        if(_mode ==Cal_ViewParameter_1){
            // do nothing
        }else if(_mode ==Cal_ViewParameter_2){
            _mode =Cal_ViewParameter_1;
            _showParameter();
        }else if(_mode ==Cal_Back){
            if(ReadSetting(secondaryDoserSet) != SecondaryDoserDisabled){
                _mode =Cal_ViewParameter_2;                
            }else{
                _mode =Cal_ViewParameter_1;
            }
            _showParameter();
        }else if(_mode == Cal_SelectVolume){
            if(_calVolume >=10) _calVolume += 10;
            else _calVolume += 5;
            _updateVolumeDisplay(_calVolume);
        }else if(_mode == Cal_InputVolume){
            _realVolume += AdjustUnit;
            _updateVolumeDisplay(_realVolume);
        }
    }
    
    void rotateBackward(){
        if(_mode ==Cal_ViewParameter_1){
            if(ReadSetting(secondaryDoserSet) != SecondaryDoserDisabled){
                _mode =Cal_ViewParameter_2;                
            }else{
                _mode =Cal_Back;
            }
            _showParameter();
        }else if(_mode ==Cal_ViewParameter_2){
            _mode =Cal_Back;
            _showParameter();
        }else if(_mode ==Cal_Back){
        }else if(_mode == Cal_SelectVolume){
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
        if(_mode ==Cal_ViewParameter_1){
            // sart calibrate doser 1
            _startCalibrate(0);
        }else if(_mode ==Cal_ViewParameter_2){
            // sart calibrate doser 2
            _startCalibrate(1);
        }else if(_mode ==Cal_Back){
            return true;
        }else if(_mode == Cal_SelectVolume){
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
        }
        return false;
    }

    void dosingStateChanged(uint8_t doserId,bool dosing){
//        DBGPrint("Cal dosingStateChanged mode=");
//        DBGPrintln(_mode);

        if(_mode != Cal_RunDoser) return;

        if(dosing){
            _startTime = millis();
            _dosed=true;
            _dosing = true;
        }else{
            EditingText.noblink();
            _dosing =false;
            _accTime += millis() - _startTime;
            lcdPrint_P(1,1,strEnter,true);
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
    uint8_t  _doserId;
    DosingController *_controller;


    void _startCalibrate(uint8_t doserid){
        loadDosingControlParameter();
        _calVolume = 10;
        _doserId = doserid;
        _controller = _doserId? &dosingController2:&dosingController;
        DBGPrint("Calibrate Doser:");
        DBGPrintln(_doserId? "2":"1");
        EditingText.noblink();
        _enterSelectVolume();
    }
    

    void _showParameter(){
        lcdClearLine(1);
        if(_mode ==Cal_ViewParameter_1){
            if(ReadSetting(secondaryDoserSet) == SecondaryDoserDisabled){
                lcdPrint_P(2,1,strRate,true);
            }else{
                lcdPrint_P(2,1,strD1,true);
            }
            _showRate(DoserSetting(0,stepPerMl));
        }
        else if(_mode ==Cal_ViewParameter_2){
            lcdPrint_P(2,1,strD2,true);
            _showRate(DoserSetting(1,stepPerMl));            
        }
        else if(_mode ==Cal_Back){
            lcdPrint_P(2,1,strBack,true);          
        }
    }

    void _enterSelectVolume(){
        _mode = Cal_SelectVolume;
        _calVolume = 10;
        lcdPrint_P(1,1,strCalBy,true);
        if(ReadSetting(useWeight)) lcdWriteAt(14,1,'g');
        else lcdPrint_P(14,1,strMl);

        _updateVolumeDisplay(_calVolume);
    }
    
    void _enterCalibratingState(){
        EditingText.noblink();
        _dosed=false;
        _controller->resetDoseAdjustment();
        lcdClearLine(1);
        EditingText.setText_P(1,1,strRunDoser);
        EditingText.blink();
        _controller->startCalibrate();
        _controller->setMode(DosingModeManual);
        _accTime =0;
    }

    void _enterInputVolumeState(){
        _realVolume = _calVolume;
        lcdPrint_P(1,1,strAdjust,true);
        if(ReadSetting(useWeight)) lcdWriteAt(14,1,'g');
        else    lcdPrint_P(14,1,strMl);

        _updateVolumeDisplay(_realVolume);
        _controller->setMode(DosingModeDisabled);
    }

    void _updateVolumeDisplay(float vol){
        //lcdPrintAt(7,1,vol,6,2);
        EditingText.setNumber(7,1,vol,6,2);
        EditingText.blink();
    }

    void _finishCalibrate(){
        _controller->calibrate(_realVolume);
    }
    //0123456789012345
    //Rate  099.34ml/s
    void _showResult(){
        if( _doserId == 0) _mode = Cal_ViewParameter_1;
        else _mode = Cal_ViewParameter_2;
        _showParameter();
    }

    void _showRate(float stepml){
        float rate = 1000.0 / stepml ; // in g/s
        lcdPrintAt(6,1,rate,6,2);
        if(ReadSetting(useWeight)) lcdPrint_P(12,1,strGramPerSec);
        else lcdPrint_P(12,1,strMlPerSec);
    }
};

/*********************************************************************************/
// calibration
// micro adjustment
// to calculate Hysteresis by average
typedef enum _DoseCalibrationState{
    CS_Doser_Primary,
    CS_Doser_Secondary,
    CS_Back,
    CS_SelectAmout,
    CS_SelectCount,
    CS_ReadyToRun,
    CS_Running,
    CS_Adjust
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
const char strGo[] PROGMEM="Go";
//const char strAdjust[] PROGMEM="Adjust";

class DoseCalibration:public SugarApe{
public:
    DoseCalibration(){}
    ~DoseCalibration(){}

   void show(){
        loadDosingControlParameter();
        lcdPrint_P(TitleCol,TitleRow,strShotCalibration,true);
        _state = CS_Doser_Primary;
        _l1menu();
    }

    void rotateForward(){
        if(_state == CS_Doser_Primary){
            // do nothing
        }else if(_state == CS_Doser_Secondary){
            _state = CS_Doser_Primary;
            _l1menu();
        }else if(_state == CS_Back){
            _state = (ReadSetting(secondaryDoserSet) == SecondaryDoserDisabled)? CS_Doser_Primary:CS_Doser_Secondary;
            _l1menu();
        }else if(_state == CS_SelectAmout){
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

        if(_state == CS_Doser_Primary){
            _state = (ReadSetting(secondaryDoserSet) == SecondaryDoserDisabled)? CS_Back:CS_Doser_Secondary;
            DBGPrint("Primary-backward:");
            DBGPrintln(_state);
            _l1menu();           
        }else if(_state == CS_Doser_Secondary){
            DBGPrintln("Secondary-backward");
            _state = CS_Back;
            _l1menu();
        }else if(_state == CS_Back){
             // do nothing
        }else if(_state == CS_SelectAmout){
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
        if(_state == CS_Doser_Primary){
            _startCalibrate(0);
        }else if(_state == CS_Doser_Secondary){
            _startCalibrate(1);
        }else if(_state == CS_Back){
            return true;
        }else if(_state == CS_SelectAmout){
            // show
            // Count    100
            EditingText.noblink();
            lcdPrint_P(1,1,strCount,true); //rare case            
            _showTotalCount();
            _state = CS_SelectCount;
        }else if(_state == CS_SelectCount){
            EditingText.noblink();
            // show
            //0123456789012345
            //Run 10.12ml *100
            // the "count" numer shold already be there
            // show extra volume
            lcdPrint_P(1,1,strGo);
            lcdPrintAt(3,1,_amount,6,2);
            if(ReadSetting(useWeight)) lcdWriteAt(9,1,'g');
            else lcdPrint_P(9,1,strMl);
            lcd->setCursor(12,1);
            lcd->write('*');

            _state = CS_ReadyToRun;
        }else if(_state == CS_ReadyToRun){
            _displayCalibrating();
           _state = CS_Running;
            _dropCount = 0;

            _dosing = true;
            controller->resetDoseAdjustment();
            controller->setDosage(_amount);
            controller->dose();
        }else if(_state == CS_Running){
            /* do nothing */
        }else if(_state == CS_Adjust){
            EditingText.noblink();
            //  realAmount =  (_amount + x ) * _count
            // x = realAmount/_count - _amount
            // 
            //
            float adjust =_amount - _realAmount/(float)_count;
            controller->setDoseAdjustment( adjust );
            // return to l1menu
            _state = _doserId? CS_Doser_Secondary:CS_Doser_Primary;
            _l1menu();

        }
        return false;
    }

    void dosingStateChanged(uint8_t doserId,bool dosing){
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
            controller->dose();
        }
    }

protected:

    void _startCalibrate(uint8_t did){        
        _doserId = did;
        controller = _doserId? &dosingController2:&dosingController;
        _startEnteringAmount();
    }

    void _startEnteringAmount(){
        _state = CS_SelectAmout;
        _amount = 5;
        _count = 10;
        lcdPrint_P(TitleCol,TitleRow,strCalibrateBy,true);
        // show Amount
        lcdPrint_P(1,1,strAmount,true);
        if(ReadSetting(useWeight)) lcdWriteAt(14,1,'g');
        else lcdPrint_P(14,1,strMl);
        _showCalAmount();
    }
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
        lcdPrint_P(TitleCol,TitleRow,strCalibrating,true);
        lcdClear(0,1,11);
        lcd->setCursor(12,1);
        lcd->write('/');
        if(ReadSetting(useWeight)) lcdWriteAt(6,1,'g');
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
        lcdPrint_P(TitleCol,TitleRow,strCalibratedTo,true);
        lcdClearLine(1);
        if(ReadSetting(useWeight)) lcdWriteAt(14,1,'g');
        else lcdPrint_P(14,1,strMl);
        _showAjustedAmount();
    }

    void _showAjustedAmount(){
        //lcdPrintAt(7,1,_realAmount,6,2);
        EditingText.setNumber(7,1,_realAmount,6,2);
        EditingText.blink();
    }
    void _showAdjustValue(float adjust){
            //0123456789012345
            //Adjsut   -0.12ml
        lcdPrintAt(9,1,adjust,5,2);
        if(ReadSetting(useWeight)) lcdWriteAt(14,1,'g');
        else lcdPrint_P(14,1,strMl);
    }


    void _l1menu(){
            DBGPrint("_l1menu:");
            DBGPrintln(_state);

        lcdClearLine(1);
        if(_state == CS_Doser_Primary){
            if(ReadSetting(secondaryDoserSet) == SecondaryDoserDisabled){
                lcdPrint_P(2,1,strAdjust,false);
            }else{
                lcdPrint_P(2,1,strD1,false);
            }
            _showAdjustValue(DoserSetting(0,shotAdjustment));

        }else if(_state == CS_Doser_Secondary){
            lcdPrint_P(2,1,strD2,false);
            _showAdjustValue(DoserSetting(1,shotAdjustment));

        }else if(_state == CS_Back){
            lcdPrint_P(2,1,strBack);
        }
    }

    uint32_t _dropEndTime;
    uint16_t _count;
    uint16_t _dropCount;
    float _amount;
    float _realAmount;
    DoseCalibrationState _state;
    bool _dosing;
    DosingController *controller;
    uint8_t _doserId;
};

/*********************************************************************************/
// dose setting, including
// delay start, cool time, trigger type
#define MaximumDelayTime 10
#define MaximumCoolTime 10   // in 0.5 seconds

const char strDosing[] PROGMEM ="Dosing";
const char strControl[] PROGMEM ="Control";
const char strDelay[] PROGMEM ="Delay";
const char strCoolTime[] PROGMEM ="CoolTime";

const char strButton[] PROGMEM ="Button";
const char strSensor[] PROGMEM ="Sensor";
/*
0123456789012345
Dosing Secondary
TriggerSecondary
 Doser 
*/

enum TSState{
    TS_TrigerType=0,
    TS_DelayTime,
    TS_CoolTime,
    TS_Back
};
class TriggerSettings:public SugarApe{
public:
    TriggerSettings(){}
    ~TriggerSettings(){}

    void show(){
        _doserId = 0;
        _editing =false;
        _state = TS_TrigerType;
        _displayItems();
    }
    
    void rotateForward(){
        if(_editing){
            if(_state == TS_TrigerType){
                if(_trigger==0){
                    _trigger=1;
                    _displayTrigerType();
                    _dirty = true;
                }
            }else{ // if(_state == TS_DelayTime || TS_CoolTime){
                if(_timeval < MaximumDelayTime){
                    _timeval ++;
                    _displayTime(_timeval);
                    _dirty = true;
                }
            } 
        }else{
            // non editing
            if(ReadSetting(secondaryDoserSet) == SecondaryDoserIndependent &&  _doserId == 1 && _state == TS_TrigerType){
                _doserId =0;
                _state = TS_CoolTime;
                _displayItems();
            }else if((int)_state > (int)TS_TrigerType){
                _state =(TSState) ((int)_state -1);
                _displayItems();
            }
        }
    }

    void rotateBackward(){
        if(_editing){
            if(_state == TS_TrigerType){
                if(_trigger!=0){
                    _trigger=0;
                    _displayTrigerType();
                    _dirty = true;
                }
            }else{ // the same for DelayTime and CoolTime if(_state == TS_DelayTime){
                if(_timeval > 0){
                    _timeval --;
                    _displayTime(_timeval);
                    _dirty = true;
                }
            }
            
        }else{
            // non editing
            if( ReadSetting(secondaryDoserSet) == SecondaryDoserIndependent && _doserId ==0 && _state==TS_CoolTime){
                _doserId =1;
                _state =TS_TrigerType;
                _displayItems();
            }else if((int)_state < (int)TS_Back){
                _state =(TSState) ((int)_state +1);
                _displayItems();
            }
        }

    }

    bool switchPushed(){
        if(_editing){
            _editing=false;
            EditingText.noblink();

            if(_state == TS_TrigerType){
                if(_dirty) UpdateDoserSetting(_doserId,triggerType, _trigger);
            }else if(_state == TS_DelayTime){
                if(_dirty) UpdateDoserSetting(_doserId,delayTime,_timeval);
            }else if(_state == TS_CoolTime){
                 if(_dirty) UpdateDoserSetting(_doserId,coolTime,_timeval);
            }
            _dirty  = false;

        }else{
            if(_state == TS_Back){
                SettingManager.save();
                //dosingController.loadSetting();
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
    uint8_t _trigger;
    bool _dirty;
    uint8_t _timeval;
    uint8_t _doserId;

    void _displayItems(){
        if(_state == TS_TrigerType){
            lcdPrint_P(1,0,strDosing);
            if(ReadSetting(secondaryDoserSet) == SecondaryDoserIndependent)
                lcdPrint_P(7,0,(_doserId ==0)? strPrimary:strSecondary,true);
            lcdPrint_P(2,1,strControl,true);
            _trigger = DoserSetting(_doserId,triggerType);
            _displayTrigerType();

        }else if(_state == TS_DelayTime){
            lcdPrint_P(2,1,strDelay,true);
            lcd->setCursor(1,15);
            lcd->write('s');
            _timeval = DoserSetting(_doserId,delayTime);
            if(_timeval > MaximumDelayTime) _timeval = MaximumDelayTime;
            _displayTime(_timeval); 
        }else if(_state == TS_CoolTime){
            if(ReadSetting(secondaryDoserSet) == SecondaryDoserIndependent) lcdPrint_P(7,0,(_doserId ==0)? strPrimary:strSecondary,true);
            lcdPrint_P(2,1,strCoolTime,true);
            lcd->setCursor(1,15);
            lcd->write('s');
            _timeval = DoserSetting(_doserId,coolTime);
            if(_timeval > MaximumDelayTime) _timeval = MaximumDelayTime;
            _displayTime(_timeval);            
        }else{ // _state == TS_Back
            lcdPrint_P(2,1,strBack,true);
        }
        _dirty = false;
    }    
    void _displayTrigerType(){
        if(_trigger){
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
/*        DBGPrint(F("Time of "));
        DBGPrint(_doserId);
        DBGPrint(F(": "));
        DBGPrintln(val); */
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
    SSS_DoseEnd=2,
    SSS_Back=3
};

const char strBuzz[] PROGMEM="Buzz";
const char strDoseStart[] PROGMEM="Dose Start";
const char strDoseEnd[] PROGMEM="Dose End";

const char strOn[] PROGMEM="ON";
const char strOff[] PROGMEM="OFF";

const char* const SoundSettingLabels[]={
    strButton,
    strDoseStart,
    strDoseEnd
};

class SoundSetting:public SugarApe{
public:
    SoundSetting(){}
    ~SoundSetting(){}

    void show(){
        _editing=false;
        _state = SSS_Button;
        _doserId=0;
        lcdPrint_P(1,0,strBuzz,true);
        
        _displayItem();
    }
    
    void rotateForward(){
        if(_editing){
            if(!_on){
                _on = true;
                _dirty = true;
                EditingText.hide();
                _displayOnOff();
            }
        }else{
            if(ReadSetting(secondaryDoserSet) == SecondaryDoserIndependent && _doserId ==1 && _state == SSS_Button){
                _doserId =0;
                _state = SSS_DoseEnd;
                _displayItem();
            }else if((uint8_t)_state >(uint8_t)SSS_Button){
                _state = (SoundSettinState)( (uint8_t)_state - 1);
                _displayItem();
            }
        }
    }

    void rotateBackward(){
        if(_editing){
            if(_on){
                _on = false;
                _dirty = true;
                EditingText.hide();
                _displayOnOff();
            }
        }else{
            if(ReadSetting(secondaryDoserSet) == SecondaryDoserIndependent&& _doserId ==0 && _state == SSS_DoseEnd){
                _doserId =1;
                _state = SSS_Button;
                _displayItem();
            }else 
            if((uint8_t)_state <(uint8_t)SSS_Back){
                _state = (SoundSettinState)( (uint8_t)_state + 1);
                _displayItem();
            }
        }
    }

    bool switchPushed(){
        if(_editing){
            _editing = false;
            EditingText.noblink();
            
            if(_state == SSS_Button) UpdateDoserSetting(_doserId,beepButton, _on? 1:0);
            else if(_state == SSS_DoseStart)  UpdateDoserSetting(_doserId,beepDoseStart,  _on? 1:0);
            else UpdateDoserSetting(_doserId,beepDoseEnd, _on? 1:0);

            _dirty = false;
        }else{
            if(_state == SSS_Back){
                SettingManager.save();
                return true;
            }else{
                _editing = true;
                EditingText.blink();
            }

        }
        return false;
    }

protected:
    SoundSettinState _state;
    bool _on;
    bool _editing;
    bool _dirty;
    uint8_t _doserId;

    void _displayItem(){
        if(_state != SSS_Back){
            if(ReadSetting(secondaryDoserSet) == SecondaryDoserIndependent) lcdPrint_P(7,0,(_doserId==0)? strPrimary:strSecondary,true);
            lcdPrint_P(2,1,SoundSettingLabels[_state],true);
            if(_state == SSS_Button) _on =DoserSetting(_doserId,beepButton) != 0;
            else if(_state == SSS_DoseStart)  _on =DoserSetting(_doserId,beepDoseStart) != 0;
            else _on =DoserSetting(_doserId,beepDoseEnd) != 0;
            _dirty =false;
            _displayOnOff();
        }else{
            lcdPrint_P(2,1,strBack,true);
        }
    }

    void _displayOnOff(){
        if(_on){
            EditingText.setText_P(14,1,strOn);
        }
        else  EditingText.setText_P(13,1,strOff);
        EditingText.show();
    }

};


/*********************************************************************************/
// Unit Setting

const char strUse[] PROGMEM="Use";
const char strWeight[] PROGMEM="Weight";
const char strVolume[] PROGMEM="Volume";

class UnitSetting:public SugarApe{
public:
    UnitSetting(){}
    ~UnitSetting(){}


    void show(){
        _idx=0;
        _editing = false;
        _dirty = false;
        lcdPrint_P(1,0,strUnitSetting,true);
        _showItem();
    }
    
    void rotateForward(){
        if(_editing){
            if(_useWeight ==0){
                _useWeight =1;
                _printValue();
                _dirty =true;
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
            if(_useWeight !=0){
                _useWeight =0;
                _printValue();
                _dirty =true;
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
            if(_dirty){
                WriteSetting(useWeight,_useWeight);
            }
        }else{
            if(_idx ==0){
                _editing=true;
                EditingText.blink();
            }else{
                if(_dirty) SettingManager.save();
                return true;
            }
        }
        return false;
    }

protected:
    bool _editing;
    bool _dirty;
    uint8_t _useWeight;
    uint8_t _idx;
    void _showItem(){
        if(_idx ==0){
            lcdPrint_P(2,1,strUse,true);
            _useWeight =ReadSetting(useWeight);
            _printValue();
        }else{
             lcdPrint_P(2,1,strBack,true);
        }

    }
    void _printValue(){
        EditingText.setText_P(10,1, _useWeight? strWeight:strVolume);
        EditingText.show();
    }
};

//***********************************************************************
// Bottles Setting


const char strBottleSetting[] PROGMEM="Bottles";

class BottleSetting:public SugarApe{
protected:
    bool _editing;
    bool _dirty;
    uint8_t _idx;
    int _currentValue;
    //0123456789012345
    //#10      2000 ml

    void _showItem(){
        lcdClearLine(1);
        if(_idx == NumberOfBottles){
            lcdPrint_P(12,1,strBack,true);
            return;
        }
        
        lcdPrintAt(0,1,_idx,2);
        lcdWrite(':');
        lcdPrint_P(14,1,strCl);

        _currentValue = BottleList.getBottle(_idx);
        
        if(_currentValue > MaxVolume || _currentValue< MinVolume) _currentValue =0;

        _printValue();
    }
    void _printValue(){
        EditingText.setNumber(9,1,_currentValue/10,4);
        EditingText.show();
    }
public:
    BottleSetting(){}
    ~BottleSetting(){}


    void show(){
        _idx=0;
        _editing = false;
        _dirty = false;
        lcdPrint_P(1,0,strBottleSetting,true);
        _showItem();
    }
    
    void rotateForward(){
        if(_editing){
            
            if( (_currentValue + 10) < MaxVolume){
                _currentValue +=10;
                _dirty =true;
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
            if( (_currentValue -10)> MinVolume){
                _currentValue -= 10;
                _dirty =true;
                _printValue();
            }
        }else{
            if(_idx < NumberOfBottles){
                _idx ++;
                _showItem();
            }
        }
    }

    bool switchPushed(){
        if(_editing){
            _editing=false;
            EditingText.noblink();
            if(_dirty){
                BottleList.setBottle(_idx,_currentValue);
            }
        }else{
            if(_idx < NumberOfBottles){
                _editing=true;
                EditingText.blink();
            }else{
                return true;
            }
        }
        return false;
    }


};
/*************************************************************************/
// Carbonation Settings
/*************************************************************************/
// setting of 
//  - Input Beer/Sugar  0
//  - Beer Temp          1
//  - Co2 Volume         2

//  - By Sugar/Krausen 3
//  - Sugar Brix   4
// or 
//  - Krausen SG  4 
//  - Krausen FG  5

#define IndexInput 0
#define IndexBeerTemp 1
#define IndexCo2Volume 2

#define IndexPrimeBy 3

#define IndexSugarOGravity 4
#define IndexBackOrFG 5
#define IndexBack 6


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

const char strPrimeBy[] PROGMEM="With";
const char strSuger[]   PROGMEM="  Sugar";
const char strKrausen[] PROGMEM="Krausen";

const char strKrausenSG[] PROGMEM="KrausenSG";
const char strKrausenFG[] PROGMEM="KrausenFG";

/*
0123456789012345
 Input  Beer vol
           Sugar
 Brix     012.Bx
 Co2 Vol.    3.2
 BeerTemp   12.C
 With      Sugar
         Krausen
 Krausen SG 1048
 Krausen FG 1012
 */
class PrimingSetting:public SugarApe{
public:
    PrimingSetting(){}
    ~PrimingSetting(){}
      void show(){
        _editing = false;
        _setIdx =0;
        lcdPrint_P(1,0,strCarbonation,true);
        _showItem();
    }

    void rotateForward(){
        if(_editing){
            if(_setIdx ==IndexInput){
                if(_inputBeer) _inputBeer=0;
                _printInputValue();
            }else if(_setIdx ==IndexBeerTemp){
                if(_beerTemperature < MaxBeerTemp) _beerTemperature ++;
                _printBeerTemp();
            }else if(_setIdx ==IndexCo2Volume){
                if(_carbonation < HighestCarbonation) _carbonation += 1;
                _printCo2Volume();
            }else if (_setIdx == IndexPrimeBy){
                _krausen = _krausen? 0:1;
                _printPrimeMethod();
            }else if(_setIdx ==IndexSugarOGravity){
                if(_krausen){
                    _gravity += 1;
                    _printGravity();
                }else{
                    if(_sugarRatio<100) _sugarRatio ++;
                    _printSugarRatio();
                }
            }else if(_setIdx ==IndexBackOrFG){
                if(_krausen){
                    _gravity += 1;
                    _printGravity();
                }
            }
            _dirty = true;
        }else{
            if(_setIdx > IndexInput){
                _setIdx --;
                _showItem();
            }
        }

    }
    
    void rotateBackward(){
        if(_editing){
            if(_setIdx ==IndexInput){
                if(_inputBeer ==0) _inputBeer=1;
                _printInputValue();
            }else if(_setIdx ==IndexBeerTemp){
                if(_beerTemperature > MinBeerTemp) _beerTemperature --;
                _printBeerTemp();
            }else if(_setIdx ==IndexCo2Volume){
                if(_carbonation > LowestCarbonation) _carbonation -= 1;
                _printCo2Volume();
            }else if (_setIdx == IndexPrimeBy){
                _krausen = _krausen? 0:1;
                _printPrimeMethod();
            }else if(_setIdx ==IndexSugarOGravity){
                if(_krausen){
                    _gravity -= 1;
                    _printGravity();
                }else{
                    if(_sugarRatio>0) _sugarRatio --;
                    _printSugarRatio();
                }
            }else if(_setIdx ==IndexBackOrFG){
                if(_krausen){
                    _gravity -= 1;
                    _printGravity();
                }
            }

            _dirty =true;
        }else{
            if( (_krausen && (_setIdx<IndexBack)) || (!_krausen && (_setIdx < IndexBackOrFG))){
                _setIdx ++;
                _showItem();
            }
        }
    }
    bool switchPushed(){
        if(_editing){
            _editing = false;
            EditingText.noblink();
            if(_setIdx ==IndexInput){
                if(_dirty) WriteSetting(inputBeer,_inputBeer);
            }else if(_setIdx ==IndexCo2Volume){
                if(_dirty) WriteSetting(carbonation,_carbonation);
            }else if(_setIdx ==IndexBeerTemp){
                if(_dirty) WriteSetting(beerTemperature,_beerTemperature);
            } if(_setIdx ==IndexPrimeBy){
                if(_dirty) WriteSetting(krausening,_krausen);
            } if(_setIdx ==IndexSugarOGravity){
                if(_krausen){
                    if(_dirty) WriteSetting(krausenGravity,_gravity);
                }else{
                    if(_dirty) WriteSetting(sugarRatio,_sugarRatio);
                }
            } if(_setIdx ==IndexBackOrFG){
                if(_krausen){
                    if(_dirty) WriteSetting(krausenFG,_gravity);
                }
            }
            _dirty =false;

        }else{
            if( (_krausen && (_setIdx==IndexBack)) || (!_krausen && (_setIdx == IndexBackOrFG))){
                SettingManager.save();
                return true;
            }else{
                _editing = true;
                EditingText.blink();
            }
        }
        return false;
    }
protected:
    uint8_t _setIdx;
    bool _editing;
    bool _dirty;
    int8_t _beerTemperature;
    uint8_t _carbonation;
    uint8_t _sugarRatio;
    uint8_t _inputBeer;
    uint8_t _krausen;
    uint16_t _gravity;

    void _showItem(){
        if(_setIdx ==IndexInput){
            lcdPrint_P(2,1,strInput);
            _inputBeer = ReadSetting(inputBeer);
            _printInputValue();
        }else if(_setIdx ==IndexCo2Volume){
            lcdPrint_P(2,1, strCo2Vol, true);
            _carbonation = ReadSetting(carbonation);
            if(_carbonation < LowestCarbonation) _carbonation = LowestCarbonation;
            else if(_carbonation > HighestCarbonation) _carbonation = HighestCarbonation;
            _printCo2Volume();
        }else if(_setIdx ==IndexBeerTemp){    
            lcdPrint_P(2,1,strBeerTemp,true);
            lcd->setCursor(14,1);
            lcd->write(DegreeChar);
            lcd->write('C');
            _beerTemperature = ReadSetting(beerTemperature);
            _printBeerTemp();
        }else if(_setIdx == IndexPrimeBy){
            lcdPrint_P(2,1,strPrimeBy,true);
            _krausen = ReadSetting(krausening);
            _printPrimeMethod();
        }else if(_setIdx ==IndexSugarOGravity){
            if(_krausen){
                lcdPrint_P(2,1,strKrausenSG,true);
                _gravity = ReadSetting(krausenGravity);
                if(_gravity <990 || _gravity > 1150) _gravity = 1050;
                _printGravity();
            }else{
                lcdPrint_P(2,1,strBrix,true);
                lcd->setCursor(13,1);
                lcd->write(DegreeChar);
                lcd->write('B');
                lcd->write('x');
                _sugarRatio = ReadSetting(sugarRatio);
                if(_sugarRatio>100) _sugarRatio=100;
                _printSugarRatio();
            }
        }else if(_setIdx ==IndexBackOrFG){
            if(_krausen){
                lcdPrint_P(2,1,strKrausenFG,true);
                _gravity = ReadSetting(krausenFG);
                if(_gravity <990 || _gravity > 1150) _gravity = 1012;
                _printGravity();
            }else{
                lcdPrint_P(2,1,strBack,true);
            }
        }else if(_setIdx == IndexBack){
            lcdPrint_P(2,1,strBack,true);
        }
        _dirty = false;
    }

    void _printInputValue(){
        if(_inputBeer){
            EditingText.setText_P(8,1,strBeerVol);
        }else{
            EditingText.setText_P(8,1,strXSugar);
        }
        EditingText.show();
    }

    void _printSugarRatio(){
        EditingText.setNumber(10,1,_sugarRatio,3);
        EditingText.show();
    }
    void _printCo2Volume(){
        EditingText.setNumber(13,1,Set2CV(_carbonation),3,1);
        EditingText.show();
    }

    void _printBeerTemp(){
        EditingText.setNumber(12,1,_beerTemperature,2);
        EditingText.show();
    }

    void _printPrimeMethod(){
        if(_krausen){
            EditingText.setText_P(8,1,strKrausen);
        }else{
            EditingText.setText_P(8,1,strSuger);
        }
        EditingText.show();
    }
    void _printGravity(){
        EditingText.setNumber(12,1,_gravity,4);
        EditingText.show();
    }

};

/*************************************************************************/
/* Secondary doser setting */
/*
0123456789012345
 SecondaryDoser
*/
const char strSecondaryDoser[] PROGMEM="SecondaryDoser";
const char strSet[] PROGMEM =  "";
const char strDisabled[]    PROGMEM = "Disabled";
const char strSyncronized[] PROGMEM = "Synchron";
const char strSeperate[] PROGMEM    = "Seperate";

//const char strYes[] PROGMEM = "Yes";
//const char strNo[] PROGMEM =  "No ";

const char strRatio[] PROGMEM = "Auto(%)";
const char strTrigger[] PROGMEM =  "Trigger";


#define SecondaryIndexEnable 0
#define SecondaryIndexRatio 1
#define SecondaryIndexBack 2

#define SecondaryIndexRatioInputInteger 4
#define SecondaryIndexRatioInputFraction 5


class SecondarySetting:public SugarApe{
public:
    SecondarySetting(){}
    ~SecondarySetting(){}

    void show(){
        lcdPrint_P(TitleCol,TitleRow,strSecondaryDoser);
        _setIdx=SecondaryIndexEnable;
        _editing=false;
        _showItems();
    }

    void rotateForward(){
        if(_editing){
             if(_setIdx == SecondaryIndexEnable){
                 if(_2ndDoserSet > 0){
                     _2ndDoserSet --;
                     _show2ndDoserSet();
                 }
             }else if(_setIdx ==SecondaryIndexRatioInputInteger){
                 if(_ratioInteger < MaximumDoserRatioInteger){
                     _ratioInteger ++;
                     _updateIntegerPart();
                 } 
             }else{
                _ratioFraction ++;
                if(_ratioFraction > 99){
                     _ratioFraction=0;
                }
                _updateFractionPart();
             }

        }else{
            if(_setIdx > 0){
                _setIdx--;
                _showItems();
            }
        }
    }

    void rotateBackward(){
        if(_editing){
             if(_setIdx == SecondaryIndexEnable){
                 if( (_2ndDoserSet +1) <= SecondaryDoserIndependent){
                     _2ndDoserSet ++;
                     _show2ndDoserSet();
                 }            
            }else if(_setIdx ==SecondaryIndexRatioInputInteger){
                 if(_ratioInteger > 0){
                     _ratioInteger --;
                     _updateIntegerPart();
                 } 
             }else{ // fraction
                _ratioFraction --;
                if(_ratioFraction < 0){
                    _ratioFraction =99;
                } 
                _updateFractionPart();
             }
        }else{
            if(_setIdx <SecondaryIndexBack){
                _setIdx ++;
                _showItems();
            }
        }
    }
    
    bool switchPushed(){
        if(_editing){
            if(_setIdx == SecondaryIndexEnable){
                EditingText.noblink();
                WriteSetting(secondaryDoserSet,_2ndDoserSet);
                _editing=false;
                dosingController2.setButton((ReadSetting(secondaryDoserSet) == SecondaryDoserSync)? switchButton1:switchButton2);
            }else if(_setIdx ==SecondaryIndexRatioInputInteger){
                EditingText.noblink();
                _updateFractionPart();
                EditingText.blink();                
                _setIdx = SecondaryIndexRatioInputFraction;
            }else{ //IndexRatioInputFraction
                EditingText.noblink();
                _ratio = (float) _ratioInteger  + (float)_ratioFraction/100.0;
                DBGPrint(F("Ratio:"));
                DBGPrintln(_ratio);
                WriteSetting(secondaryDosageRatio,_ratio/100.0);
                _editing=false;
                _setIdx = SecondaryIndexRatio;
            }
        }else{
            if(_setIdx == SecondaryIndexBack){
                return true;
            }else{
                _editing=true;

                if(_setIdx == SecondaryIndexEnable){
                    EditingText.blink();
                }else{
                    // editing the ratio might be complicated.
                    // let's use two part, integer and fraction(decimal)
                    _setIdx= SecondaryIndexRatioInputInteger;
                    _ratioInteger = floor(_ratio);
                    _ratioFraction =(int)((_ratio - _ratioInteger)*100.0 + 0.5);
                    if(_ratioFraction > 99) _ratioFraction =99;
                    DBGPrint(F("Ratio:"));
                    DBGPrint(_ratio);
                    DBGPrint(F(" int:"));
                    DBGPrint(_ratioInteger);
                    DBGPrint(F(" fra:"));
                    DBGPrintln(_ratioFraction);

                    _updateIntegerPart();
                    EditingText.blink();
                }
            }
        }
        return false;
    }

protected:
    uint8_t _2ndDoserSet;
    uint8_t _setIdx;
    bool _editing;
    float _ratio;
    int   _ratioInteger;
    int   _ratioFraction;

    void _show2ndDoserSet(){
        if(_2ndDoserSet == SecondaryDoserDisabled) EditingText.setText_P(8,1,strDisabled);
        else if(_2ndDoserSet == SecondaryDoserSync) EditingText.setText_P(8,1,strSyncronized);
        else EditingText.setText_P(8,1,strSeperate);

        EditingText.show();
    }

    void _showItems(){
        if(_setIdx == SecondaryIndexEnable){
            lcdPrint_P(2,1,strSet,true);
            
            _2ndDoserSet = ReadSetting(secondaryDoserSet);
            _show2ndDoserSet();        
        }else if(_setIdx ==  SecondaryIndexRatio){
            lcdPrint_P(2,1,strRatio,true);
            
            _ratio = ReadSetting(secondaryDosageRatio) * 100.0;
            if(_ratio < 0) _ratio =0;
            lcdPrintAt(10,1,_ratio,6,2); //100.99
        }else{
            lcdPrint_P(2,1,strBack,true);
        }
    }
    void _updateIntegerPart(){
        EditingText.setNumber(10,1,_ratioInteger,3);
    }
    void _updateFractionPart(){
        EditingText.setNumberLeading(14,1,_ratioFraction,2,'0');
    }
};
/*************************************************************************/
// Main
/*************************************************************************/
RotaryEncoder encoder(RA_CLK_PIN,RB_DT_PIN,SW_PIN);

class ApeController{
public:
    ApeController():_menuHandler(&MainMenu){

    }
    ~ApeController(){}

    void begin(){
        SettingManager.begin();
        Buzzer.begin();
        lcdInitialize();
//        encoder.setReversedDirection(true);

        switchButton1.begin(BUTTON_PIN);
        switchButton2.begin(BUTTON2_PIN);

        dosingController.begin(switchButton1,PUMP_PIN,DOSED_INDICATOR_PIN);
        if(ReadSetting(secondaryDoserSet) == SecondaryDoserSync){
            dosingController2.begin(switchButton1,PUMP2_PIN,DOSED_INDICATOR2_PIN);
            dosingController2.setParameter(& Settings.doser[0]);

        }else{
            dosingController2.begin(switchButton2,PUMP2_PIN,DOSED_INDICATOR2_PIN);
        }
        _running = &_menuHandler;
   
        _running->show();
    }

    void loop(){
        bool changed;
		RotaryEncoderStatus status=encoder.read();
  		switch(status)
  		{
    		case RotaryEncoderStatusDepushed: //RotaryEncoderStatusPushed:
    			changed = _running->switchPushed();                
                if(changed){
                    switchTo(_running->nextApp());
                }
    			break;

            case RotaryEncoderStatusLongPressed:
                _running->switchLongPressed();
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
            _running->dosingStateChanged(0,dosingController.isDosing());
        }

        if(dosingController2.isDosingStateChanged()){
            _running->dosingStateChanged(1,dosingController2.isDosing());
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
                _running= & _priming;
                break;
            case SugarAppSecondarySetting:
                _running= & _secondarySetting;
                break;
            case SugarAppBottleVolume:
                _running= & _bottleSetting;
                break;
            case SugarAppRunDoser:
                _running = & _runDoser;
        }
        dosingController.setMode(DosingModeDisabled);
        dosingController2.setMode(DosingModeDisabled);

        lcd->clear();
        _running->show();
    }
protected:

    SugarApe *_running;
    MenuHandler _menuHandler;
    DoseCalibration _doseCalibrator;
    SugarCalibrator _sugarCalibrate;
    AutoDoser _autoDoser;
    ManualDoser _manualDoser;
    TriggerSettings _dropSetting;
    SoundSetting _soundSetting;
    UnitSetting _unitSetting;
    PrimingSetting _priming;
    SecondarySetting _secondarySetting;
    BottleSetting _bottleSetting;
    RunDoser       _runDoser;
};