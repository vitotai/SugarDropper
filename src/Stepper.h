

class Stepper{
public:
    Stepper(byte stepPin,byte dirPin);
    ~Stepper(){}

    uint32_t run();
    void stop();
    uint32_t runSteps(unsigned long step);
    uint32_t steps();
    bool running();

    void setDirectiionForward(bool forward);
    void setRPM(unsigned long rpm);

    void begin();
    uint32_t startTime();

    void timerTick();
protected:
    byte _actuatorPin;
    bool _running;
    bool _autoStop;
    uint32_t _autoStopTime;

    uint32_t _accRunTime;
    uint32_t _startTime;

    void _powerOn();
    void _powerOff();
};