# SugarDropper
SugarDropper is designed for homebrewers to to dose precise sugar solution into bottles.
Batch priming by using bottling bucket or not is simple, but the introduce of oxygen is inevitable, i.e. either at the time the beer runs into bucket or fermenters are opened and sugar is dropped. 
Adding priming sugar into bottles alleviates oxygen problem a little bit, but it is PITA, which is the second worst thing for me in bottling. SugarDropper is a simple project that help me in bottling day.

## Hardware Component
- Arduino Nano or Uno
- HD44780 type 16x2 I2C LCD
- KY-040 rotary encoder module
- One way Relay module, or two way relay module
- Buzzer
- Button or QT30CM infra-red sensor
- Peristaltic pump with tube, two for secondary doser
- wires, box, and etc.

Connections

Pins    |  connects to | Note
--------|:-----|------------------------
D2   | Button or QT30CM input | 
D3   | Secondary Doser button/sensor |
D5   | Relay Module IN |
D6   | Secondary Relay Module IN |
D9   | CLK of K040 |
D10  | DT of K040 |
D11  | SW of K040 |
D12  | Buzzer |
A4   | SDA of LCD |
A5   | SCL of LCD |


## Operation & Menu
First level menu: ( rotate rotary encoder to change selection, push rotary encoder to select, also push to exit current application.)

Item   | Description
----|:------------------
Automatic | Automatic mode. The speicifed amount of sugar solution will run out when triggered, button pressed or sensor triggered. Changing the amount by the rotary encoder.
Manual    | Manually start/stop running.
Settings  | see next table

Setting Items   | Description
----------|:------------------
Calibration | To clibrate the flow rate of Peristaltic pump. First, input the rough amount, then push button to start running. When rough amount is output, push the button to stop. Measure the real amount by a scale, then input real amount. (The way to start and stop running depends on the triggering setting.)
Dose Calibrate | To derive a correction for one cycle of start and stop. First, input the amount of each drop, and then the times. Push enter to start. When finished, input the real amount of sugar solution dropped. Note, the correction value is reset to zero when "Calibration" procedure is performed. (If a slow peristaltic pump is used, it is not really necessary.) 
Dosing Control | The way to trigger sugar running. **Control**: Button or Sensor. **Delay**: the delay to run the pump. **Cool Time**: minimum gap between dropping. 
Priming   | **Input**: input of **Beer Vol** or **sugar** amount. **Brix**: sugar solution density. **CO2 Vol.**: desired Co2 volume. **Beer Temp.**: beer temperature to calculate priming sugar.
Sound Setting | Buzz or not on events: **Button** (pressed), **Dose Start**, **Dose End**.
Unit | **Volume** or Weight. If the density of sugar solution is even. The volume is proportional to weight. However, different density of sugar solution has different flow rate according to my experiments. 
2nd Doser | Enable or Disable secondary doser. The **Auto(%)** is used for automatic mode to calculate output of secondary. If **beer vol** is selected in **Priming** setting, beer volume input is used to calculate the output of secondary doser. If **Sugar** is selected, sugar amount is used to calculate. The unit is %, so 1 means 1%. eg. 3.3g or ml for 330ml beer volume, or 0.03g for 3g sugar amount. If the value is set to 0(zero), then there is no relation with primary doser. You will/can input the amount of secondary doser manually.

### Consideration of Peristaltic pump

Peristaltic pump with DC motor is used in this project for its low cost and easy-to-control. Depending on the speed of motor and tubing size, peristalitc pumps have different flow rates. Obviously, the higher the rate, the quicker the running. If the bottles are hand held to catch the sugar, dropping of sugar must be quick enough. Otherwise, a few seconds of dropping is fine.
IMO, in 1 second and 3 seconds for hand-held and in-place are reasonable. If 3.5 volume is desired, and the beer is fermented at 20C, 3.5g priming sugar is needed. Given 50Brix sugar solution, gravit is around 1.223, is used. the volume to rung is around
` 3.5/1.233/50%=5.68ml `
So, in 1 and 3 seconds, the flow rates are
`5.68/1*60 = 340.8 ml/min`  and `5.683*60 = 113.6 ml/min`

The is the rough requirement of peristaltic pumps.

(One second is **longer** than you might have thought. Try minicing bottling action, and insert an action of holding the bottle for one second. You will see.)

### Control
Button is simple to use. Press the button to dose. In manual mode, press to start, and press again to stop. "Delay" is usually zero and "cool time" can be zero, too.
Sensor works differently. A small value of delay is necessary because usually the sensor senses before the bottle is really in place. Non-zero Cool Time is good for preventing mis-triggering. In Manual mode, to stop running, move the bottle/container out of the sensor. In automatic mode, running is stop if the snesor detects the container is out of place. 

### Secondary Doser
Secondary doser might be helpful to dose tinctured spice.
In manual mode, secondary doser runs independantly. While in Auto mode, the dosage of secondary doser might be linked to primary or not. If it is not linked, i.e. the "Auto(%)" value is 0(zero), then **Long Press** the rotary encoder to switch input to secondary dosage. **Long press** again to back to normal primary dosage input.

# Tips
- Run manual mode to suck liquid to tube before running or calibration.
- Different solutions have different viscosity and different flow speed. Don't calibrate only with water. In my experience, sugar solution of 70Brix is like 1/4 rate of water, while 50Brix is almost equal to water. 
- QT30CM has different types. You should choose NPN Normal Open one, which outputs **LOW** when blocked.

# Note
It is a simple but helpful, yet fun, project. Use the code as you like, and modify the code whatever you want.
Don't ask for Imperial unit support.
