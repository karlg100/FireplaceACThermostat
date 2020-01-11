// This #include statement was automatically added by the Particle IDE.
#include <Encoder.h>

// This #include statement was automatically added by the Particle IDE.
#include <Grove_4Digit_Display.h>

// This #include statement was automatically added by the Particle IDE.
#include <Adafruit_Si7021.h>

// This #include statement was automatically added by the Particle IDE.
#include <pid.h>

Adafruit_Si7021 sensor = Adafruit_Si7021();

#define MIN_TEMP 30
#define MAX_TEMP 80
#define DEFAULT_TARGETTEMP 70
#define SWING_TEMP 2

// ** PID Setup
//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Define the aggressive and conservative Tuning Parameters
double aggKp=4, aggKi=0.2, aggKd=1;
double consKp=1, consKi=0.05, consKd=0.25;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, PID::DIRECT);

// LCD display
#define CLK D2//pins definitions for TM1637 and can be changed to other ports
#define DIO D3
TM1637 tm1637(CLK,DIO);

// time tracking vars
size_t lastReport = 0;
size_t overrideTimer = 0;

// manual override rotary encoder
// Change these two numbers to the pins connected to your encoder.
// Both pins must have interrupt capability
Encoder myEnc(D4, D5);
//   avoid using pins with LEDs attached


//define ROTARY_ANGLE_SENSOR A0
//define ROTARY_DEBOUNCE 200
//int lastManualTemp = 0;
long lastRotary = 0;
//define OVERRIDE_TIME 10000
#define OVERRIDE_TIME 3600000

// working vars
float f;
String temp;
int manualTemp;
int targetTemp;
bool heating = false;


// EEPROM state struct and handlers
// Struct for storing state in RAM and EEPROM
struct CCState {
    int magicNumber = 0xC10000; // this should change when struct changes

    // Target temp to maintain output at
    int targetTemp;

    // checkum value
    int checkSum = 0;
} cfgData;

//  EEPROM handing functions
#define CFG_BASE 0
bool readCfg()
{
  uint8_t *_data = (uint8_t*) &cfgData;
  //uint8_t* _data     = &cfgData;
  int      _chkSum   = 0;

  for (int i = 0; i < sizeof(cfgData); i++)
  {
    _data[i] = EEPROM.read(CFG_BASE + i);
    if (i < sizeof(cfgData) - sizeof(_chkSum))
      _chkSum += (_data[i] << i);
  }
  return (_chkSum == cfgData.checkSum);
}

int writeCfg()
{
  uint8_t *_data = (uint8_t*) &cfgData;
  //uint8_t* _data = &cfgData;
  cfgData.checkSum = 0;

  for (int i = 0; i < sizeof(cfgData); i++)
  {
    EEPROM.write(CFG_BASE + i, _data[i]);
    if (i < sizeof(cfgData) - sizeof(cfgData.checkSum))
      cfgData.checkSum += (_data[i] << i);
  }

  return cfgData.checkSum;
}

bool validCfg() {
    if (cfgData.targetTemp < MIN_TEMP)
        return false;
    if (cfgData.targetTemp > MAX_TEMP)
        return false;

    // everything looks good!  We can use these values
    return true;
}

int initCfg() {
    cfgData.targetTemp = DEFAULT_TARGETTEMP;
    writeCfg();
    Particle.publish("EEPROM", "New state initialized and written");
}

int setTargetTemp(String tempValue) {
    cfgData.targetTemp = tempValue.toInt();
    targetTemp = tempValue.toInt();
    overrideTimer = 0;
    writeCfg();
    return cfgData.targetTemp;
}

void setup() {
    // set BLE to high power
    BLE.setTxPower(8);
    Particle.publishVitals(3600);
    
    if (!sensor.begin()) {
        Particle.publish("Status", "Did not find Si7021 sensor!");
//        while (true)
//            ;
    }

    // retrieve values from EEPROM or init if needed
    if ( !readCfg() ) {
        Particle.publish("EEPROM", "State failed checksum from EEPROM");
        initCfg();
    } else
        Particle.publish("EEPROM", "State successfully loaded from EEPROM");
    if ( !validCfg() ) {
        Particle.publish("EEPROM", "State values are out of range");
        initCfg();
    } else
        Particle.publish("EEPROM", "State has valid values");

    Particle.function("setTargetTemp", setTargetTemp);
    Particle.variable("getTargetTemp", &targetTemp, INT);
    Particle.variable("getConfigTemp", &cfgData.targetTemp, INT);

    // Angle sensor
    //pinMode(ROTARY_ANGLE_SENSOR, INPUT);

    // init the display
    tm1637.init();
    tm1637.set(BRIGHT_TYPICAL);//BRIGHT_TYPICAL = 2,BRIGHT_DARKEST = 0,BRIGHTEST = 7;
    tm1637.point(POINT_OFF);
    
    // get initial telem
    myEnc.write(0);
    lastRotary = 0;
    getTelem();
//    lastManualTemp = manualTemp;
    overrideTimer = 0;

    // PID
    
    // PID routine
    if (temp.toInt() > 0)
        Input = (double)temp.toInt();

    myPID.SetOutputLimits(0, 1);
    myPID.SetMode(PID::AUTOMATIC);

    pinMode(D7, OUTPUT);
}

void getTelem() {

/*    
    // for pot on analog pins
    if (analogRead(ROTARY_ANGLE_SENSOR) <= lastRotary + ROTARY_DEBOUNCE && analogRead(ROTARY_ANGLE_SENSOR) >= lastRotary - ROTARY_DEBOUNCE)
        manualTemp = MIN_TEMP + int((MAX_TEMP + 1 - MIN_TEMP) * (4095-lastRotary)/4095);
    else {
        lastRotary = analogRead(ROTARY_ANGLE_SENSOR);
        manualTemp = MIN_TEMP + int((MAX_TEMP + 1 - MIN_TEMP) * (4095-analogRead(ROTARY_ANGLE_SENSOR))/4095);
    }
//    manualTemp = MIN_TEMP + int((MAX_TEMP + 1 - MIN_TEMP) * (4095-lastRotary)/4095);
*/

    // get temp
    f = sensor.readTemperature() * 1.8 + 32.0;
    temp = String(f);
    
    // verify we haven't wrapped millis(), if so reset overrideTimer to 1 to trigger below
    if (overrideTimer > 0 && millis() < overrideTimer - OVERRIDE_TIME) {
        Particle.publish("override", "millis() wrapped!  triggering override timer reset");
        overrideTimer = 1;
    }

    // reset encoder back to current setpoint if overrideTimer is over
    if (overrideTimer > 0 && overrideTimer < millis()) {
        overrideTimer = 0;
        myEnc.write(0);
        lastRotary = 0;
        manualTemp = cfgData.targetTemp;
    }

    // get rotary value, if changed
    //   make sure we're within the min and max, reset if not
    //   update timer

    if (myEnc.read() != lastRotary) {
        if (myEnc.read()/4 + cfgData.targetTemp < MIN_TEMP)
            myEnc.write((MIN_TEMP - cfgData.targetTemp) * 4);
        if (myEnc.read()/4 + cfgData.targetTemp > MAX_TEMP)
            myEnc.write((MAX_TEMP - cfgData.targetTemp) * 4);
        lastRotary = myEnc.read();
        manualTemp = lastRotary/4 + cfgData.targetTemp;
        overrideTimer = millis() + OVERRIDE_TIME;
    }
}

// decide if we need to be on or off
// decide if we are manual override, or back to homeKit set temp
void checkTemp() {
/*
    if (manualTemp != lastManualTemp) {
        lastManualTemp = manualTemp;
    }
*/

    if (overrideTimer != 0 && overrideTimer > millis()) {
        targetTemp = manualTemp;
        tm1637.set(BRIGHT_TYPICAL);//BRIGHT_TYPICAL = 2,BRIGHT_DARKEST = 0,BRIGHTEST = 7;
    } else {
        targetTemp = cfgData.targetTemp;
        tm1637.set(BRIGHT_DARKEST);//BRIGHT_TYPICAL = 2,BRIGHT_DARKEST = 0,BRIGHTEST = 7;
    }

    // decide if we need to heat or not
    if (temp.toInt() >= targetTemp + SWING_TEMP) {
        heating = false;
        digitalWrite(D7, LOW);
        tm1637.point(POINT_OFF);
    } else if (temp.toInt() <= targetTemp - SWING_TEMP) {
        heating = true;
        digitalWrite(D7, HIGH);
        tm1637.point(POINT_ON);
    }

   // PID routine
    if (f > 0)
        Input = (double)f;
    myPID.Compute();
}

void publishTelem() {
    if (lastReport + 5000 < millis()) {
//        Particle.publish("Humidity", String(sensor.readHumidity()));
        Particle.publish("Temperature", temp);
        Particle.publish("TargetTemp", String(targetTemp));
        if (heating) {
            Particle.publish("Heating", "On");
            Mesh.publish("Heating", "On");
        } else {
            Particle.publish("Heating", "Off");
            Mesh.publish("Heating", "Off");
        }

        Particle.publish("PID", String(Output));

        lastReport = millis();
    }
    
    tm1637.display(2,String(String(targetTemp).charAt(0)).toInt());
    tm1637.display(3,String(String(targetTemp).charAt(1)).toInt());
    tm1637.display(0,String(temp.charAt(0)).toInt());
    tm1637.display(1,String(temp.charAt(1)).toInt());
}

void loop() {

    // retrieve telementry
    getTelem();

    // check temp, adjust servo
    checkTemp();

    // publish telementry
    publishTelem();

}

