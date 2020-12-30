#include <Particle.h>

// Support for Rotary Encoder input
#include <Encoder.h>

// Support for I2C
//include <Wire.h>

// Support for Temp/Humidity sensor
#include <Adafruit_Si7021.h>

// support for PID controller
#include <pid.h>

// OLED output support
#include <SPI.h>
#include <Adafruit_GFX_RK.h>
#include <Adafruit_SSD1306_RK.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET    D6 // Reset pin # (or -1 if sharing Arduino reset pin)

#define LOGO_HEIGHT   16
#define LOGO_WIDTH    16

#define REFRESH_DELAY 300

static const unsigned char PROGMEM flame1_bmp[] = {
    B00000001, B00000000,
    B00000011, B10000000,
    B00000001, B10000000,
    B00000001, B11000000,
    B00000111, B11000000,
    B00001111, B11000110,
    B00001111, B11101100,
    B00001011, B11111100,
    B01011010, B01111100,
    B01110010, B00110110,
    B01100000, B00100100,
    B00100000, B01100100,
    B00110100, B00001100,
    B00010110, B10001000,
    B00011011, B00001000,
    B00001111, B11110000};

static const unsigned char PROGMEM flame2_bmp[] =
    {B00001000, B00000000,
     B00000110, B00000000,
     B00000011, B00000000,
     B00000011, B10100010,
     B00000001, B11000001,
     B00000111, B11000110,
     B01001111, B11101100,
     B10001011, B11111100,
     B01011010, B01111101,
     B01110001, B00110110,
     B01100011, B00100100,
     B00100000, B01100100,
     B00110100, B01001100,
     B00010110, B00101000,
     B00011011, B10011000,
     B00001111, B11110000};

static const unsigned char PROGMEM flame3_bmp[] =
    {B00000000, B00000000,
     B00000110, B00000000,
     B00011011, B01100010,
     B00000011, B10010100,
     B00000001, B11001000,
     B01000011, B11001100,
     B00101111, B11111100,
     B01001011, B11111100,
     B01011011, B01111100,
     B01110001, B00110110,
     B00111000, B00100101,
     B00111000, B00101100,
     B00011001, B00001000,
     B00010010, B00001000,
     B00011011, B10011000,
     B00001111, B11110000};

static const unsigned char PROGMEM flame4_bmp[] =
    {B00000001, B10000000,
     B00000010, B00010000,
     B00000011, B00101000,
     B00100011, B10001000,
     B00010001, B11000100,
     B00100011, B11001100,
     B00010111, B11111100,
     B00011111, B11111100,
     B00111011, B01111100,
     B00110001, B00110100,
     B00111111, B00100100,
     B00111001, B00101100,
     B00011010, B10011100,
     B00010010, B01001100,
     B00011011, B00111000,
     B00000111, B11110000};

/*
static const unsigned char PROGMEM flame1_bmp[] =
{ B00010110, B00000000,
  B01001100, B10000000,
  B00100100, B00000000,
  B00101110, B01000000,
  B00101110, B11000000,
  B00111111, B10000000,
  B10111011, B10000000,
  B10110111, B10000000,
  B10110101, B11000000,
  B11110110, B11000000,
  B11100110, B11000000,
  B11000010, B11000000,
  B11000000, B11000000,
  B11101001, B11000000,
  B01111111, B10000000,
  B00111111, B00000000 };

static const unsigned char PROGMEM flame2_bmp[] =
{ B01000100, B10000000,
  B00101100, B00000000,
  B00010010, B00000000,
  B00110110, B01000000,
  B10011110, B11000000,
  B10111111, B10000000,
  B00111011, B10000000,
  B00110101, B10000000,
  B10110001, B11000000,
  B11110011, B11000000,
  B11101000, B11000000,
  B11011000, B11000000,
  B11001000, B11000000,
  B11100001, B11000000,
  B01111111, B10000000,
  B00111111, B00000000 };
*/

size_t lastOLEDRefresh = 0;

Adafruit_Si7021 sensor(&Wire1);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// temp limits
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

// time tracking vars
size_t lastReport = 0;
size_t overrideTimer = 0;

// manual override rotary encoder
// Change these two numbers to the pins connected to your encoder.
// Both pins must have interrupt capability
Encoder myEnc(D4, D5);
//   avoid using pins with LEDs attached


long lastRotary = 0;
//define OVERRIDE_TIME 10000
#define OVERRIDE_TIME 3600000

// working vars
float f;
String temp;
int manualTemp;
int targetTemp;
bool heating = false;

// Modes
// 0 - Off
#define MODE_OFF 0
#define MODE_OFF_TXT "Off"
// 1 - Heating
#define MODE_HEAT 1
#define MODE_HEAT_TXT "Heat"
// 2 - Cool (not yet supported)
#define MODE_COOL 2
#define MODE_COOL_TXT "Cool"
// 3 - Auto
#define MODE_AUTO 3
#define MODE_AUTO_TXT "Auto"
#define DEFAULT_MODE 0
#define MODE_MIN 0
#define MODE_MAX 1

// EEPROM state struct and handlers
// Struct for storing state in RAM and EEPROM
struct CCState {
    int magicNumber = 0xC10000; // this should change when struct changes

    int mode;

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

  for (unsigned int i = 0; i < sizeof(cfgData); i++)
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

  for (unsigned int i = 0; i < sizeof(cfgData); i++)
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

    if (cfgData.mode < MODE_MIN)
        return false;
    if (cfgData.mode > MODE_MAX)
        return false;

    // everything looks good!  We can use these values
    return true;
}

void initCfg() {
    cfgData.targetTemp = DEFAULT_TARGETTEMP;
    cfgData.targetTemp = DEFAULT_MODE;
    writeCfg();
    Particle.publish("EEPROM", String("New state initialized and written"), PRIVATE);
}

int setTargetTemp(String tempValue) {
    if (tempValue.toInt() >= MIN_TEMP && tempValue.toInt() <= MAX_TEMP) {
        cfgData.targetTemp = tempValue.toInt();
        targetTemp = tempValue.toInt();
        overrideTimer = 0;
        writeCfg();
        return cfgData.targetTemp;
    } else
        return false;
}

int setMode(String modeValue) {
    if (modeValue.toInt() >= MODE_MIN && modeValue.toInt() <= MODE_MAX) {
        cfgData.mode = modeValue.toInt();
        overrideTimer = 0;
        writeCfg();
        return cfgData.mode;
    } else
        return false;
}

// OLED functions
void displayString(String msg) {
    // Clear the buffer
    display.clearDisplay();

    display.setTextSize(2);             // Normal 1:1 pixel scale
    display.setTextColor(WHITE);        // Draw white text
    display.setCursor(20,0);             // Start at top-left corner
    display.println(msg);
}

void printTemp(int f) {
  display.setTextSize(2);             // Normal 1:1 pixel scale
  display.setTextColor(WHITE);        // Draw white text
  display.setCursor(20,0);             // Start at top-left corner
  display.println(String(f)+(char)247+"F/");
}

void printClearTemp() {
  display.setTextSize(2);             // Normal 1:1 pixel scale
  display.setTextColor(WHITE);        // Draw white text
  display.setCursor(20,0);             // Start at top-left corner
  display.println(String("    "));
}

void printSetPt(int f) {
  display.setTextSize(2);             // Normal 1:1 pixel scale
  display.setTextColor(WHITE);        // Draw white text
  display.setCursor(80,0);             // Start at top-left corner
  display.println(String(f)+(char)247+"F");
}

void printMode(String mode) {
  display.setTextSize(2);             // Normal 1:1 pixel scale
  if (heating)
    display.setTextColor(BLACK);        // Draw white text
  else
    display.setTextColor(WHITE);        // Draw white text
  //display.setTextColor(BLACK, WHITE);        // Draw white text
  display.setCursor(44,18);             // Start at top-left corner
  display.println(mode);
}

#define FLAMES 8
int flame[FLAMES];

void drawFire() {
  for (int x=0; x<FLAMES; x++) {
      if (flame[x] < 1 || flame[x] > 4)
        flame[x] = random(1,5);
      if (flame[x] == 1)
        display.drawBitmap(
          x*LOGO_WIDTH,
          (display.height() - LOGO_HEIGHT),
          flame1_bmp, LOGO_WIDTH, LOGO_HEIGHT, 1);
      else if (flame[x] == 2)
        display.drawBitmap(
          x*LOGO_WIDTH,
          (display.height() - LOGO_HEIGHT),
          flame2_bmp, LOGO_WIDTH, LOGO_HEIGHT, 1);
      else if (flame[x] == 3)
        display.drawBitmap(
          x*LOGO_WIDTH,
          (display.height() - LOGO_HEIGHT),
          flame3_bmp, LOGO_WIDTH, LOGO_HEIGHT, 1);
      else if (flame[x] == 4)
        display.drawBitmap(
          x*LOGO_WIDTH,
          (display.height() - LOGO_HEIGHT),
          flame4_bmp, LOGO_WIDTH, LOGO_HEIGHT, 1);
        flame[x]++;
      if (flame[x] > 4)
        flame[x] = 1;
  }
}

void setup() {
    if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x32
      Serial.println(F("SSD1306 allocation failed"));
      Particle.publish("Status", String("Did not find SSD1306 display!"), PRIVATE);
      for(;;); // Don't proceed, loop forever
    }

    displayString("Booting...");

    // set BLE to high power
    BLE.setTxPower(8);
    Particle.publishVitals(3600);

    if (!sensor.begin()) {
        Particle.publish("Status", String("Did not find Si7021 sensor!"), PRIVATE);
//        while (true)
//            ;
    }

    // retrieve values from EEPROM or init if needed
    if ( !readCfg() ) {
        Particle.publish("EEPROM", String("State failed checksum from EEPROM"), PRIVATE);
        initCfg();
    } else {
        Particle.publish("EEPROM", String("State successfully loaded from EEPROM"), PRIVATE);
    }
    if ( !validCfg() ) {
        Particle.publish("EEPROM", String("State values are out of range"), PRIVATE);
        initCfg();
    } else
        Particle.publish("EEPROM", String("State has valid values"), PRIVATE);

    Particle.function("setTargetTemp", setTargetTemp);
    Particle.function("setMode", setMode);
    Particle.variable("getTargetTemp", &targetTemp, INT);
    Particle.variable("getConfigTemp", &cfgData.targetTemp, INT);

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
    // get temp
    f = sensor.readTemperature() * 1.8 + 32.0;
    temp = String(f);
    
    // verify we haven't wrapped millis(), if so reset overrideTimer to 1 to trigger below
    if (overrideTimer > 0 && millis() < overrideTimer - OVERRIDE_TIME) {
        Particle.publish("override", String("millis() wrapped!  triggering override timer reset"), PRIVATE);
        overrideTimer = 1;
    }

    // reset encoder back to current setpoint if overrideTimer is over
    if (overrideTimer > 0 && overrideTimer < millis()) {
        overrideTimer = 0;
        lastOLEDRefresh = 0;
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

// decide if Heating need to be on or off
// decide if we are manual override, or back to particle varilabe temp setpoint
void checkTemp() {
    if (overrideTimer != 0 && overrideTimer > millis()) {
        targetTemp = manualTemp;
//        tm1637.set(BRIGHT_TYPICAL);//BRIGHT_TYPICAL = 2,BRIGHT_DARKEST = 0,BRIGHTEST = 7;
    } else {
        targetTemp = cfgData.targetTemp;
//        tm1637.set(BRIGHT_DARKEST);//BRIGHT_TYPICAL = 2,BRIGHT_DARKEST = 0,BRIGHTEST = 7;
    }

    // decide if we need to heat or not
    if (cfgData.mode == MODE_HEAT || cfgData.mode == MODE_AUTO) {
        if (temp.toInt() >= targetTemp + SWING_TEMP) {
            heating = false;
            digitalWrite(D7, LOW);
        } else if (temp.toInt() <= targetTemp - SWING_TEMP) {
            heating = true;
            digitalWrite(D7, HIGH);
        }
    }

    // if we're off or cooling, stop heating
    if (cfgData.mode == MODE_OFF || cfgData.mode == MODE_COOL) {
        heating = false;
        digitalWrite(D7, LOW);
    }

   // PID routine
    if (f > 0)
        Input = (double)f;
    myPID.Compute();
}

int updateSet = 0;
void publishTelem() {
    if (lastReport + 5000 < millis()) {
        if (heating) {
            Particle.publish("LivingroomHeating", String("On"), PRIVATE);
            //Mesh.publish("Heating", String("On"));
        } else {
            Particle.publish("LivingroomHeating", String("Off"), PRIVATE);
            //Mesh.publish("Heating", String("Off"));
        }
        Particle.publish("TargetTemp", String(targetTemp), PRIVATE);
        Particle.publish("Mode", String(cfgData.mode), PRIVATE);
        if (updateSet == 0) {
            Particle.publish("Humidity", String(sensor.readHumidity()), PRIVATE);
            updateSet = 1;
        } else {
            // noting
            updateSet = 0;
            Particle.publish("Temperature", String(temp), PRIVATE);
        }

        //Particle.publish("PID", String(Output), PRIVATE);

        lastReport = millis();
    }

    if (lastOLEDRefresh + REFRESH_DELAY < millis())
    {
        display.clearDisplay();
        if (heating)
            drawFire();
        printTemp(temp.toInt());
        printSetPt(targetTemp);
        if (cfgData.mode == MODE_OFF) {
            printMode(MODE_OFF_TXT);
            printClearTemp();
        }
        if (cfgData.mode == MODE_HEAT) {
            printMode(MODE_HEAT_TXT);
            printSetPt(targetTemp);
        }
        if (cfgData.mode == MODE_COOL) {
            printMode(MODE_COOL_TXT);
            printSetPt(targetTemp);
        }
        if (cfgData.mode == MODE_AUTO) {
            printMode(MODE_AUTO_TXT);
            printSetPt(targetTemp);
        }
        display.display();
        lastOLEDRefresh = millis();
    }
}

void loop() {

    // retrieve telementry
    getTelem();

    // check temp, adjust servo
    checkTemp();

    // publish telementry
    publishTelem();

}

