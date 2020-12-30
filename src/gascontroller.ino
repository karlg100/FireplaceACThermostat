
#define RELAY D4

bool heating = false;

// maximum time we will run without a mesh update from the thermostat
size_t lastUpdate = 0;
#define MAX_RUN_NO_UPDATE 60000

void myHandler(const char *event, const char *data)
{
    if (String(data) == String("On")) {
        heating = true;
        //Particle.publish("Status", "Heating");
    } else {
        heating = false;
        //Particle.publish("Status", "Not Heating");
     }

    //Particle.publish("data", data);
    Particle.publish("Relay", String(heating));
    lastUpdate = millis();
}

void setup()
{
    Particle.publishVitals(300);
    pinMode(RELAY, OUTPUT);
    pinMode(D7, OUTPUT);
    pinMode(PWR, INPUT);
    Particle.subscribe("LivingroomHeating", myHandler);
    //Mesh.subscribe("Heating", myHandler);
    //BLE.setTxPower(8);
}

void loop()
{
    if (heating == true && lastUpdate + MAX_RUN_NO_UPDATE > millis()) {
    //if (heating == true) {
        digitalWrite(RELAY, HIGH);
        digitalWrite(D7, HIGH);
    } else {
        digitalWrite(RELAY, LOW);
        digitalWrite(D7, LOW);
    }
    
    // if we are on battery, let's conserve.
    // we got an update recently, sleep for 1/4th the max time
    if (!digitalRead(PWR) && lastUpdate + (MAX_RUN_NO_UPDATE/4) > millis()) {
        //System.sleep(D1, RISING, MAX_RUN_NO_UPDATE/2);
        System.sleep(D1, RISING, 10);
    }

}

