#include <FS.h>                     // FileSystem
#include <Wire.h>                   // I2C Connections
#include <Adafruit_Sensor.h>        // BME Sensor and SSD1306 Display
#include <Adafruit_BME280.h>        // BME Sensor
#include <HX711.h>                  // Loadcell
#include <U8x8lib.h>                // ssd1306 Display

#define VERBOSE false
//Hardware
#define LED 2
//Pin Settings
#define I2C_SDA D1                  // BME Sensor and SSD1306 Display
#define I2C_SCL D2                  // BME Sensor and SSD1306 Display
#define SCALE_SDA D3                // Loadcell
#define SCALE_SCL D4                // Loadcell
#define HEATERPIN D5                // Heater Relay
// Sensor Settings
#define BME280 0x76                 // BME Adress

// Serial Read
const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];
boolean newData = false;
	
// Scale, ME Sensor and SSD1306 Display
U8X8_SSD1306_128X64_NONAME_SW_I2C display(I2C_SCL, I2C_SDA, U8X8_PIN_NONE); 
Adafruit_BME280 bme;
HX711 scale;

//Predefined Vars
float weight = 10000;
float reference = 0;
float spoolweight = 100;
float factor;
float offset;
float tempfloat;
float coil = 100;
float tara = 200;
float density = 0;
float containersize = 10.6;           // in liter
float altitude = 59;                  // in metern
float heatertemp = 0;

//===============================================================
// This routines loads persistent data
//===============================================================

bool loadConfig() {
  File configFile = SPIFFS.open("/config.txt", "r");
  if (!configFile) {
    return false;
  }
  factor = configFile.readStringUntil('\n').toFloat();
  offset = configFile.readStringUntil('\n').toFloat();
  containersize = configFile.readStringUntil('\n').toFloat();
  altitude = configFile.readStringUntil('\n').toFloat();
  coil = configFile.readStringUntil('\n').toFloat();
  spoolweight = configFile.readStringUntil('\n').toFloat();
  density = configFile.readStringUntil('\n').toFloat();
  tara=coil+spoolweight;
  return true;
}

//===============================================================
// This routine saves persistent data
//===============================================================

bool saveConfig() {
  File configFile = SPIFFS.open("/config.txt", "w");
  if (!configFile) {
    return false;
  }
  configFile.println(factor);
  configFile.println(offset);
  configFile.println(containersize);
  configFile.println(altitude);
  configFile.println(coil);
  configFile.println(spoolweight);
  configFile.println(density);
  tara=coil+spoolweight;
  return true;
}

//===============================================================
// This routines to update Display
//===============================================================
void handleupdateDisplay() {
  float newweight = 0;

// Lineare Regression
  if (scale.is_ready()) {
    newweight = scale.get_units(1);
  }
  if (weight < newweight - 10 || weight > newweight + 10) {
      weight = weight*0.2 + newweight*0.8;  
  }
  else {
    if (weight > newweight) {
      weight = weight*0.8 + newweight*0.2;  
    }
  }  
  if (weight<tara) {
    weight=tara;
  }

 // Calc Sensor Values
  float temperature = bme.readTemperature();
  float humidity = bme.readHumidity();
  float pressure = bme.readPressure();
  pressure = bme.seaLevelForAltitude(altitude,pressure);
  pressure = pressure/100.0F;
  float water = (humidity * containersize * 13.24 * exp((17.62 * temperature) / (temperature + 243.12))) / (273.15 + temperature);
  float length=(weight-tara) / density / 2.41;

// Display and Print to USB
  String str1 = "Fila:         ";
  String serialtext = "[U:";
  serialtext += String(newweight,2);
  serialtext += ";W:";
  String str=String(weight-tara,2);
  str1.remove(7,str.length());
  str1 +=str;
  str1 += " g";
  serialtext +=str;
  serialtext += ";L:";
  display.drawString(0, 0, str1.c_str());
  str1 = "Length:       ";
  str=String(length,2);
  str1.remove(7,str.length());
  str1 +=str;
  str1 += " m";
  serialtext +=str;
  serialtext += ";T:";
  display.drawString(0, 2, str1.c_str());
  str1 = "Temp:         ";
  str=String(temperature,2);
  str1.remove(7,str.length());
  str1 +=str;
  str1 += " C";
  serialtext +=str;
  serialtext += ";R:";
  display.drawString(0, 3, str1.c_str());
  str1 = "Humi%:        ";
  str=String(humidity,2);
  str1.remove(7,str.length());
  str1 +=str;
  str1 += " %";
  serialtext +=str;
  serialtext += ";A:";
  display.drawString(0, 4, str1.c_str());
  str1 = "Water:        ";
  str=String(water,2);
  str1.remove(7,str.length());
  str1 +=str;
  str1 += "mg";
  display.drawString(0, 5, str1.c_str());
  serialtext +=str;
  serialtext += ";P:";
  serialtext += String(pressure,2);
  serialtext += "]";
  Serial.println(serialtext);

// Toggle Heater on/off
  if (heatertemp>0){
    if (heatertemp>temperature) {
        digitalWrite(HEATERPIN, LOW);
    }
    else {
        digitalWrite(HEATERPIN, HIGH);
    }
  }
}

//===============================================================
// This routines read from Serial Connection
//===============================================================
void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;
    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();
        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }
        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

// Process Read Data
void parseData() {      // split the data into its parts
    char * strtokIndx; // this is used by strtok() as an index
    String command = strtok(tempChars,":");      // get the first part - the string
    if (command == "tara"){
      scale.tare(10);
      offset = scale.get_offset();
      if (VERBOSE){
        Serial.println("Tara");
      }
    }
    else if(command == "cali") {
      strtokIndx = strtok(NULL, ";");
      reference = atof(strtokIndx);     // convert this part to a float
      scale.set_scale();
      tempfloat = scale.get_units(10);
      scale.set_scale(tempfloat/reference);
      factor = scale.get_scale();

      if (VERBOSE){
        Serial.printf("Reference weight set to %.2f\n", reference);
      }
    }
    else if(command == "coil") {
      strtokIndx = strtok(NULL, ";");
      coil = atof(strtokIndx);     // convert this part to a float
      if (VERBOSE){
        Serial.printf("Coilweight set to %.2f\n", tara);
      }
    }
    else if(command == "spow") {
      strtokIndx = strtok(NULL, ";");
      spoolweight = atof(strtokIndx);     // convert this part to a float
      if (VERBOSE){
        Serial.printf("Spoolweight set to %.2f\n", spoolweight);
      }
    }
    else if(command == "dens") {
      strtokIndx = strtok(NULL, ";");
      density = atof(strtokIndx);     // convert this part to a float
      if (VERBOSE){
        Serial.printf("Density set to %.2f\n", density);
      }
    }
    else if(command == "cont") {
      strtokIndx = strtok(NULL, ";");
      containersize = atof(strtokIndx);     // convert this part to a float
      if (VERBOSE){
        Serial.printf("Containersize set to %.2f\n", containersize);
      }
    } 
    else if(command == "alti") {
      strtokIndx = strtok(NULL, ";");
      altitude = atof(strtokIndx);     // convert this part to a float
      if (VERBOSE){
        Serial.printf("Altitude set to %.2f\n", altitude);
      }
    }
    else if(command == "heat") {
      strtokIndx = strtok(NULL, ";");
      heatertemp = atof(strtokIndx);     // convert this part to a float
      if (VERBOSE){
        Serial.printf("Heatertemp set to %.2f\n", heatertemp);
      }
    } 
    else if(command == "dele") {
      if (VERBOSE){
        Serial.printf("Restore Factorysettings");
      }
      strtokIndx = strtok(NULL, ";");    //delete all configfiles
      File configFile = SPIFFS.open("/config.txt", "w");
      configFile.close();
      ESP.restart();
    } 
    else if (VERBOSE){
      Serial.println("Unknown Command. Please Check Manual");
    }
    saveConfig();
}

//===============================================================
// This routines initialises Sensors
//===============================================================

void initSensor()
{
  display.drawString(1, 0, ".");
  scale.begin(SCALE_SDA, SCALE_SCL, 128);
  if (scale.wait_ready_timeout(1000)) {
    display.drawString(0, 3, "Scale found.");
    scale.set_scale(factor);
    scale.set_offset(offset);
  }
  else {
    display.drawString(0, 3, "Scale missing!");
    delay(1000);
  }
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);
  bool status = bme.begin(BME280);
  display.drawString(2, 0, ".");
  if (status) {
    display.drawString(0, 5, "Sensor found.");
  }
  else {
    display.drawString(0, 5, "Sensor missing!");
    delay(1000);
  }
}

//==============================================================
//                  SETUP
//==============================================================
void setup(void){
  Serial.begin(115200);
  display.begin();
  display.setFont(u8x8_font_chroma48medium8_r);
  display.drawString(0, 0, ".");
  if (SPIFFS.begin()) {
    if (loadConfig()) {
      display.drawString(0, 2, "Config loaded.");
    }
    else {
      display.drawString(0, 2, "Loadingerror!");
      delay(1000);
    }
  }
  else {
    display.drawString(0, 2, "Dataerror!");
    delay(1000);
  }
  initSensor();
  display.clear();
  pinMode(LED,OUTPUT); 
  pinMode(HEATERPIN, OUTPUT);
  digitalWrite(HEATERPIN,HIGH);
}
//==============================================================
//                     LOOP
//==============================================================
void loop(void){
  handleupdateDisplay();
  recvWithStartEndMarkers();
  if (newData == true) {
      strcpy(tempChars, receivedChars);
      parseData();
      newData = false;
  }
}
