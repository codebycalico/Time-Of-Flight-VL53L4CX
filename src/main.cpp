// This code was developed from the example code obtained via the Arduino IDE for the
// Grove Time-of-Flight Distance Sensor VL53L0X.
// Modified by Calico for OMSI "How Fast"
// January 20, 2025
 
#include <Wire.h>
#include <Adafruit_VL53L0X.h>  

#define TOTAL_TOFS 2

#define X_SHUT_PIN1 7
#define X_SHUT_PIN2 8

#define TOFS1_ADDR 0x30
#define TOFS2_ADDR 0x31

Adafruit_VL53L0X TOFS1;
Adafruit_VL53L0X TOFS2;

// Create array-variable for the sensors' range (in mm):
uint16_t ranges_mm[TOTAL_TOFS];

//====================================================================
// Define global variables for the sensors:
//====================================================================
typedef struct {
  Adafruit_VL53L0X *psensor; // pointer to object
  TwoWire *pwire;
  int addr;            // IIC addr number for the sensor 
  int shutdown_pin;  // which pin for shutdown;
  Adafruit_VL53L0X::VL53L0X_Sense_config_t sensor_config;     // options for how to use the sensor
  uint16_t range;        // range value used in continuous mode stuff.
  uint8_t sensor_status; // status from last ranging in continuous.
} sensorList_t;

sensorList_t sensors[] = {
  {&TOFS1, &Wire, TOFS1_ADDR, X_SHUT_PIN1, Adafruit_VL53L0X::VL53L0X_SENSE_DEFAULT, 0, 0},
  {&TOFS2, &Wire, TOFS2_ADDR, X_SHUT_PIN2, Adafruit_VL53L0X::VL53L0X_SENSE_DEFAULT, 0, 0}
};

/*
    Reset all sensors by setting all of their XSHUT pins LOW for delay(10), then
    set all XSHUT HIGH to bring out of reset. Keep sensor #1 awake by keeping XSHUT
    pin HIGH. Put all other sensors into shutdown by pulling XSHUT pins LOW.
    Initialize sensor #1 with sensor.begin(new_i2c_address). Pick any number except
    0x29 and it must be under 0x7F. Going with 0x30 to 0x3F is probably OK.
    Keep sensor #1 awake, and now bring sensor #2 out of reset by setting its
    XSHUT pin HIGH. Initialize sensor #2 with sensor.begin(new_i2c_address).
*/
void tofs_init() {
  bool found_any_sensors = false;
  // Set all shutdown pins low to shutdown sensors
  for (int i = 0; i < TOTAL_TOFS; i++) {
    digitalWrite(sensors[i].shutdown_pin, LOW);
  }
  delay(10);

  for (int i = 0; i < TOTAL_TOFS; i++) {
    // one by one enable sensors and set their I2C address
    digitalWrite(sensors[i].shutdown_pin, HIGH);
    delay(10);

    if (sensors[i].psensor->begin(sensors[i].addr, false, sensors[i].pwire, sensors[i].sensor_config)) {
      found_any_sensors = true;
    } else {
      Serial.print(i, DEC);
      Serial.println(": failed to begin.");
    }
  }

  if (!found_any_sensors) {
    Serial.println("No valid sensors found.");
  }
}

void setup() {
  Serial.begin(9600);
  // Start the IIC bus:
  Wire.begin();

  while(!Serial);

  // Initialize all of the pins.
  Serial.println("Initializing TOFS...");
  for (int i = 0; i < TOTAL_TOFS; i++) {
    pinMode(sensors[i].shutdown_pin, OUTPUT);
    digitalWrite(sensors[i].shutdown_pin, LOW);
  }
  tofs_init();
  Serial.println("TOFS intialized.");
}

void loop() {
  // Detect within a specific range of either sensor
  // for (int i = 0; i < TOTAL_TOFS; i++) {
  //   if(sensors[i].psensor->readRange() <= 40) {
  //     digitalWrite(13, HIGH);
  //     Serial.print("Sensor ");
  //     Serial.print(i + 1);
  //     Serial.print(": ");
  //     Serial.print(sensors[i].psensor->readRange());
  //     Serial.println("mm");
  //   } else {
  //     digitalWrite(13, LOW);
  //   }
  // }
  
  // Read the data from the sensors
  for (int i = 0; i < TOTAL_TOFS; i++) {
    // capture the sensor's data
    ranges_mm[i] = sensors[i].psensor->readRange();
  }
  
  //Print out the distances to the serial monitor, again using a 'for' loop:
  // for (int i = 0; i < TOTAL_TOFS; i++) {
  //   Serial.print("Sensor ");
  //   Serial.print(i + 1);
  //   Serial.print(" : ");
  //   if (ranges_mm[i] == 8190) {
  //     Serial.print("Out of Bounds.");
  //   } else {
  //     Serial.print(ranges_mm[i], DEC);
  //     Serial.print("mm  ");
  //   }
  // }

  Serial.print("Sensor 1: ");
  if(ranges_mm[0] == 8190) {
    Serial.println("Out of Bounds.");
  } else {
    Serial.print(ranges_mm[0], DEC);
    Serial.println("mm");
  }

  // // Detect when within a specific range of either sensor
  // for (int i = 0; i < TOTAL_TOFS; i++) {
  //   if(ranges_mm[i] <= 40) {
  //     digitalWrite(13, HIGH);
  //     Serial.print("Sensor ");
  //     Serial.print(i + 1);
  //     Serial.print(": ");
  //     Serial.println(ranges_mm[i]);
  //   } else {
  //     digitalWrite(13, LOW);
  //   }
  // }
} 