
#include <Wire.h>
#include <Adafruit_INA219.h>

Adafruit_INA219 ina219;

// specify the pins
const int voltageInPin = A0;
const int voltageOutPin = A1;
// connect INA219 SDA to Arduino Nano pin A4
// connect INA219 SCL to Arduino Nano pin A5

const int buzzerPin = 5; // D5

byte thresholdAge;
char buffer[256]; // sprintf buffer
float gndRatio, gndOffset, battRatio;

void setup() {\
  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, LOW);

  Serial.begin(115200);
  Serial.println("initialize INA219");
  ina219.begin();
  
  thresholdAge = 0;
  Serial.println("setup done");
}

void loop() {

  /* the voltageIn (and voltageOut) increases about 3% when the load increases from 0 to 60 Watt */
//  float voltageIn = 19.96/520.0 * analogRead(voltageInPin); // 520 = 19.96V
//  float voltageOut1 = 11.84/418.0 * analogRead(voltageOutPin); // 418 = 11.84V
//  float voltageIn = 20.16/495.0 * analogRead(voltageInPin); // 495 = 20.16V
//  float voltageOut1 = 12.04/406.0 * analogRead(voltageOutPin); // 406 = 12.04V
//  float voltageIn = 19.87/495.0 * analogRead(voltageInPin); // 495 = 19.87
//  float voltageOut1 = 11.78/403 * analogRead(voltageOutPin); // 403 = 11.78
  float voltageIn = 20.01/499.0 * analogRead(voltageInPin); // 499 = 20.01V
  float voltageOut1 = 11.51/403.0 * analogRead(voltageOutPin); // 403 = 11.51V

  float inaVoltage = ina219.getBusVoltage_V();
  float inaShuntVoltage = ina219.getShuntVoltage_mV();
  float inaVoltageOut = inaVoltage - inaShuntVoltage/1000.0; // because the DC/DC compensates for the voltage drop accross shunt
  float inaCurrent = ( 9.0/9.1 ) * 10.0 * ina219.getCurrent_mA() / 1000.0; // 9.0/9.5 for calibration, 10.0 for shunt of 10mOhm instead of 100mOhm
  float power = inaVoltageOut * inaCurrent;

  if( voltageIn < 21.0 ) {
    if( thresholdAge < 255 ) {
      thresholdAge++;
    }
  } else {
    thresholdAge = 0;
  }

  if( thresholdAge > 2 ) {
    digitalWrite(buzzerPin, HIGH);
  } else {
    digitalWrite(buzzerPin, LOW);
  }

  sprintf(buffer, "ver 0003, VoltIn ");
  dtostrf(voltageIn, 2, 1, &buffer[strlen(buffer)]);
  sprintf(buffer, "%sV, VoltOut1 ", buffer);
  dtostrf(voltageOut1, 2, 1, &buffer[strlen(buffer)]);
  sprintf(buffer, "%sV, VoltOut2 ", buffer);
  dtostrf(inaVoltageOut, 2, 1, &buffer[strlen(buffer)]);
  sprintf(buffer, "%sV, CurrOut ", buffer);
  dtostrf(inaCurrent, 4, 1, &buffer[strlen(buffer)]);
  sprintf(buffer, "%sA, PwrOut ", buffer);
  dtostrf(power, 4, 1, &buffer[strlen(buffer)]);
  sprintf(buffer, "%sW, age %3d", buffer, thresholdAge);
  Serial.println(buffer);

  delay(1000);
}
