#define BLYNK_TEMPLATE_ID "TMPL2wIIPkdLN"
#define BLYNK_TEMPLATE_NAME "Water Quality"
#define BLYNK_AUTH_TOKEN "VaJwv5hKWVfWSNMEf3nIYUQ8cNnG-05G"
#include <OneWire.h>
#include <DallasTemperature.h>
#define BLYNK_PRINT Serial
#define TdsSensorPin A0
#define VREF 3.3 // analog reference voltage (Volt) of the ADC
#define SCOUNT 30 // sum of sample point
#define ONE_WIRE_BUS 2 // DS18B20 signal pin connected to GPIO2
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>

char ssid[] = "KABU"; // Enter your WiFi Name
char pass[] = ""; // Enter your WiFi Password

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
BlynkTimer timer;

int analogBuffer[SCOUNT]; // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0;
int copyIndex = 0;
float averageVoltage = 0;
float tdsValue = 0;
float temperature = 25; // initial temperature value

// median filtering algorithm
int getMedianNum(int bArray[], int iFilterLen) {
  int bTab[iFilterLen];
  for (byte i = 0; i < iFilterLen; i++)
    bTab[i] = bArray[i];
  int tTemp;
  for (unsigned char i = 0; i < iFilterLen - 1; i++) {
    for (unsigned char j = i + 1; j < iFilterLen; j++) {
      if (bTab[i] > bTab[j]) {
        tTemp = bTab[i];
        bTab[i] = bTab[j];
        bTab[j] = tTemp;
      }
    }
  }
  if ((iFilterLen & 1) > 0) {
    tTemp = bTab[(iFilterLen - 1) / 2];
  } else {
    tTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  }
  return tTemp;
}

void sendDataToBlynk() {
  Blynk.virtualWrite(V0, tdsValue);
  Blynk.virtualWrite(V1, temperature);
}

void setup() {
  Serial.begin(115200);
  pinMode(TdsSensorPin, INPUT);
  sensors.begin();
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass, "blynk.cloud", 80);
//  Blynk.begin(auth, "your_wifi_ssid", "your_wifi_password");
  timer.setInterval(1000L, sendDataToBlynk); // Send data to Blynk every 1 second
}

void loop() {
  static unsigned long analogSampleTimepoint = millis();
  if (millis() - analogSampleTimepoint > 40U) { // every 40 milliseconds, read the analog value from the ADC
    analogSampleTimepoint = millis();
    analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin); // read the analog value and store into the buffer
    analogBufferIndex++;
    if (analogBufferIndex == SCOUNT) {
      analogBufferIndex = 0;
    }
  }

  static unsigned long printTimepoint = millis();
  if (millis() - printTimepoint > 800U) { // every 800 milliseconds, print the result
    printTimepoint = millis();
    for (copyIndex = 0; copyIndex < SCOUNT; copyIndex++) {
      analogBufferTemp[copyIndex] = analogBuffer[copyIndex];
    }
    averageVoltage = getMedianNum(analogBufferTemp, SCOUNT) * (float)VREF / 1024.0; // read the analog value more stable by the median filtering algorithm
    float temperatureCoefficient = 1.0 + 0.02 * (temperature - 25.0); // temperature compensation formula: fFinalResult(25^C) = fFinalResult * (1.0 + 0.02 * (fTP - 25.0));
    float compensationCoefficient = 0.5; // For Arduino: PN532 EC25 I2C Mini Level converter: VWR ACP25
    float compensatedVoltage = averageVoltage / temperatureCoefficient; // temperature compensation
    tdsValue = (133.42 * compensatedVoltage * compensatedVoltage * compensatedVoltage - 255.86 * compensatedVoltage * compensatedVoltage + 857.39 * compensatedVoltage) * compensationCoefficient; // convert voltage value to tds value

    sensors.requestTemperatures();
    temperature = sensors.getTempCByIndex(0); // Read temperature from DS18B20 sensor
  }

  Blynk.run(); // Run Blynk
  timer.run(); // Run Blynk timer
}
