/* An exmaple of using alternate pins for SDA and SCL */

#include "Adafruit_Sensor.h"
#include "Adafruit_AM2320.h"

Adafruit_AM2320 am2320 = Adafruit_AM2320();

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10); // hang out until serial port opens
  }

  Serial.println("Adafruit AM2320 Basic Test");
  am2320.begin(D6,D5);  /* alternate SDA and SCL pins are declared here */
}

void loop() {
  Serial.print("Temp: "); Serial.println(am2320.readTemperature());
  Serial.print("Hum: "); Serial.println(am2320.readHumidity());

  delay(2000);
}
