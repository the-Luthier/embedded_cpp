#include <LidarLite_v4.h>

void setup() {
  // Initialize the serial port.
  Serial.begin(9600);

  // Initialize the Lidar sensor.
  lidar.begin();
}

void loop() {
  // Read the distance from the Lidar sensor.
  int distance = lidar.distance();

  // Print the distance to the console.
  Serial.println(distance);

  // If the distance is less than 100 cm, turn on the LED.
  if (distance < 100) {
    digitalWrite(LED_BUILTIN, HIGH);
  } else {
    digitalWrite(LED_BUILTIN, LOW);
  }

  // Delay for 1 second.
  delay(1000);
}
