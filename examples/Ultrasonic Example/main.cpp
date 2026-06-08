// The following code will print out the distance read from the ultrasonic sensor
// The neopixel colour will change from green to red as the ultrasonic sensors detects distances from 0 - 40cm
// The motor speed will also be proportionally set to 0 - 100 depending on the distance


#include <Arduino.h>
#include <TerraDrive.h>
#include <Adafruit_NeoPixel.h>

TerraDrive m_terraDrive{};

// Pin Definitions
const int TRIGGER_PIN = 39;
const int ECHO_PIN = 48;

// Timing Variables for Non-blocking Superloop
unsigned long lastMeasurementTime = 0;
const unsigned long MEASURE_INTERVAL_MS = 10; // 500ms between reads
const unsigned long MAX_TIMEOUT_US = 30000;    // 30ms sensor timeout

// Function prototype
float getDistanceCM();

void setup() {
  // Initialize Serial communication
  Serial.begin(115200);
  // while (!Serial) {
  //   ; // Wait for serial port to connect (needed for native USB boards)
  // }
  
    // Initialise Terradrive board
  m_terraDrive.init(); // always make sure to init the board before everything else
  m_terraDrive.setEnableMotors(true);

  Serial.println("Initializing Ultrasonic Sensor...");

  // Configure Pin Modes. Note you have to us the terraDrive pinMode5V to setup pinMode for 5v circuits.
  m_terraDrive.pinMode5V(Pins5v::PIN39, OUTPUT); // Trigger pin
  m_terraDrive.pinMode5V(Pins5v::PIN48, INPUT); // Echo pin

  // Ensure trigger starts LOW
  digitalWrite(TRIGGER_PIN, LOW);
  delay(50); 
  



  Serial.println("Initialization complete.");
}

void loop() {
  unsigned long currentTime = millis();

  // Check if 500ms has passed since the last sensor reading
  if (currentTime - lastMeasurementTime >= MEASURE_INTERVAL_MS) {
    
    float distance = getDistanceCM();

    if (distance < 0) {
      Serial.println("Error: Target out of range or sensor timeout.");
    } else {
      Serial.print("Distance: ");
      Serial.print(distance);
      Serial.println(" cm");
    }

    float speed = distance * 100.0f / 40.0f;

    m_terraDrive.setLeftMotor(speed); // sets the motor speed to run faster the closer it is to 20cm
    m_terraDrive.setRightMotor(speed);

    int redValue = map(speed, 0, 100, 0, 255);
    int greenValue = map(speed, 0, 100, 255, 0);
    int blueValue = 0; // Keeping blue at 0

    Adafruit_NeoPixel& pixels = m_terraDrive.getNeoPixel();

    pixels.setPixelColor(0, pixels.Color(redValue, greenValue, blueValue));
    pixels.setPixelColor(1, pixels.Color(redValue, greenValue, blueValue));

    pixels.show();

    // Update the timestamp for the next interval
    lastMeasurementTime = currentTime;
  }

  // --- YOU CAN ADD OTHER NON-BLOCKING SUPERLOOP CODE HERE ---
  // e.g., digitalRead(buttonPin), flashing an LED, etc.
}

float getDistanceCM() {
  // 1. Send a 10-microsecond HIGH pulse to Trigger
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);

  // 2. Measure how long Echo pin stays HIGH (with a 30ms timeout)
  // pulseIn returns duration in microseconds
  unsigned long duration_us = pulseIn(ECHO_PIN, HIGH, MAX_TIMEOUT_US);

  // 3. If pulseIn times out, it returns -1
  if (duration_us == 0) {
    return -1.0f; 
  }

  // 4. Calculate distance in cm
  // (Speed of sound is 0.0343 cm/us. Divided by 2 for the round trip)
  float distance_cm = (duration_us * 0.0343f) / 2.0f;

  return distance_cm;
}