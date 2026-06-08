#include <Arduino.h>
#include <TerraDrive.h>

TerraDrive m_terraDrive{};

void setup() {
  Serial.begin(115200);
  m_terraDrive.init();
  m_terraDrive.setEnableMotors(true);
  Serial.println("Enter speed (-100 to 100):");
}

float filtered{0.0};

void loop() {

  filtered += 0.001 * (m_terraDrive.getLeftCurrentRaw() - filtered);

  Serial.print(">Left Current:");
  Serial.println(filtered);
  // Serial.print(">Right Current:");
  // Serial.println(m_terraDrive.getRightCurrentRaw());

  if (Serial.available()) {
    int speed = Serial.parseInt();
    Serial.flush();
    m_terraDrive.setLeftMotor(speed);
    m_terraDrive.setRightMotor(speed);
    Serial.printf("Speed set to %d\n", speed);
  }
}
