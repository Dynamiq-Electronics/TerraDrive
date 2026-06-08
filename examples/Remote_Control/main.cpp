#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

// ── Change this to your robot ESP32's MAC address ──
uint8_t robotMac[] = {0x10, 0x20, 0xBA, 0x74, 0xCA, 0x2C};

struct Packet {
    float left;
    float right;
};

void onSent(const esp_now_send_info_t *info, esp_now_send_status_t status) {
    // optional debug
}

void setup() {
    Serial.begin(115200);
    WiFi.mode(WIFI_STA);
    esp_now_init();
    esp_now_register_send_cb(onSent);

    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, robotMac, 6);
    peer.channel = 0;
    peer.encrypt = false;
    esp_now_add_peer(&peer);

    Serial.println("Bridge ready");
}

void loop() {
    if (Serial.available()) {
        String line = Serial.readStringUntil('\n');
        line.trim();

        int comma = line.indexOf(',');
        if (comma == -1) return;

        Packet pkt;
        pkt.left  = line.substring(0, comma).toFloat();
        pkt.right = line.substring(comma + 1).toFloat();

        esp_now_send(robotMac, (uint8_t*)&pkt, sizeof(pkt));
    }
}

// #include <Arduino.h>
// #include <esp_now.h>
// #include <WiFi.h>
// #include <TerraDrive.h>

// TerraDrive m_terraDrive{};

// struct Packet {
//     float left;
//     float right;
// };

// void onReceive(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
//     if (len != sizeof(Packet)) return;
//     Packet pkt;
//     memcpy(&pkt, data, sizeof(pkt));
//     m_terraDrive.setLeftMotor(pkt.left);
//     m_terraDrive.setRightMotor(pkt.right);
// }

// void setup() {
//     Serial.begin(115200);

//     m_terraDrive.init();
//     m_terraDrive.setEnableMotors(true);

//     WiFi.mode(WIFI_STA);
//     delay(1000);
//     // Print MAC so you can copy it into the bridge firmware
//     Serial.print("Robot MAC: ");
//     Serial.println(WiFi.macAddress());

//     esp_now_init();
//     esp_now_register_recv_cb(onReceive);
// }

// void loop(){}
