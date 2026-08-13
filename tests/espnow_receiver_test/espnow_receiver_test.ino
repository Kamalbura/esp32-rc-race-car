#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_idf_version.h>

#define ESPNOW_CHANNEL 6
#define TEST_MAGIC 0x52434144UL  // "RCAD"
#define TEST_VERSION 1

// COM3 transmitter MAC from esptool read_mac: B4:3A:45:3F:46:BC.
#include "secrets.h"

// Must match the transmitter test sketch.


struct __attribute__((packed)) RawAdcPacket {
  uint32_t magic;
  uint8_t version;
  uint16_t sequence;
  uint32_t txUptimeMs;
  int16_t adcRaw[4];
  uint8_t encoderBits;
  uint16_t crc;
  
};

struct __attribute__((packed)) AckPacket {
  uint32_t magic;
  uint8_t version;
  uint16_t sequenceAck;
  uint32_t rxUptimeMs;
  uint32_t rxPacketCount;
  int16_t echoedAdcRaw[4];
  uint16_t crc;
};

uint8_t lastSender[6] = {0};
RawAdcPacket lastPacket = {};
portMUX_TYPE packetMux = portMUX_INITIALIZER_UNLOCKED;

volatile bool haveSender = false;
volatile bool newPacket = false;
volatile uint32_t packets = 0;
volatile uint32_t badPackets = 0;
volatile uint32_t ignoredPackets = 0;
volatile uint32_t missedPackets = 0;
volatile uint32_t lastPacketRxMs = 0;
volatile bool haveSequence = false;

uint16_t lastAckSequence = 0;
uint16_t lastReceivedSequence = 0;
uint32_t lastSerialMs = 0;

uint16_t crc16Ccitt(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  while (len--) {
    crc ^= (uint16_t)(*data++) << 8;
    for (uint8_t i = 0; i < 8; i++) {
      crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
  }
  return crc;
}

uint16_t rawPacketCrc(const RawAdcPacket &packet) {
  RawAdcPacket copy = packet;
  copy.crc = 0;
  return crc16Ccitt((const uint8_t *)&copy, sizeof(copy));
}

uint16_t ackPacketCrc(const AckPacket &packet) {
  AckPacket copy = packet;
  copy.crc = 0;
  return crc16Ccitt((const uint8_t *)&copy, sizeof(copy));
}

void printMac(const uint8_t *mac) {
  Serial.printf("%02X:%02X:%02X:%02X:%02X:%02X",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

bool sameMac(const uint8_t *a, const uint8_t *b) {
  return memcmp(a, b, 6) == 0;
}

bool addPeer(const uint8_t *mac) {
  if (esp_now_is_peer_exist(mac)) return true;

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, mac, 6);
  peer.channel = ESPNOW_CHANNEL;
  peer.encrypt = true;
  peer.ifidx = WIFI_IF_STA;
  memcpy(peer.lmk, ESPNOW_LMK, sizeof(ESPNOW_LMK));
  esp_err_t result = esp_now_add_peer(&peer);
  return result == ESP_OK || result == ESP_ERR_ESPNOW_EXIST;
}

void sendAck(const uint8_t *mac, const RawAdcPacket &packet) {
  if (!addPeer(mac)) {
    Serial.println("Failed to add transmitter peer for ACK.");
    return;
  }

  AckPacket ack = {};
  ack.magic = TEST_MAGIC;
  ack.version = TEST_VERSION;
  ack.sequenceAck = packet.sequence;
  ack.rxUptimeMs = millis();
  ack.rxPacketCount = packets;
  memcpy(ack.echoedAdcRaw, packet.adcRaw, sizeof(ack.echoedAdcRaw));
  ack.crc = ackPacketCrc(ack);

  esp_now_send(mac, (const uint8_t *)&ack, sizeof(ack));
}

#if ESP_IDF_VERSION_MAJOR >= 5
void onReceive(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  const uint8_t *mac = info->src_addr;
#else
void onReceive(const uint8_t *mac, const uint8_t *data, int len) {
#endif
  if (!sameMac(mac, TRANSMITTER_MAC)) {
    ignoredPackets++;
    return;
  }

  if (len != sizeof(RawAdcPacket)) {
    badPackets++;
    return;
  }

  RawAdcPacket packet;
  memcpy(&packet, data, sizeof(packet));
  if (packet.magic != TEST_MAGIC || packet.version != TEST_VERSION || rawPacketCrc(packet) != packet.crc) {
    badPackets++;
    return;
  }

  portENTER_CRITICAL(&packetMux);
  lastPacket = packet;
  memcpy(lastSender, mac, 6);
  haveSender = true;
  newPacket = true;
  if (haveSequence) {
    uint16_t delta = (uint16_t)(packet.sequence - lastReceivedSequence);
    if (delta > 1 && delta < 32768) {
      missedPackets += delta - 1;
    }
  } else {
    haveSequence = true;
  }
  lastReceivedSequence = packet.sequence;
  packets++;
  lastPacketRxMs = millis();
  portEXIT_CRITICAL(&packetMux);
}

void setupEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  WiFi.setSleep(false);
  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    while (true) delay(1000);
  }

  esp_now_set_pmk(ESPNOW_PMK);
  if (!addPeer(TRANSMITTER_MAC)) {
    Serial.println("Encrypted transmitter peer add failed");
    while (true) delay(1000);
  }

  esp_now_register_recv_cb(onReceive);
}

void setup() {
  Serial.begin(115200);
  delay(400);

  setupEspNow();

  Serial.print("ESP-NOW raw ADC receiver MAC: ");
  Serial.println(WiFi.macAddress());
  Serial.println("Secure unicast peer: B4:3A:45:3F:46:BC");
  Serial.println("Waiting for encrypted raw ADS1115 A0..A3 packets. ACKs are sent back to the transmitter.");
}

void loop() {
  uint8_t sender[6];
  RawAdcPacket packet;
  bool shouldAck = false;

  portENTER_CRITICAL(&packetMux);
  if (newPacket) {
    memcpy(sender, lastSender, 6);
    packet = lastPacket;
    newPacket = false;
    shouldAck = true;
  }
  portEXIT_CRITICAL(&packetMux);

  if (shouldAck && packet.sequence != lastAckSequence) {
    sendAck(sender, packet);
    lastAckSequence = packet.sequence;
  }

  if (millis() - lastSerialMs >= 250) {
    lastSerialMs = millis();

    portENTER_CRITICAL(&packetMux);
    packet = lastPacket;
    memcpy(sender, lastSender, 6);
    bool hasSender = haveSender;
    uint32_t packetCount = packets;
    uint32_t badCount = badPackets;
    uint32_t ignoredCount = ignoredPackets;
    uint32_t missedCount = missedPackets;
    uint32_t packetAgeMs = millis() - lastPacketRxMs;
    portEXIT_CRITICAL(&packetMux);

    Serial.printf("packets=%lu bad=%lu ignored=%lu missed=%lu ",
                  packetCount, badCount, ignoredCount, missedCount);
    if (hasSender) {
      Serial.print("from=");
      printMac(sender);
      Serial.printf(" seq=%u rxAge=%lums A0=%d A1=%d A2=%d A3=%d enc=0x%02X\n",
                    packet.sequence,
                    packetAgeMs,
                    packet.adcRaw[0], packet.adcRaw[1], packet.adcRaw[2], packet.adcRaw[3],
                    packet.encoderBits);
    } else {
      Serial.println("waiting...");
    }
  }
}
