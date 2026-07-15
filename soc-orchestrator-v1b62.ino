/*
Name: SOC Orchestrator (JBD BMS Edition)
Version: 1.2.0
Software Version: V1B62
Year: 2026

Target protocol: JBD BMS (0xFF00 / 0xFF01 / 0xFF02, CMD03)
Config source:   0x1815 BLE controller JSON (B1..B10 keys + Conf)
Aggregation:     networkConf 0=series, 1=parallel 
*/

#include <NimBLEDevice.h>
#include <ArduinoJson.h>
#include <vector>
#include <string>
#include <algorithm>
#include <cstring>
#include <cmath>
#include <cctype>
#include <cstdlib>
#include <cstdio>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#if __has_include("nvs.hpp")
#include "nvs.hpp"
#else
#error "nvs.hpp not found. Add it to include path."
#endif

#if __has_include("can_protocols.hpp")
#include "can_protocols.hpp"
#else
#error "can_protocols.hpp not found. Add it to include path."
#endif

// =========================== CONFIG ===========================

// --- 0x1815 controller UUIDs (config fetch + TX to phone) ---
static const NimBLEUUID DISCOVER_SERVICE_UUID((uint16_t)0x1815);
static const NimBLEUUID DISCOVER_WRITE_UUID("9f7c66a2-b6ed-4be4-a045-ec12af692e91");
static const NimBLEUUID DISCOVER_RECV_UUID("8149f2ad-2324-4210-a98d-a7739eb87b4f");

// --- JBD BMS UUIDs ---
static const NimBLEUUID JBD_SERVICE_UUID((uint16_t)0xFF00);
static const NimBLEUUID JBD_NOTIFY_UUID((uint16_t)0xFF01);
static const NimBLEUUID JBD_WRITE_UUID((uint16_t)0xFF02);

// --- JBD CMD03 query command ---
static const uint8_t JBD_CMD03[] = { 0xDD, 0xA5, 0x03, 0x00, 0xFF, 0xFD, 0x77 };
static const uint8_t JBD_CMD04[] = { 0xDD, 0xA5, 0x04, 0x00, 0xFF, 0xFC, 0x77 };

// Outgoing monitoring frame source MAC (12 hex chars, no separators)
static const char* UNIT_MAC_HEX = "3030F94ED7AD";

// Timing
constexpr uint32_t JSON_SCAN_TIMEOUT_MS = 20000;
constexpr uint32_t JBD_READ_TIMEOUT_MS = 400;  // per-device CMD03 reply timeout
constexpr uint32_t JBD_FRAME_IDLE_MS = 80;     // silence = frame complete
constexpr uint32_t POST_CYCLE_DELAY_MS = 50;
constexpr uint8_t MAX_RETRY_PER_DEVICE = 3;

// BLE connection params
constexpr uint16_t MIN_INTERVAL_UNIT = 6;
constexpr uint16_t MAX_INTERVAL_UNIT = 9;
constexpr uint16_t LATENCY = 0;
constexpr uint16_t TIMEOUT_UNIT = 200;

constexpr size_t MAX_BATTERIES = 10;

// NVS
static const char* NVS_NAMESPACE = "soc_ble_cfg";
static const char* NVS_KEY_COUNT = "mac_count";
static const char* NVS_KEY_CONF = "conf";

// CAN
can_protocols::Victron vic;
constexpr uint32_t canRate = 500;

// Global aggregated values (shared with CAN task)
int aggSOC = 0;
float totalVoltage = 0.0f;
float totalCurrent = 0.0f;
float maxTempC = 0.0f;

// =========================== JBD DATA ===========================
struct JBDData {
  float voltage = 0.0f;
  float current = 0.0f;
  float temperature = 0.0f;
  uint8_t soc = 0;
  std::vector<float> cellVoltages;  // new
  bool valid = false;
};

// =========================== STATE ===========================
static std::vector<std::string> discoveredMACs;
static int networkConf = 0;  // 0=series, 1=parallel

// Per-cycle JBD RX state (reused for each connection)
static std::vector<uint8_t> jbdRxBuffer;
static uint32_t jbdLastRxMs = 0;

// Aggregates
static std::vector<float> parsedVoltages;
static std::vector<int> parsedSOCs;
static std::vector<float> parsedTempsC;
static int parsedCount = 0;
static float aggregatedVoltage = 0.0f;
static float aggregatedCurrent = 0.0f;
static int aggregatedSOC = 0;
static float highestTempC = -1000.0f;

// Write helper globals (1815 TX)
static NimBLEClient* pClient = nullptr;
static bool isConnected = false;
static bool actionsTaken = false;

// NVS
static Storage bleCfgNvs(NVS_NAMESPACE);
static bool nvsReady = false;

// =========================== HELPERS ===========================
static std::string normalizeMAC(const std::string& raw) {
  std::string out = raw;
  for (char& c : out) c = (char)std::toupper((unsigned char)c);
  return out;
}

static std::string hexPad(uint32_t value, int width) {
  char buf[16];
  snprintf(buf, sizeof(buf), "%0*X", width, (unsigned int)value);
  return std::string(buf);
}

static String macKeyByIndex(uint32_t i) {
  return String("mac_") + String((int)i);
}

static bool initBleCfgNvs() {
  if (nvsReady) return true;
  nvsReady = (bleCfgNvs.initialise() == ESP_OK);
  if (!nvsReady) Serial.println("[NVS] Failed to initialize.");
  return nvsReady;
}

static bool saveConfigToNvs(const std::vector<std::string>& macs, int conf) {
  if (!initBleCfgNvs()) return false;
  uint32_t oldCount = bleCfgNvs.getUInt(NVS_KEY_COUNT, 0);
  for (uint32_t i = 0; i < oldCount; ++i) bleCfgNvs.remove(macKeyByIndex(i).c_str());
  uint32_t count = (uint32_t)std::min(macs.size(), (size_t)MAX_BATTERIES);
  bleCfgNvs.putUInt(NVS_KEY_COUNT, count);
  bleCfgNvs.putUChar(NVS_KEY_CONF, (conf == 1) ? 1 : 0);
  for (uint32_t i = 0; i < count; ++i)
    bleCfgNvs.putString(macKeyByIndex(i).c_str(), String(macs[i].c_str()));
  Serial.printf("[NVS] Saved config. MAC count=%u Conf=%d\n", (unsigned)count, conf);
  return true;
}

static bool loadConfigFromNvs(std::vector<std::string>& macs, int& conf) {
  macs.clear();
  conf = 0;
  if (!initBleCfgNvs()) return false;
  conf = (bleCfgNvs.getUChar(NVS_KEY_CONF, 0) == 1) ? 1 : 0;
  uint32_t count = std::min(bleCfgNvs.getUInt(NVS_KEY_COUNT, 0), (uint32_t)MAX_BATTERIES);
  for (uint32_t i = 0; i < count; ++i) {
    String s = bleCfgNvs.getString(macKeyByIndex(i).c_str(), "");
    s.trim();
    if (s.length() > 0) macs.push_back(normalizeMAC(std::string(s.c_str())));
  }
  if (macs.empty()) return false;
  Serial.printf("[NVS] Loaded config. MAC count=%u Conf=%d\n", (unsigned)macs.size(), conf);
  return true;
}

// =========================== CONFIG JSON ===========================
static bool parseControllerJson(const std::string& json,
                                std::vector<std::string>& macs, int& conf) {
  macs.clear();
  conf = 0;
  DynamicJsonDocument doc(4096);
  auto jerr = deserializeJson(doc, json);
  if (jerr) {
    Serial.print("[Error] JSON parse failed: ");
    Serial.println(jerr.c_str());
    return false;
  }
  if (!doc.containsKey("Device03")) return false;
  JsonArray arr = doc["Device03"].as<JsonArray>();
  if (arr.size() == 0) return false;
  JsonObject entry = arr[0].as<JsonObject>();
  if (entry.containsKey("Conf")) {
    JsonVariant cv = entry["Conf"];
    if (cv.is<const char*>()) conf = atoi(cv.as<const char*>());
    else if (cv.is<int>()) conf = cv.as<int>();
    else if (cv.is<long>()) conf = (int)cv.as<long>();
  }
  conf = (conf == 1) ? 1 : 0;
  for (int i = 1; i <= (int)MAX_BATTERIES; ++i) {
    char key[4];
    snprintf(key, sizeof(key), "B%d", i);
    if (!entry.containsKey(key)) break;
    const char* mac = entry[key];
    if (mac && strlen(mac) > 0) macs.push_back(normalizeMAC(std::string(mac)));
  }
  return !macs.empty();
}

static bool fetchLiveConfigFrom1815(std::vector<std::string>& macsOut, int& confOut) {
  macsOut.clear();
  confOut = 0;
  NimBLEScan* scan = NimBLEDevice::getScan();
  scan->setInterval(100);
  scan->setWindow(100);
  scan->setActiveScan(true);
  scan->clearResults();
  Serial.println("[Config] Scanning for 0x1815 controller...");
  NimBLEScanResults res = scan->getResults(100, false);

  const NimBLEAdvertisedDevice* controller = nullptr;
  for (int i = 0; i < res.getCount(); ++i) {
    const NimBLEAdvertisedDevice* adv = res.getDevice(i);
    if (adv && adv->haveServiceUUID() && adv->isAdvertisingService(DISCOVER_SERVICE_UUID)) {
      controller = adv;
      break;
    }
  }
  if (!controller) {
    Serial.println("[Config] 0x1815 controller not found.");
    return false;
  }
  Serial.printf("[Config] Found controller: %s\n", controller->toString().c_str());

  NimBLEClient* ctlClient = NimBLEDevice::createClient();
  if (!ctlClient->connect(controller)) {
    Serial.println("[Config] Failed to connect controller.");
    NimBLEDevice::deleteClient(ctlClient);
    return false;
  }
  NimBLERemoteService* ctlSvc = ctlClient->getService(DISCOVER_SERVICE_UUID);
  if (!ctlSvc) {
    Serial.println("[Config] Controller service missing.");
    ctlClient->disconnect();
    NimBLEDevice::deleteClient(ctlClient);
    return false;
  }
  NimBLERemoteCharacteristic* recvChr = ctlSvc->getCharacteristic(DISCOVER_RECV_UUID);
  if (!recvChr || !recvChr->canNotify()) {
    Serial.println("[Config] Controller RX characteristic missing/no notify.");
    ctlClient->disconnect();
    NimBLEDevice::deleteClient(ctlClient);
    return false;
  }

  std::string jsonAcc;
  bool jsonReady = false;
  bool subOk = recvChr->subscribe(true, [&](NimBLERemoteCharacteristic*, uint8_t* data, size_t len, bool) {
    jsonAcc.append(reinterpret_cast<char*>(data), len);
    if (jsonAcc.find("}]}") != std::string::npos) jsonReady = true;
  });
  if (!subOk) {
    Serial.println("[Config] Subscribe failed.");
    ctlClient->disconnect();
    NimBLEDevice::deleteClient(ctlClient);
    return false;
  }
  uint32_t t0 = millis();
  while (!jsonReady && (millis() - t0) < JSON_SCAN_TIMEOUT_MS) vTaskDelay(pdMS_TO_TICKS(10));
  recvChr->unsubscribe();
  ctlClient->disconnect();
  NimBLEDevice::deleteClient(ctlClient);
  if (!jsonReady) {
    Serial.println("[Config] JSON timeout.");
    return false;
  }
  Serial.print("[Config] JSON: ");
  Serial.println(jsonAcc.c_str());
  return parseControllerJson(jsonAcc, macsOut, confOut);
}

// =========================== JBD PARSE ===========================
static uint16_t readBE16(const uint8_t* d, int pos) {
  return (uint16_t)(d[pos] << 8) | d[pos + 1];
}
static int16_t readSignedBE16(const uint8_t* d, int pos) {
  return (int16_t)readBE16(d, pos);
}

static bool parseJBDCmd03(const std::vector<uint8_t>& f, JBDData& out) {
  if (f.size() < 10) return false;
  if (f[0] != 0xDD || f[1] != 0x03) return false;

  uint8_t payloadLen = f[3];
  size_t expected = 4 + payloadLen + 3;
  if (f.size() < expected || f[expected - 1] != 0x77) return false;

  const uint8_t* data = &f[4];

  out.voltage = readBE16(data, 0) * 0.01f;
  out.current = readSignedBE16(data, 2) * 0.01f;
  out.soc = data[19];

  // Average all NTC sensors
  uint8_t ntc = data[22];
  float tempSum = 0.0f;
  int validNTCs = 0;
  for (int i = 0; i < ntc; i++) {
    int pos = 23 + (i * 2);
    if ((pos + 1) >= (int)payloadLen) break;
    tempSum += (readBE16(data, pos) - 2731) * 0.1f;
    validNTCs++;
  }
  if (validNTCs > 0) out.temperature = tempSum / validNTCs;

  out.valid = true;
  return true;
}
static bool parseJBDCmd04(const std::vector<uint8_t>& f, JBDData& out) {
  if (f.size() < 7) return false;
  if (f[0] != 0xDD || f[1] != 0x04) return false;
  uint8_t payloadLen = f[3];
  size_t expected = 4 + payloadLen + 3;
  if (f.size() < expected || f[expected - 1] != 0x77) return false;
  out.cellVoltages.clear();
  int cellCount = payloadLen / 2;
  for (int i = 0; i < cellCount; i++) {
    out.cellVoltages.push_back(readBE16(f.data(), 4 + (i * 2)) / 1000.0f);
  }
  return true;
}

// =========================== JBD READ ===========================
// Reads one JBD BMS by MAC. Sends CMD03, waits for complete frame, parses.
static bool readJBDBattery(const std::string& mac, JBDData& out) {
  out = JBDData{};  // reset

  NimBLEAddress addr(mac, BLE_ADDR_PUBLIC);
  NimBLEClient* client = NimBLEDevice::createClient();

  Serial.printf("[JBD %s] Connecting...\n", mac.c_str());
  if (!client->connect(addr)) {

    Serial.printf("[JBD %s] Connect failed\n", mac.c_str());
    NimBLEDevice::deleteClient(client);
    return false;
  }
  Serial.printf(
    "[JBD %s] Connected peer=%s\n",
    mac.c_str(),
    client->getPeerAddress().toString().c_str());

  client->updateConnParams(MIN_INTERVAL_UNIT, MAX_INTERVAL_UNIT, LATENCY, TIMEOUT_UNIT);

  NimBLERemoteService* svc = client->getService(JBD_SERVICE_UUID);
  if (!svc) {
    Serial.printf("[JBD %s] Service 0xFF00 not found\n", mac.c_str());
    client->disconnect();
    NimBLEDevice::deleteClient(client);
    return false;
  }

  NimBLERemoteCharacteristic* notifyChr = svc->getCharacteristic(JBD_NOTIFY_UUID);
  NimBLERemoteCharacteristic* writeChr = svc->getCharacteristic(JBD_WRITE_UUID);

  if (!notifyChr || !writeChr) {
    Serial.printf("[JBD %s] Characteristics not found\n", mac.c_str());
    client->disconnect();
    NimBLEDevice::deleteClient(client);
    return false;
  }
  if (!notifyChr->canNotify()) {
    Serial.printf("[JBD %s] Notify not supported\n", mac.c_str());
    client->disconnect();
    NimBLEDevice::deleteClient(client);
    return false;
  }

  // Prepare RX state
  jbdRxBuffer.clear();
  jbdLastRxMs = 0;

  std::string expectedMac = mac;

  bool subOk = notifyChr->subscribe(
    true,
    [expectedMac](NimBLERemoteCharacteristic*,
                  uint8_t* data,
                  size_t len,
                  bool) {
      Serial.printf(
        "[RX %s] len=%u\n",
        expectedMac.c_str(),
        (unsigned)len);

      jbdLastRxMs = millis();

      for (size_t i = 0; i < len; i++) {

        Serial.printf("%02X ", data[i]);

        if (data[i] == 0xDD)
          jbdRxBuffer.clear();

        jbdRxBuffer.push_back(data[i]);
      }

      Serial.println();
    });
  if (!subOk) {
    Serial.printf("[JBD %s] Subscribe failed\n", mac.c_str());
    client->disconnect();
    NimBLEDevice::deleteClient(client);
    return false;
  }

  vTaskDelay(pdMS_TO_TICKS(300));

  // Send CMD03
  bool wr = writeChr->writeValue(
    JBD_CMD03,
    sizeof(JBD_CMD03),
    true);

  Serial.printf(
    "[JBD %s] CMD03 write=%s\n",
    mac.c_str(),
    wr ? "OK" : "FAILED");
  // Wait for a complete frame (idle silence after last byte)
  bool parsed = false;
  uint32_t deadline = millis() + JBD_READ_TIMEOUT_MS;
  while (millis() < deadline) {
    if (!jbdRxBuffer.empty() && jbdLastRxMs > 0 && (millis() - jbdLastRxMs) > JBD_FRAME_IDLE_MS) {
      std::vector<uint8_t> frame = jbdRxBuffer;
      jbdRxBuffer.clear();

      // Debug: print raw frame
      Serial.printf("[JBD %s] Frame (%u bytes):", mac.c_str(), (unsigned)frame.size());
      for (auto b : frame) Serial.printf(" %02X", b);
      Serial.println();

      parsed = parseJBDCmd03(frame, out);
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  notifyChr->unsubscribe();
  client->disconnect();
  NimBLEDevice::deleteClient(client);

  if (!parsed) Serial.printf("[JBD %s] Parse failed or timeout\n", mac.c_str());
  return parsed;
}

// =========================== FRAME BUILD ===========================
static uint32_t encodeCurrentForFrame(float amps) {
  uint32_t mag = (uint32_t)lroundf(fabsf(amps) * 100.0f);
  if (mag > 0x7FFFFFFFUL) mag = 0x7FFFFFFFUL;
  return (amps < 0.0f) ? (0x80000000UL | mag) : mag;
}

static std::string buildPhoneFrame(
  int aggSOC_, float totalVoltage_, float totalCurrent_,
  const std::vector<float>& batteryVoltages,
  const std::vector<int>& batterySOCs,
  const std::vector<float>& batteryTempsC,
  float maxTempF) {

  constexpr uint8_t WIFI_FLAG = 0x0;
  constexpr uint8_t ALARM = 0x0A;
  constexpr uint8_t VERSION = 0x01;
  constexpr uint16_t YEAR = 0x07E8;

  aggSOC_ = std::max(0, std::min(100, aggSOC_));
  uint32_t totalVRaw = (uint32_t)lroundf(std::max(0.0f, totalVoltage_) * 100.0f);
  if (totalVRaw > 0xFFFF) totalVRaw = 0xFFFF;

  std::string frame;
  frame.reserve(119);
  frame += "~";
  frame += UNIT_MAC_HEX;
  frame += hexPad(WIFI_FLAG, 1);
  frame += hexPad((uint32_t)aggSOC_, 2);
  frame += hexPad(totalVRaw, 4);
  frame += hexPad(encodeCurrentForFrame(totalCurrent_), 8);

  for (size_t i = 0; i < MAX_BATTERIES; ++i) {
    float v = (i < batteryVoltages.size()) ? batteryVoltages[i] : 0.0f;
    uint32_t raw = (uint32_t)lroundf(std::max(0.0f, v) * 100.0f);
    if (raw > 0xFFFF) raw = 0xFFFF;
    frame += hexPad(raw, 4);
  }
  for (size_t i = 0; i < MAX_BATTERIES; ++i) {
    int s = (i < batterySOCs.size()) ? batterySOCs[i] : 0;
    frame += hexPad((uint32_t)std::max(0, std::min(100, s)), 2);
  }
  frame += hexPad(ALARM, 2);
  for (size_t i = 0; i < MAX_BATTERIES; ++i) {
    int tC = (i < batteryTempsC.size()) ? (int)lroundf(batteryTempsC[i]) : 0;
    frame += hexPad((uint32_t)std::max(0, std::min(255, tC)), 2);
  }
  int maxTF = std::max(0, std::min(255, (int)lroundf(maxTempF)));
  frame += hexPad((uint32_t)maxTF, 2);
  frame += hexPad(VERSION, 2);
  frame += hexPad(YEAR, 4);
  frame += ":";
  return frame;
}

// =========================== TX TO 1815 ===========================
static bool writeValue(const char* data, size_t length) {
  if (!(isConnected && actionsTaken && pClient)) {
    Serial.println("[Write] Not connected or not ready.");
    return false;
  }
  NimBLERemoteService* remoteSvc = pClient->getService(DISCOVER_SERVICE_UUID);
  if (!remoteSvc) {
    Serial.println("[Write] Service 0x1815 not found.");
    return false;
  }
  NimBLERemoteCharacteristic* writeChr = remoteSvc->getCharacteristic(DISCOVER_WRITE_UUID);
  if (!writeChr) {
    Serial.println("[Write] Write characteristic not found.");
    return false;
  }
  bool withResponse = writeChr->canWrite();
  if (!withResponse && !writeChr->canWriteNoResponse()) {
    Serial.println("[Write] Characteristic is not writable.");
    return false;
  }
  constexpr size_t chunkSize = 20;
  for (size_t i = 0; i < length; i += chunkSize) {
    size_t n = std::min(chunkSize, length - i);
    bool ok = writeChr->writeValue(reinterpret_cast<const uint8_t*>(data + i), n, withResponse);
    if (!ok) {
      Serial.printf("[Write] Failed at offset %u\n", (unsigned)i);
      return false;
    }
    Serial.printf("[Write] chunk: %.*s\n", (int)n, data + i);
    vTaskDelay(pdMS_TO_TICKS(50));
  }
  return true;
}

static void scanAndSendToPhone(const std::string& frame) {
  Serial.println("[TX] Scanning for 0x1815 device...");
  NimBLEScan* scan = NimBLEDevice::getScan();
  scan->setInterval(50);
  scan->setWindow(50);
  scan->setActiveScan(true);
  scan->clearResults();
  NimBLEScanResults res = scan->getResults(50, false);

  const NimBLEAdvertisedDevice* phone = nullptr;
  for (int i = 0; i < res.getCount(); ++i) {
    const NimBLEAdvertisedDevice* adv = res.getDevice(i);
    if (adv && adv->haveServiceUUID() && adv->isAdvertisingService(DISCOVER_SERVICE_UUID)) {
      phone = adv;
      break;
    }
  }
  if (!phone) {
    Serial.println("[TX] 0x1815 not nearby. Aggregated values shown on Serial only.");
    return;
  }
  Serial.printf("[TX] Found 0x1815: %s\n", phone->toString().c_str());

  NimBLEClient* txClient = NimBLEDevice::createClient();
  if (!txClient->connect(phone)) {
    Serial.println("[TX] Connect failed.");
    NimBLEDevice::deleteClient(txClient);
    return;
  }
  pClient = txClient;
  isConnected = true;
  actionsTaken = true;
  bool ok = writeValue(frame.c_str(), frame.size());
  Serial.println(ok ? "[TX] Message sent successfully." : "[TX] Message send failed.");
  txClient->disconnect();
  NimBLEDevice::deleteClient(txClient);
  pClient = nullptr;
  isConnected = false;
  actionsTaken = false;
}

// =========================== MAIN TASK ===========================
void orchestratorTask(void* pvParameters) {
  (void)pvParameters;

  NimBLEDevice::init("ESP32-Orch");
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);
  NimBLEDevice::setSecurityAuth(false, false, false);

  initBleCfgNvs();
  loadConfigFromNvs(discoveredMACs, networkConf);

  for (;;) {
    Serial.println("=== New cycle ===");

    // 1) Fetch live config from 1815 (MACs + Conf)
    std::vector<std::string> liveMACs;
    int liveConf = 0;
    if (fetchLiveConfigFrom1815(liveMACs, liveConf)) {
      discoveredMACs = liveMACs;
      networkConf = liveConf;
      saveConfigToNvs(discoveredMACs, networkConf);
      Serial.printf("[Config] Live config applied. MAC count=%u Conf=%d\n",
                    (unsigned)discoveredMACs.size(), networkConf);
    } else {
      if (!loadConfigFromNvs(discoveredMACs, networkConf)) {
        Serial.println("[Config] No live config and no saved MACs. Waiting...");
        vTaskDelay(pdMS_TO_TICKS(POST_CYCLE_DELAY_MS));
        continue;
      }
      Serial.printf("[Config] Using saved config. MAC count=%u Conf=%d\n",
                    (unsigned)discoveredMACs.size(), networkConf);
    }

    // 2) Read all JBD batteries
    aggregatedVoltage = 0.0f;
    aggregatedCurrent = 0.0f;
    aggregatedSOC = 0;
    highestTempC = -1000.0f;
    parsedCount = 0;
    parsedVoltages.clear();
    parsedSOCs.clear();
    parsedTempsC.clear();

    for (size_t di = 0; di < discoveredMACs.size(); ++di) {
      const std::string& mac = discoveredMACs[di];
      bool gotDevice = false;

for (uint8_t attempt = 1; attempt <= MAX_RETRY_PER_DEVICE && !gotDevice; ++attempt) {
        Serial.printf("=== JBD %s (attempt %u/%u) ===\n",
                      mac.c_str(), (unsigned)attempt, (unsigned)MAX_RETRY_PER_DEVICE);

        JBDData batt;
        if (!readJBDBattery(mac, batt)) {
          Serial.printf("[JBD %s] Read failed, retrying...\n", mac.c_str());
          vTaskDelay(pdMS_TO_TICKS(300));
          continue;
        }

        aggregatedVoltage += batt.voltage;
        aggregatedCurrent += batt.current;
        aggregatedSOC += batt.soc;
        if (batt.temperature > highestTempC) highestTempC = batt.temperature;

        parsedVoltages.push_back(batt.voltage);
        parsedSOCs.push_back((int)batt.soc);
        parsedTempsC.push_back(batt.temperature);
        parsedCount++;
        gotDevice = true;

        Serial.printf("[JBD %s] OK: I=%.2fA V=%.2fV SOC=%d Temp=%.2fC\n",
                      mac.c_str(), batt.current, batt.voltage, batt.soc, batt.temperature);
      }  

      if (!gotDevice) {
        Serial.printf("[JBD %s] Failed after all retries. Inserting zero placeholder.\n", mac.c_str());
        parsedVoltages.push_back(0.0f);
        parsedSOCs.push_back(0);
        parsedTempsC.push_back(0.0f);
      }
    }  

    if (parsedCount <= 0) {
      Serial.println("[Warning] No device data parsed this cycle.");
      vTaskDelay(pdMS_TO_TICKS(POST_CYCLE_DELAY_MS));
      continue;
    }

    // 3) Aggregate
    // Conf 0 = series  → V=sum,  I=avg
    // Conf 1 = parallel → V=avg, I=sum
    aggSOC = aggregatedSOC / parsedCount;
    totalVoltage = (networkConf == 1) ? (aggregatedVoltage / parsedCount) : aggregatedVoltage;
    totalCurrent = (networkConf == 1) ? aggregatedCurrent : (aggregatedCurrent / parsedCount);
    maxTempC = (highestTempC < -200.0f) ? 0.0f : highestTempC;
    float maxTempF = (maxTempC * 9.0f / 5.0f) + 32.0f;

    Serial.println("=== Aggregated Results ===");
    Serial.printf("Configured batteries : %u\n", (unsigned)discoveredMACs.size());
    Serial.printf("Devices parsed       : %d\n", parsedCount);
    Serial.printf("Aggregated SOC       : %d%%\n", aggSOC);
    Serial.printf("Total Voltage        : %.2fV\n", totalVoltage);
    Serial.printf("Total Current        : %.2fA\n", totalCurrent);
    Serial.printf("Highest Temp         : %.2fC / %.2fF\n", maxTempC, maxTempF);

    // 4) Build + transmit frame to phone app via 1815
    std::string frame = buildPhoneFrame(
      aggSOC, totalVoltage, totalCurrent,
      parsedVoltages, parsedSOCs, parsedTempsC, maxTempC);

    Serial.printf("[TX] Frame len=%u\n", (unsigned)frame.size());
    scanAndSendToPhone(frame);

    vTaskDelay(pdMS_TO_TICKS(POST_CYCLE_DELAY_MS));
  }
}

// =========================== CAN TASK ===========================
void can_task(void* parameter) {
  esp_err_t ret = canInit(GPIO_NUM_36, GPIO_NUM_35, canRate);
  while (1) {
    float can_data[11] = { totalVoltage, totalCurrent, (float)aggSOC,
                           0, 1000, maxTempC, 0, 0, 0, 0, 0 };
    vic.canDataSend(can_data, 0);
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

// =========================== SETUP ===========================
void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && (millis() - t0) < 1500) delay(10);
  Serial.println("Starting JBD SOC Orchestrator...");

  initBleCfgNvs();

  BaseType_t orchOk = xTaskCreate(orchestratorTask, "Orchestrator", 24000, nullptr, 1, nullptr);
  BaseType_t canOk = xTaskCreate(can_task, "CAN", 8192, nullptr, 1, nullptr);

  if (orchOk != pdPASS) Serial.println("[Task] Failed to create Orchestrator task");
  if (canOk != pdPASS) Serial.println("[Task] Failed to create CAN task");
}

void loop() {}