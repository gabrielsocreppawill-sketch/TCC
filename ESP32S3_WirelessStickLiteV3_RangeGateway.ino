/*
21:07:09.695 -> test_name=range_test_001&gateway_chipid=B04DE2E22748&gateway_count=135&gateway_lat=-26.485000&gateway_long=-49.075000
&device_chipid=000093A3A6A73333&device_boardName=CubeCell_AB01_V2&device_count=138&device_txpower=20&rssi=-68&snr=12.0
&gps_lat=0.000000&gps_long=0.000000&gps_date=&gps_time=&gps_altitude=0.00&gps_satellites=0&battery_mv=3580&distance_m=0.00&command_sent=CMD_TXP%3D20%26LED%3DGOOD


20:30:07.582 -> test_name=range_test_001&gateway_chipid=B04DE2E22748&gateway_count=124
&gateway_lat=-26.485000&gateway_long=-49.075000
&device_chipid=000093A3A6A73333&device_boardName=CubeCell_AB01_V2
&device_count=4&device_txpower=14&rssi=-54&snr=12.0
&gps_lat=0.000000&gps_long=0.000000&gps_date=&gps_time=
&gps_altitude=0.00&gps_satellites=0
&battery_mv=3630
&distance_m=0.00
&command_sent=CMD_TXP%3D14%26LED%3DGOOD

*/
#include "Arduino.h"
#include "LoRaWan_APP.h"

#include <WiFi.h>
#include <HTTPClient.h>

// ======================================================
// WIFI / SERVIDOR
// ======================================================

const char* ssid     = "raspiAP";
const char* password = "raspberry";

const char* loggerServerName = "http://10.0.0.226/loggerLora/saveData.php";
const char* rangeServerName  = "http://10.0.0.226/loggerLora/saveRangeTest.php";

// ======================================================
// POSIÇÃO FIXA DO GATEWAY
// Ajustar para a posição real do Gateway
// ======================================================
// BLOCO H - LABORATORIOS DA INSTITUIÇÃO
const double GATEWAY_LAT  = -26.466257;
const double GATEWAY_LONG = -49.116018;
// ======================================================
// MODO TESTE
// 0 = LoRa real
// 1 = POST aleatório sem LoRa
// ======================================================

#define TEST_MODE_RANDOM 0

// ======================================================
// CONFIGURAÇÕES LORA
// Devem ser iguais às do CubeCell
// ======================================================

#define LORA_FREQUENCY        915000000

#define LORA_BANDWIDTH        0       // 0 = 125 kHz
#define LORA_SPREADING_FACTOR 7
#define LORA_CODINGRATE       1       // 1 = 4/5
#define LORA_PREAMBLE_LENGTH  8
#define LORA_SYMBOL_TIMEOUT   0
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON  false

#define GATEWAY_TX_POWER      14

const int8_t TX_POWER_MIN = 14;
const int8_t TX_POWER_MAX = 20;

// ======================================================
// VARIÁVEIS GATEWAY
// ======================================================

String g_chipid = "";
uint8_t g_count = 0;
uint8_t g_countI = 0;

String g_reservado = "0";

volatile bool loraPacketReceived = false;
volatile bool gatewayTxBusy = false;

String receivedPayload = "";

int16_t d_rssi = 0;
float d_srn = 0.0;

static RadioEvents_t RadioEvents;

// ======================================================
// FUNÇÃO: CHIP ID ESP32-S3
// ======================================================

String getGatewayChipId() {
  uint64_t chipid = ESP.getEfuseMac();

  char id[17];

  snprintf(
    id,
    sizeof(id),
    "%04X%08lX",
    (unsigned int)(chipid >> 32),
    (unsigned long)(chipid & 0xFFFFFFFF)
  );

  return String(id);
}

// ======================================================
// FUNÇÃO: URL ENCODE
// ======================================================

String urlEncode(const String& str) {
  String encoded = "";

  for (int i = 0; i < str.length(); i++) {
    char c = str.charAt(i);

    if (isalnum(c) || c == '-' || c == '_' || c == '.' || c == '~') {
      encoded += c;
    } else if (c == ' ') {
      encoded += '+';
    } else {
      char code0;
      char code1;

      code1 = (c & 0xF) + '0';
      if ((c & 0xF) > 9) {
        code1 = (c & 0xF) - 10 + 'A';
      }

      c = (c >> 4) & 0xF;
      code0 = c + '0';
      if (c > 9) {
        code0 = c - 10 + 'A';
      }

      encoded += '%';
      encoded += code0;
      encoded += code1;
    }
  }

  return encoded;
}

// ======================================================
// PARSER DO PAYLOAD COMPACTO DO CUBECELL
// Formato esperado:
// ID=...;BN=...;C=...;L=...;T=...;P=...;U=...;B=...;
// LA=...;LO=...;GD=...;GT=...;GA=...;GS=...;TX=...
// ======================================================

String getValueFromCompactPayload(String payload, String key) {
  String searchKey = key + "=";

  int startIndex = payload.indexOf(searchKey);

  if (startIndex < 0) {
    return "";
  }

  startIndex += searchKey.length();

  int endIndex = payload.indexOf(";", startIndex);

  if (endIndex < 0) {
    endIndex = payload.length();
  }

  return payload.substring(startIndex, endIndex);
}

// ======================================================
// WIFI
// ======================================================

void connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) {
    return;
  }

  Serial.print("Conectando ao WiFi: ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  unsigned long startAttempt = millis();

  while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < 20000) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi conectado!");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("ERRO: nao foi possivel conectar ao WiFi.");
  }
}

// ======================================================
// DISTÂNCIA HAVERSINE
// ======================================================

double degToRad(double deg) {
  return deg * PI / 180.0;
}

double distanceMeters(double lat1, double lon1, double lat2, double lon2) {
  const double R = 6371000.0;

  double dLat = degToRad(lat2 - lat1);
  double dLon = degToRad(lon2 - lon1);

  lat1 = degToRad(lat1);
  lat2 = degToRad(lat2);

  double a = sin(dLat / 2.0) * sin(dLat / 2.0) +
             cos(lat1) * cos(lat2) *
             sin(dLon / 2.0) * sin(dLon / 2.0);

  double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));

  return R * c;
}

// ======================================================
// DECISÃO DE POTÊNCIA E LED
// ======================================================

int decideRequestedTxPower(int currentTxPower, int rssi, float snr) {
  int requested = currentTxPower;

  if (rssi < -118 || snr < 0.0) {
    requested += 2;
  } else if (rssi < -110 || snr < 2.0) {
    requested += 1;
  }

  if (requested > TX_POWER_MAX) {
    requested = TX_POWER_MAX;
  }

  if (requested < TX_POWER_MIN) {
    requested = TX_POWER_MIN;
  }

  return requested;
}

String decideLedMode(int rssi, float snr) {
  if (rssi < -118 || snr < 0.0) {
    return "CRITICAL";
  }

  if (rssi < -110 || snr < 2.0) {
    return "LOW";
  }

  return "GOOD";
}

String buildCommandForCubeCell(String cubePayload, int rssi, float snr) {
  String txpStr = getValueFromCompactPayload(cubePayload, "TX");

  int currentTxPower = txpStr.length() > 0 ? txpStr.toInt() : 14;

  int requestedTxPower = decideRequestedTxPower(currentTxPower, rssi, snr);

  String ledModeStr = decideLedMode(rssi, snr);

  String command = "";

  command += "CMD_TXP=" + String(requestedTxPower);
  command += "&LED=" + ledModeStr;

  return command;
}

// ======================================================
// CALLBACKS LORA
// ======================================================

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
  char buffer[300];

  if (size >= sizeof(buffer)) {
    size = sizeof(buffer) - 1;
  }

  memcpy(buffer, payload, size);
  buffer[size] = '\0';

  receivedPayload = String(buffer);

  d_rssi = rssi;
  d_srn = (float)snr;

  loraPacketReceived = true;

  Radio.Sleep();
}

void OnRxTimeout(void) {
  Serial.println("LoRa RX timeout.");
  Radio.Sleep();
  Radio.Rx(0);
}

void OnRxError(void) {
  Serial.println("LoRa RX error.");
  Radio.Sleep();
  Radio.Rx(0);
}

void OnTxDone(void) {
  Serial.println("Comando LoRa enviado ao CubeCell.");
  gatewayTxBusy = false;
  Radio.Sleep();
  Radio.Rx(0);
}

void OnTxTimeout(void) {
  Serial.println("Timeout ao enviar comando LoRa.");
  gatewayTxBusy = false;
  Radio.Sleep();
  Radio.Rx(0);
}

// ======================================================
// INICIALIZAÇÃO LORA
// ======================================================

void initLoRa() {
  Serial.println("Inicializando LoRa SX1262...");

  RadioEvents.RxDone = OnRxDone;
  RadioEvents.RxTimeout = OnRxTimeout;
  RadioEvents.RxError = OnRxError;
  RadioEvents.TxDone = OnTxDone;
  RadioEvents.TxTimeout = OnTxTimeout;

  Radio.Init(&RadioEvents);
  Radio.SetChannel(LORA_FREQUENCY);

  Radio.SetRxConfig(
    MODEM_LORA,
    LORA_BANDWIDTH,
    LORA_SPREADING_FACTOR,
    LORA_CODINGRATE,
    0,
    LORA_PREAMBLE_LENGTH,
    LORA_SYMBOL_TIMEOUT,
    LORA_FIX_LENGTH_PAYLOAD_ON,
    0,
    true,
    0,
    0,
    LORA_IQ_INVERSION_ON,
    true
  );

  Radio.Rx(0);

  Serial.println("LoRa em modo RX.");
}

// ======================================================
// ENVIO DE COMANDO LORA PARA O CUBECELL
// ======================================================

bool sendCommandToCubeCell(String command) {
  Serial.print("Enviando comando ao CubeCell: ");
  Serial.println(command);

  gatewayTxBusy = true;

  Radio.Sleep();

  Radio.SetTxConfig(
    MODEM_LORA,
    GATEWAY_TX_POWER,
    0,
    LORA_BANDWIDTH,
    LORA_SPREADING_FACTOR,
    LORA_CODINGRATE,
    LORA_PREAMBLE_LENGTH,
    LORA_FIX_LENGTH_PAYLOAD_ON,
    true,
    0,
    0,
    LORA_IQ_INVERSION_ON,
    3000
  );

  Radio.Send((uint8_t *)command.c_str(), command.length());

  unsigned long startWait = millis();

  while (gatewayTxBusy && millis() - startWait < 1200) {
    Radio.IrqProcess();
    delay(1);
  }

  if (gatewayTxBusy) {
    Serial.println("Aviso: comando pode nao ter finalizado dentro do tempo esperado.");
    gatewayTxBusy = false;
    Radio.Sleep();
    Radio.Rx(0);
    return false;
  }

  return true;
}

// ======================================================
// MONTA POST OPERACIONAL PARA saveData.php
// ======================================================

String buildPostDataFromLoRaPayload(String cubeCellPayload) {
  String postData = "";

  postData += "g_chipid=" + urlEncode(g_chipid);
  postData += "&g_count=" + String(g_count);

  postData += "&d_chipid=" + urlEncode(getValueFromCompactPayload(cubeCellPayload, "ID"));
  postData += "&d_boardName=" + urlEncode(getValueFromCompactPayload(cubeCellPayload, "BN"));
  postData += "&d_count=" + getValueFromCompactPayload(cubeCellPayload, "C");

  postData += "&d_l_amb=" + getValueFromCompactPayload(cubeCellPayload, "L");
  postData += "&d_t_amb=" + getValueFromCompactPayload(cubeCellPayload, "T");
  postData += "&d_p_amb=" + getValueFromCompactPayload(cubeCellPayload, "P");
  postData += "&d_u_amb=" + getValueFromCompactPayload(cubeCellPayload, "U");

  postData += "&d_mvbat=" + getValueFromCompactPayload(cubeCellPayload, "B");

  postData += "&d_gps_lat=" + getValueFromCompactPayload(cubeCellPayload, "LA");
  postData += "&d_gps_long=" + getValueFromCompactPayload(cubeCellPayload, "LO");
  postData += "&d_gps_date=" + urlEncode(getValueFromCompactPayload(cubeCellPayload, "GD"));
  postData += "&d_gps_time=" + urlEncode(getValueFromCompactPayload(cubeCellPayload, "GT"));
  postData += "&d_gps_altitude=" + getValueFromCompactPayload(cubeCellPayload, "GA");
  postData += "&d_gps_satellites=" + getValueFromCompactPayload(cubeCellPayload, "GS");

  postData += "&d_rssi=" + String(d_rssi);
  postData += "&d_srn=" + String(d_srn, 1);

  postData += "&g_reservado=" + urlEncode(g_reservado);
  postData += "&g_countI=" + String(g_countI);

  return postData;
}

// ======================================================
// ENVIA POST OPERACIONAL
// ======================================================

bool sendPostToServer(String postData) {
  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi();
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("ERRO: WiFi desconectado. POST operacional cancelado.");
    return false;
  }

  HTTPClient http;

  http.begin(loggerServerName);
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");

  Serial.println();
  Serial.println("Enviando POST operacional:");
  Serial.println(postData);

  int httpResponseCode = http.POST(postData);

  Serial.print("HTTP Response code operacional: ");
  Serial.println(httpResponseCode);

  if (httpResponseCode > 0) {
    String response = http.getString();

    Serial.print("Resposta saveData.php: ");
    Serial.println(response);

    http.end();

    return response.indexOf("OK") >= 0;
  }

  Serial.print("Erro no POST operacional: ");
  Serial.println(httpResponseCode);

  http.end();
  return false;
}

// ======================================================
// MONTA POST RANGE PARA saveRangeTest.php
// ======================================================

String buildRangePostData(String cubePayload, String commandSent) {
  String device_chipid    = getValueFromCompactPayload(cubePayload, "ID");
  String device_boardName = getValueFromCompactPayload(cubePayload, "BN");
  String device_count     = getValueFromCompactPayload(cubePayload, "C");

  String d_txpower = getValueFromCompactPayload(cubePayload, "TX");

  String gps_lat        = getValueFromCompactPayload(cubePayload, "LA");
  String gps_long       = getValueFromCompactPayload(cubePayload, "LO");
  String gps_date       = getValueFromCompactPayload(cubePayload, "GD");
  String gps_time       = getValueFromCompactPayload(cubePayload, "GT");
  String gps_altitude   = getValueFromCompactPayload(cubePayload, "GA");
  String gps_satellites = getValueFromCompactPayload(cubePayload, "GS");

  String battery_mv = getValueFromCompactPayload(cubePayload, "B");

  double latDevice = gps_lat.toDouble();
  double lonDevice = gps_long.toDouble();

  double dist = 0.0;

  if (latDevice != 0.0 && lonDevice != 0.0) {
    dist = distanceMeters(GATEWAY_LAT, GATEWAY_LONG, latDevice, lonDevice);
  }

  String postData = "";

  postData += "test_name=range_test_001";

  postData += "&gateway_chipid=" + urlEncode(g_chipid);
  postData += "&gateway_count=" + String(g_count);
  postData += "&gateway_lat=" + String(GATEWAY_LAT, 6);
  postData += "&gateway_long=" + String(GATEWAY_LONG, 6);

  postData += "&device_chipid=" + urlEncode(device_chipid);
  postData += "&device_boardName=" + urlEncode(device_boardName);
  postData += "&device_count=" + device_count;

  postData += "&device_txpower=" + d_txpower;

  postData += "&rssi=" + String(d_rssi);
  postData += "&snr=" + String(d_srn, 1);

  postData += "&gps_lat=" + gps_lat;
  postData += "&gps_long=" + gps_long;
  postData += "&gps_date=" + urlEncode(gps_date);
  postData += "&gps_time=" + urlEncode(gps_time);
  postData += "&gps_altitude=" + gps_altitude;
  postData += "&gps_satellites=" + gps_satellites;

  postData += "&battery_mv=" + battery_mv;

  postData += "&distance_m=" + String(dist, 2);

  postData += "&command_sent=" + urlEncode(commandSent);

  return postData;
}

// ======================================================
// ENVIA POST RANGE
// ======================================================

bool sendRangePostToServer(String postData) {
  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi();
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("ERRO: WiFi desconectado. POST range cancelado.");
    return false;
  }

  HTTPClient http;

  http.begin(rangeServerName);
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");

  Serial.println();
  Serial.println("Enviando POST range:");
  Serial.println(postData);

  int httpResponseCode = http.POST(postData);

  Serial.print("HTTP Response code range: ");
  Serial.println(httpResponseCode);

  if (httpResponseCode > 0) {
    String response = http.getString();

    Serial.print("Resposta saveRangeTest.php: ");
    Serial.println(response);

    http.end();

    return response.indexOf("OK") >= 0;
  }

  Serial.print("Erro no POST range: ");
  Serial.println(httpResponseCode);

  http.end();
  return false;
}

// ======================================================
// TESTE ALEATÓRIO SEM LORA
// ======================================================

String buildRandomPostData() {
  String postData = "";

  postData += "g_chipid=" + urlEncode(g_chipid);
  postData += "&g_count=" + String(g_count);

  postData += "&d_chipid=CUBECELL_FAKE";
  postData += "&d_boardName=CubeCell_AB01_V2";
  postData += "&d_count=" + String(g_count);

  postData += "&d_l_amb=" + String(random(1000, 90000) / 100.0, 2);
  postData += "&d_t_amb=" + String(random(1800, 3500) / 100.0, 2);
  postData += "&d_p_amb=" + String(random(95000, 103000) / 100.0, 2);
  postData += "&d_u_amb=" + String(random(3000, 9000) / 100.0, 2);

  postData += "&d_mvbat=" + String(random(3300, 4200));

  postData += "&d_gps_lat=-26.485000";
  postData += "&d_gps_long=-49.075000";
  postData += "&d_gps_date=20260605";
  postData += "&d_gps_time=170000";
  postData += "&d_gps_altitude=120.50";
  postData += "&d_gps_satellites=8";

  postData += "&d_rssi=-70";
  postData += "&d_srn=7.5";

  postData += "&g_reservado=TESTE_RANDOM";
  postData += "&g_countI=" + String(g_countI);

  return postData;
}

String buildRandomRangePostData() {
  String commandSent = "CMD_TXP=14&LED=GOOD";

  String postData = "";

  postData += "test_name=range_test_random";

  postData += "&gateway_chipid=" + urlEncode(g_chipid);
  postData += "&gateway_count=" + String(g_count);
  postData += "&gateway_lat=" + String(GATEWAY_LAT, 6);
  postData += "&gateway_long=" + String(GATEWAY_LONG, 6);

  postData += "&device_chipid=CUBECELL_FAKE";
  postData += "&device_boardName=CubeCell_AB01_V2";
  postData += "&device_count=" + String(g_count);

  postData += "&device_txpower=14";

  postData += "&rssi=-70";
  postData += "&snr=7.5";

  postData += "&gps_lat=-26.485000";
  postData += "&gps_long=-49.075000";
  postData += "&gps_date=20260605";
  postData += "&gps_time=170000";
  postData += "&gps_altitude=120.50";
  postData += "&gps_satellites=8";

  postData += "&battery_mv=3900";
  postData += "&distance_m=0.00";

  postData += "&command_sent=" + urlEncode(commandSent);

  return postData;
}

// ======================================================
// SETUP
// ======================================================

void setup() {
  Serial.begin(115200);
  delay(1000);

  // IMPORTANTE para Heltec ESP32-S3 + SX1262
  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);
  delay(100);

  Serial.println();
  Serial.println("LoggerLora Range Gateway");
  Serial.println("Heltec Wireless Stick Lite V3 - ESP32-S3 + SX1262");

  g_chipid = getGatewayChipId();

  Serial.print("Gateway ChipID: ");
  Serial.println(g_chipid);

  connectWiFi();

#if TEST_MODE_RANDOM == 0
  initLoRa();
#else
  Serial.println("MODO TESTE RANDOM ATIVO.");
#endif

  Serial.println("Sistema pronto.");
}

// ======================================================
// LOOP
// ======================================================

void loop() {

#if TEST_MODE_RANDOM == 1

  String postData = buildRandomPostData();

  bool ok = sendPostToServer(postData);

  if (ok) {
    Serial.println("POST random operacional enviado com sucesso.");
  } else {
    Serial.println("Falha no POST random operacional.");
  }

  String rangePostData = buildRandomRangePostData();

  bool okRange = sendRangePostToServer(rangePostData);

  if (okRange) {
    Serial.println("POST random range enviado com sucesso.");
  } else {
    Serial.println("Falha no POST random range.");
  }

  g_count++;
  g_countI++;

  delay(10000);

#else

  Radio.IrqProcess();

  if (loraPacketReceived) {
    loraPacketReceived = false;

    Serial.println();
    Serial.println("Pacote LoRa recebido:");
    Serial.println(receivedPayload);

    Serial.print("RSSI: ");
    Serial.println(d_rssi);

    Serial.print("SNR: ");
    Serial.println(d_srn);

    String commandToSend = buildCommandForCubeCell(receivedPayload, d_rssi, d_srn);

    bool commandOk = sendCommandToCubeCell(commandToSend);

    if (commandOk) {
      Serial.println("Comando enviado antes dos POSTs.");
    } else {
      Serial.println("Comando nao confirmado antes dos POSTs.");
    }

    String postDataOperational = buildPostDataFromLoRaPayload(receivedPayload);

    bool okLogger = sendPostToServer(postDataOperational);

    if (okLogger) {
      Serial.println("Dados operacionais enviados para loggerLora.");
    } else {
      Serial.println("Falha ao enviar dados operacionais.");
    }

    String postDataRange = buildRangePostData(receivedPayload, commandToSend);

    bool okRange = sendRangePostToServer(postDataRange);

    if (okRange) {
      Serial.println("Dados de alcance enviados para lora_range_tests.");
    } else {
      Serial.println("Falha ao enviar dados de alcance.");
    }

    g_count++;
    g_countI++;

    Radio.Rx(0);
  }

#endif
}