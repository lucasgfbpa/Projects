// ============================================================================
// Monitor IoT de Conforto Acústico e Acessibilidade (Versão Completa & Unificada)
// Plataforma: ESP32 | Sensor: MAX4466 | Display: OLED SSD1306
// ============================================================================

#include <Arduino.h>
#include <math.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ThingSpeak.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "secrets.h"

// ---------------- Configurações de Hardware ----------------
#define MIC_PIN 34                 // Entrada analógica (ADC1)
#define OLED_SDA 21
#define OLED_SCL 22
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_ADDRESS 0x3C
#define OLED_RESET -1

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
WiFiClient espClient;
PubSubClient mqttClient(espClient);

// ---------------- Amostragem e Filtragem  ----------------
constexpr uint32_t SAMPLE_RATE_HZ = 8000;
constexpr uint32_t SAMPLE_WINDOW_MS = 1000;
constexpr uint32_t SAMPLES_PER_WINDOW = SAMPLE_RATE_HZ * SAMPLE_WINDOW_MS / 1000;
constexpr uint32_t SAMPLE_PERIOD_US = 1000000UL / SAMPLE_RATE_HZ;

constexpr size_t MINUTE_WINDOW_SIZE = 60;
float minuteReadings[MINUTE_WINDOW_SIZE] = {0};
size_t minuteWriteIndex = 0;
size_t minuteValidCount = 0;
float minuteAverageRms = 0.0f;
float overallAverageRms = 0.0f;
uint32_t totalValidReadings = 0;

// ---------------- Classificação e Histerese  ----------------
constexpr float RMS_CALM_TO_MODERATE = 1800.0f;
constexpr float RMS_MODERATE_TO_NOISY = 3000.0f;
constexpr float HYSTERESIS_FACTOR = 0.85f;

enum EnvironmentState : uint8_t {
  CALMO = 0,
  MODERADO = 1,
  RUIDOSO = 2
};

EnvironmentState environmentState = CALMO;

// ---------------- Credenciais e Nuvem----------------
const char* WIFI_NAME = SECRET_WIFI_NAME;
const char* WIFI_PASSWORD = SECRET_WIFI_PASSWORD;

// ThingSpeak
const int myChannelNumber = SECRET_myChannelNumber;
const char* myApiKey = SECRET_myApiKey;

// MQTT Broker - Exemplo usando broker público da HiveMQ ou Mosquitto
const char* MQTT_SERVER = "broker.hivemq.com";
const int MQTT_PORT = 1883;
const char* MQTT_TOPIC_RMS = "esp32_ruido/rms";
const char* MQTT_TOPIC_STATE = "esp32_ruido/estado";

// Intervalos de controle
constexpr uint32_t WIFI_RETRY_INTERVAL_MS = 10000;
constexpr uint32_t THINGSPEAK_INTERVAL_MS = 20000;
constexpr uint32_t MQTT_PUBLISH_INTERVAL_MS = 2000;

uint32_t lastWifiAttemptMs = 0;
uint32_t lastThingSpeakWriteMs = 0;
uint32_t lastMqttPublishMs = 0;

float latestRms = 0.0f;
bool hasValidReading = false;
bool oledReady = false;

// ---------------- Funções Auxiliares ----------------

const char* stateName(EnvironmentState state) {
  switch (state) {
    case CALMO: return "CALMO";
    case MODERADO: return "MODERADO";
    case RUIDOSO: return "RUIDOSO";
    default: return "INDEFINIDO";
  }
}

// Cálculo do RMS real eliminando offset DC 
float readMicrophoneRms() {
  double sum = 0.0;
  double sumSquares = 0.0;
  uint32_t nextSampleUs = micros();

  for (uint32_t i = 0; i < SAMPLES_PER_WINDOW; ++i) {
    while ((int32_t)(micros() - nextSampleUs) < 0) {
      // Aguarda o tempo exato da amostragem
    }

    const uint16_t sample = analogRead(MIC_PIN);
    sum += sample;
    sumSquares += (double)sample * (double)sample;
    nextSampleUs += SAMPLE_PERIOD_US;
  }

  const double mean = sum / SAMPLES_PER_WINDOW;
  double variance = (sumSquares / SAMPLES_PER_WINDOW) - (mean * mean);

  if (variance < 0.0) variance = 0.0;
  return (float)sqrt(variance);
}

// Máquina de Estados com Histerese 
EnvironmentState classifyEnvironment(float rms) {
  const float calmReturnLimit = RMS_CALM_TO_MODERATE * HYSTERESIS_FACTOR;
  const float noisyReturnLimit = RMS_MODERATE_TO_NOISY * HYSTERESIS_FACTOR;

  switch (environmentState) {
    case CALMO:
      if (rms >= RMS_MODERATE_TO_NOISY) environmentState = RUIDOSO;
      else if (rms >= RMS_CALM_TO_MODERATE) environmentState = MODERADO;
      break;

    case MODERADO:
      if (rms >= RMS_MODERATE_TO_NOISY) environmentState = RUIDOSO;
      else if (rms < calmReturnLimit) environmentState = CALMO;
      break;

    case RUIDOSO:
      if (rms < calmReturnLimit) environmentState = CALMO;
      else if (rms < noisyReturnLimit) environmentState = MODERADO;
      break;
  }

  return environmentState;
}

// Atualização de Médias Móveis 
void updateAverages(float rms) {
  ++totalValidReadings;
  if (totalValidReadings == 1) {
    overallAverageRms = rms;
  } else {
    overallAverageRms += (rms - overallAverageRms) / totalValidReadings;
  }

  minuteReadings[minuteWriteIndex] = rms;
  minuteWriteIndex = (minuteWriteIndex + 1) % MINUTE_WINDOW_SIZE;
  if (minuteValidCount < MINUTE_WINDOW_SIZE) {
    ++minuteValidCount;
  }

  float sum = 0.0f;
  for (size_t i = 0; i < minuteValidCount; ++i) {
    sum += minuteReadings[i];
  }
  minuteAverageRms = (minuteValidCount > 0) ? sum / minuteValidCount : 0.0f;
}

// Conexão e manutenção Wi-Fi
void startOrMaintainWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;

  const uint32_t now = millis();
  if (now - lastWifiAttemptMs >= WIFI_RETRY_INTERVAL_MS) {
    lastWifiAttemptMs = now;
    Serial.println("Reconectando Wi-Fi...");
    WiFi.disconnect();
    WiFi.begin(WIFI_NAME, WIFI_PASSWORD);
  }
}

// Gestão de Protocolo MQTT 
void reconnectMQTT() {
  if (WiFi.status() == WL_CONNECTED && !mqttClient.connected()) {
    String clientId = "ESP32_NoiseMonitor_" + String(random(0xffff), HEX);
    if (mqttClient.connect(clientId.c_str())) {
      Serial.println("MQTT Conectado!");
    }
  }
}

void publishMQTTIfDue() {
  if (!hasValidReading || !mqttClient.connected()) return;

  const uint32_t now = millis();
  if (now - lastMqttPublishMs >= MQTT_PUBLISH_INTERVAL_MS) {
    lastMqttPublishMs = now;
    
    char rmsStr[8];
    dtostrf(latestRms, 1, 2, rmsStr);
    
    mqttClient.publish(MQTT_TOPIC_RMS, rmsStr);
    mqttClient.publish(MQTT_TOPIC_STATE, stateName(environmentState));
  }
}

// Envio ThingSpeak
void sendToThingSpeakIfDue() {
  if (!hasValidReading || WiFi.status() != WL_CONNECTED) return;

  const uint32_t now = millis();
  if (now - lastThingSpeakWriteMs >= THINGSPEAK_INTERVAL_MS) {
    lastThingSpeakWriteMs = now;
    ThingSpeak.setField(1, latestRms);
    ThingSpeak.setField(2, (int)environmentState);

    const int responseCode = ThingSpeak.writeFields(myChannelNumber, myApiKey);
    if (responseCode == 200) {
      Serial.println("ThingSpeak atualizado com sucesso!");
    }
  }
}

// Interface gráfica no Display OLED 
void drawDisplay() {
  if (!oledReady) return;

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  
  display.setCursor(0, 0);
  display.print("MONITOR ACUSTICO IOT");

  display.setCursor(0, 12);
  display.print("Estado: ");
  display.print(stateName(environmentState));

  display.setCursor(0, 25);
  display.print("RMS Rel: ");
  if (hasValidReading) display.print(latestRms, 1);
  else display.print("lendo...");

  display.setCursor(0, 38);
  display.print("Med 1m: ");
  display.print(minuteAverageRms, 1);

  display.setCursor(0, 49);
  display.print("Med Geral: ");
  display.print(overallAverageRms, 1);

  display.setCursor(0, 58);
  display.print("Rede: ");
  display.print(WiFi.status() == WL_CONNECTED ? "Wi-Fi+MQTT" : "Offline");

  display.display();
}

// ---------------- Setup e Loop Principal ----------------
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\nInicializando Monitor Acústico Unificado...");

  // Configuração Analógica
  analogReadResolution(12);
  analogSetPinAttenuation(MIC_PIN, ADC_11db);
  pinMode(MIC_PIN, INPUT);

  // Inicialização do OLED
  Wire.begin(OLED_SDA, OLED_SCL);
  oledReady = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS);

  // Inicialização Wi-Fi e Nuvem
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_NAME, WIFI_PASSWORD);
  
  ThingSpeak.begin(espClient);
  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);

  drawDisplay();
  Serial.println("Sistema totalmente pronto!");
}

void loop() {
  // 1. Processamento e Tratamento do Som
  latestRms = readMicrophoneRms();
  hasValidReading = true;

  classifyEnvironment(latestRms);
  updateAverages(latestRms);

  // 2. Logs no Terminal Serial
  Serial.printf("RMS: %.2f | Estado: %s | Med 1m: %.2f | MQTT: %s\n",
                latestRms, stateName(environmentState), minuteAverageRms,
                mqttClient.connected() ? "OK" : "NOK");

  // 3. Gestão de Rede e Nuvem
  startOrMaintainWiFi();
  if (!mqttClient.connected()) reconnectMQTT();
  mqttClient.loop();

  publishMQTTIfDue();
  sendToThingSpeakIfDue();

  // 4. Interface Visual
  drawDisplay();
}