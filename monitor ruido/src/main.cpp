#include <Arduino.h>

const int MIC_PIN = 34; 
const bool SIMULATION_MODE = true; 

const int WINDOW_SIZE = 10;
int readings[WINDOW_SIZE];
int readIndex = 0;
long total = 0;
int average = 0;

enum EstadoRuido { CALMO, MODERADO, RUIDOSO };
EstadoRuido estadoAtual = CALMO;

const int LIM_CALMO_MOD_SUBIDA  = 1800;
const int LIM_CALMO_MOD_DESCIDA = 1600;
const int LIM_MOD_RUIDO_SUBIDA  = 3000;
const int LIM_MOD_RUIDO_DESCIDA = 2800;

void setup() {
  Serial.begin(115200);

  if (!SIMULATION_MODE) {
    pinMode(MIC_PIN, INPUT);
  }

  for (int i = 0; i < WINDOW_SIZE; i++) {
    readings[i] = 0;
  }
}

void loop() {
  int rawValue = 0;

  if (SIMULATION_MODE) {
    rawValue = random(1000, 1650); 
    if (random(0, 30) == 0) { 
      rawValue = random(3100, 3900); 
    }
  } else {
    rawValue = analogRead(MIC_PIN);
  }

  total = total - readings[readIndex];
  readings[readIndex] = rawValue;
  total = total + readings[readIndex];
  readIndex = (readIndex + 1) % WINDOW_SIZE;
  average = total / WINDOW_SIZE;

  switch (estadoAtual) {
    case CALMO:
      if (average > LIM_CALMO_MOD_SUBIDA) {
        estadoAtual = MODERADO;
      }
      break;

    case MODERADO:
      if (average > LIM_MOD_RUIDO_SUBIDA) {
        estadoAtual = RUIDOSO;
      } else if (average < LIM_CALMO_MOD_DESCIDA) {
        estadoAtual = CALMO;
      }
      break;

    case RUIDOSO:
      if (average < LIM_MOD_RUIDO_DESCIDA) {
        estadoAtual = MODERADO;
      }
      break;
  }

  Serial.print("Raw:");
  Serial.print(rawValue);
  Serial.print(",Filtered:");
  Serial.print(average);
  Serial.print(",Status:");

  if (estadoAtual == CALMO) {
    Serial.println("CALMO");
  } else if (estadoAtual == MODERADO) {
    Serial.println("MODERADO");
  } else {
    Serial.println("RUIDOSO");
  }

  delay(100);
}