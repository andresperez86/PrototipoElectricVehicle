/*
  Título: Medidor de velocidad, voltaje y corriente (ACS712-30A con divisor 2:1) con registro en SD (ESP32)
  Descripción: 
   - Calcula velocidad promediada con dos sensores Hall.
   - Mide voltaje (divisor 15k/1k) y corriente (ACS712 con divisor 2:1 en Vout).
   - Guarda los datos en formato CSV en microSD.
  Autor: Andres Pérez, Fernand Hernandez, Call Guerra
*/

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>

// --- Pines ESP32 ---
const int SENSOR_HALL_PIN_1 = 13;   // Sensor Hall 1
const int SENSOR_HALL_PIN_2 = 22;   // Sensor Hall 2
const int SensorPin_C      = 4;    // ACS712 (corriente)
const int SensorPin_V      = 12;    // Divisor resistivo (voltaje)
const int SD_CS_PIN        = 33;     // Chip Select de la SD

// --- ADC ---
#define ADC_RESOLUCION 4096.0
inline float rawToMilliVolts(int raw) { return 3300.0f * (raw / 4095.0f); }

// --- Parámetros físicos ---
float radioLlanta_mm = 250.0;
const int PULSOS_POR_REVOLUCION_POR_SENSOR = 1;

// --- Variables globales ---
volatile unsigned long contadorPulsos1 = 0;
volatile unsigned long contadorPulsos2 = 0;
unsigned long tiempoAnterior = 0;
const int intervaloMedicion_ms = 1000;

float voltaje = 0.0;
float corriente = 0.0;

// ===============================================================
// ⚙️ CONFIGURACIÓN SENSOR DE CORRIENTE (ACS712-30A con divisor 2:1)
// ===============================================================
const float K_DIV_I = 0.50f;               // 10k/10k → divide a la mitad
const float SENS_mV_A_PIN = 66.0f * K_DIV_I; // Sensibilidad efectiva = 33 mV/A
float I_ZERO_mV = 0.0f;                    // Offset medido (≈1650 mV)
const float alphaI = 0.22f;                // Filtro suavizado

// --- Divisor de voltaje para batería (15k / 1k) ---
const float R1_V = 15000.0f, R2_V = 1000.0f;
const float DIV_GAIN_V = (R1_V + R2_V) / R2_V; // = 16.0
const float alphaV = 0.15f;                    // Filtro suavizado voltaje

// --- ISR ---
void IRAM_ATTR pulsoDetectado1() { contadorPulsos1++; }
void IRAM_ATTR pulsoDetectado2() { contadorPulsos2++; }

// ===============================================================
// 🔧 Funciones auxiliares
// ===============================================================
int readAdcMedian(int pin, int samples=15) {
  samples = constrain(samples, 3, 31);
  int buf[31];
  for (int i=0; i<samples; i++) buf[i] = analogRead(pin);
  for (int i=0; i<samples-1; i++)
    for (int j=i+1; j<samples; j++)
      if (buf[j] < buf[i]) { int t=buf[i]; buf[i]=buf[j]; buf[j]=t; }
  return buf[samples/2];
}

// Calibrar el punto cero del ACS712 en reposo
float calibrarCero_mV() {
  const int N = 120;
  long acc = 0;
  for (int i=0; i<N; i++) { acc += (long)rawToMilliVolts(readAdcMedian(SensorPin_C,9)); delay(2); }
  return (float)acc / N;
}

// ===============================================================
// ⚡ FUNCIÓN: Leer Corriente y Voltaje
// ===============================================================
void leerCorrienteVoltaje() {
  // Voltaje de batería (divisor 15k/1k)
  float mvV  = rawToMilliVolts(readAdcMedian(SensorPin_V,15));
  float Vbat = (mvV / 1000.0f) * DIV_GAIN_V;

  // Corriente ACS712 con divisor 2:1
  float mvI  = rawToMilliVolts(readAdcMedian(SensorPin_C,15));
  float Iamp = (mvI - I_ZERO_mV) / SENS_mV_A_PIN;

  // Filtros suaves IIR
  static bool init=false; static float v_f=0, i_f=0;
  if(!init){ v_f=Vbat; i_f=Iamp; init=true; }

  v_f += alphaV * (Vbat - v_f);
  i_f += alphaI * (Iamp - i_f);

  voltaje = v_f;
  corriente = i_f;
}

// ===============================================================
// 💾 FUNCIÓN: Guardar datos en SD (CSV)
// ===============================================================
void guardarEnSD(float v1, float v2, float vProm, float volt, float corr) {
  File archivo = SD.open("/mediciones.csv", FILE_APPEND);
  if (archivo) {
    archivo.print(millis() / 1000.0, 2); archivo.print(",");
    archivo.print(v1, 2); archivo.print(",");
    archivo.print(v2, 2); archivo.print(",");
    archivo.print(vProm, 2); archivo.print(",");
    archivo.print(volt, 2); archivo.print(",");
    archivo.println(corr, 3);
    archivo.close();
  } else {
    Serial.println("⚠️ Error al abrir el archivo CSV.");
  }
}

// ===============================================================
// 🧩 SETUP
// ===============================================================
void setup() {
  Serial.begin(115200);
  Serial.println("🔧 Inicializando sistema de medición...");

  // ADC
  analogReadResolution(12);
  analogSetPinAttenuation(SensorPin_V, ADC_11db);
  analogSetPinAttenuation(SensorPin_C, ADC_11db);

  // Sensores Hall
  pinMode(SENSOR_HALL_PIN_1, INPUT_PULLUP);
  pinMode(SENSOR_HALL_PIN_2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(SENSOR_HALL_PIN_1), pulsoDetectado1, FALLING);
  attachInterrupt(digitalPinToInterrupt(SENSOR_HALL_PIN_2), pulsoDetectado2, FALLING);

  // Calibración del cero del ACS712 (en reposo)
  Serial.println("Calibrando cero de corriente (reposo)...");
  I_ZERO_mV = calibrarCero_mV();
  Serial.print("I_ZERO_mV = "); Serial.println(I_ZERO_mV,1);
  Serial.println("Teclea 'z' + Enter para recalibrar en ejecución.");

  // SD
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("❌ No se pudo inicializar la tarjeta SD.");
  } else {
    Serial.println("✅ Tarjeta SD inicializada correctamente.");
    if (!SD.exists("/mediciones.csv")) {
      File archivo = SD.open("/mediciones.csv", FILE_WRITE);
      if (archivo) {
        archivo.println("Tiempo_s,Velocidad_S1_kmh,Velocidad_S2_kmh,Velocidad_Prom_kmh,Voltaje_V,Corriente_A");
        archivo.close();
        Serial.println("📝 Archivo 'mediciones.csv' creado con encabezado.");
      }
    }
  }
}

// ===============================================================
// 🔁 LOOP PRINCIPAL
// ===============================================================
void loop() {
  // Permitir recalibrar el cero manualmente
  if (Serial.available()) {
    char c = Serial.read();
    if (c=='z' || c=='Z') {
      Serial.println("Recalibrando cero...");
      I_ZERO_mV = calibrarCero_mV();
      Serial.print("Nuevo I_ZERO_mV = "); Serial.println(I_ZERO_mV, 1);
    }
  }

  if (millis() - tiempoAnterior >= intervaloMedicion_ms) {
    float velocidad_kmh_1 = 0, velocidad_kmh_2 = 0;

    noInterrupts();
    unsigned long pulsos1 = contadorPulsos1;
    unsigned long pulsos2 = contadorPulsos2;
    contadorPulsos1 = 0;
    contadorPulsos2 = 0;
    interrupts();

    float circunferencia_m = 2 * PI * (radioLlanta_mm / 1000.0);

    if (pulsos1 > 0) {
      float rev1 = (float)pulsos1 / PULSOS_POR_REVOLUCION_POR_SENSOR;
      float dist1 = rev1 * circunferencia_m;
      velocidad_kmh_1 = (dist1 / (intervaloMedicion_ms / 1000.0)) * 3.6;
    }

    if (pulsos2 > 0) {
      float rev2 = (float)pulsos2 / PULSOS_POR_REVOLUCION_POR_SENSOR;
      float dist2 = rev2 * circunferencia_m;
      velocidad_kmh_2 = (dist2 / (intervaloMedicion_ms / 1000.0)) * 3.6;
    }

    float velocidadPromedio_kmh = (velocidad_kmh_1 + velocidad_kmh_2) / 2.0;

    // --- Medición de corriente y voltaje ---
    leerCorrienteVoltaje();

    // --- Mostrar resultados ---
    Serial.print("S1: "); Serial.print(velocidad_kmh_1, 2);
    Serial.print(" km/h | S2: "); Serial.print(velocidad_kmh_2, 2);
    Serial.print(" km/h | PROM: "); Serial.print(velocidadPromedio_kmh, 2);
    Serial.print(" km/h || Voltaje: "); Serial.print(voltaje, 2);
    Serial.print(" V | Corriente: "); Serial.print(corriente, 3);
    Serial.println(" A");

    // --- Guardar en SD ---
    guardarEnSD(velocidad_kmh_1, velocidad_kmh_2, velocidadPromedio_kmh, voltaje, corriente);

    tiempoAnterior = millis();
  }
}
