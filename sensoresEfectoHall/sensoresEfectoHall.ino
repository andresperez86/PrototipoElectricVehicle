/*
  Título: Medidor de velocidad, voltaje y corriente con registro en SD (ESP32)
  Descripción: 
   - Calcula velocidad promediada con dos sensores Hall.
   - Mide voltaje y corriente con ACS709 y divisor resistivo.
   - Guarda los datos en formato CSV en microSD.
  Autor: Andres Pérez, Fernand Hernandez, Call Guerra
*/

// --- Librerías ---
#include <SPI.h>
#include <SD.h>

// --- Pines ESP32 ---
const int SENSOR_HALL_PIN_1 = 25;   // Sensor Hall 1
const int SENSOR_HALL_PIN_2 = 26;   // Sensor Hall 2
const int SensorPin_C = 34;         // ACS709 (corriente)
const int SensorPin_V = 35;         // Divisor resistivo (voltaje)
const int SD_CS_PIN = 5;            // Chip Select de la SD

// --- ADC ---
#define ADC_VREF_mV 3300.0
#define ADC_RESOLUCION 4096.0

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

// --- ISR ---
void IRAM_ATTR pulsoDetectado1() { contadorPulsos1++; }
void IRAM_ATTR pulsoDetectado2() { contadorPulsos2++; }

// ===============================================================
// ⚡ FUNCIÓN: Leer Corriente y Voltaje (versión solicitada)
// ===============================================================
void leerCorrienteVoltaje() {
  // Lectura de corriente y voltaje con el ACS709
  int adc = analogRead(SensorPin_C);
  int adc1 = analogRead(SensorPin_V);
  corriente = map(adc, 2816, 3040, 0 , 6600) * 5 / 5000.0; // Escalado de corriente
  voltaje = map(adc1, 2816, 3040, 0 , 6600) * 5 / 5000.0;  // Escalado de voltaje
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
    archivo.println(corr, 2);
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

  pinMode(SENSOR_HALL_PIN_1, INPUT_PULLUP);
  pinMode(SENSOR_HALL_PIN_2, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(SENSOR_HALL_PIN_1), pulsoDetectado1, FALLING);
  attachInterrupt(digitalPinToInterrupt(SENSOR_HALL_PIN_2), pulsoDetectado2, FALLING);

  // Inicialización de SD
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
    Serial.print(" V | Corriente: "); Serial.print(corriente, 2);
    Serial.println(" A");

    // --- Guardar en SD ---
    guardarEnSD(velocidad_kmh_1, velocidad_kmh_2, velocidadPromedio_kmh, voltaje, corriente);

    tiempoAnterior = millis();
  }
}

