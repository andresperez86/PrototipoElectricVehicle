/*
  Título: Medidor de velocidad, voltaje y corriente en ESP32
  Descripción: 
   - Calcula velocidad usando 2 sensores de efecto Hall y promedia.
   - Lee voltaje desde divisor resistivo.
   - Lee corriente desde ACS709.
  Autor: Andres Pérez, Fernand Hernandez, Call Guerra 
*/

// --- Pines ---
const int SENSOR_HALL_PIN_1 = 25;   // GPIO25 → Sensor Hall 1
const int SENSOR_HALL_PIN_2 = 26;   // GPIO26 → Sensor Hall 2
const int SensorPin_C = 34;         // GPIO34 → ACS709 (corriente, ADC1_CH6)
const int SensorPin_V = 35;         // GPIO35 → Divisor de voltaje (ADC1_CH7)

// --- Constantes ADC ---
#define ADC_VREF_mV 3300.0     // Referencia ADC ESP32 en mV
#define ADC_RESOLUCION 4096.0  // Resolución ADC (12 bits)

// --- Parámetros del sistema ---
float radioLlanta_mm = 250.0;
const int PULSOS_POR_REVOLUCION_POR_SENSOR = 1;

// --- Variables globales ---
volatile unsigned long contadorPulsos1 = 0;
volatile unsigned long contadorPulsos2 = 0;
unsigned long tiempoAnterior = 0;
const int intervaloMedicion_ms = 1000;

float voltaje = 0.0;
float corriente = 0.0;

// --- ISRs ---
void IRAM_ATTR pulsoDetectado1() { contadorPulsos1++; }
void IRAM_ATTR pulsoDetectado2() { contadorPulsos2++; }

// --- Lectura de corriente y voltaje ---
void leerCorrienteVoltaje() {
  int rawC = analogRead(SensorPin_C); // Lectura ADC corriente
  int rawV = analogRead(SensorPin_V); // Lectura ADC voltaje

  // Conversión ADC → voltaje medido en mV
  float mV_C = (rawC * ADC_VREF_mV) / ADC_RESOLUCION;
  float mV_V = (rawV * ADC_VREF_mV) / ADC_RESOLUCION;

  // --- Ajustar según calibración del ACS709 ---
  // Ejemplo: ACS709 de 30A → 37 mV/A, offset en Vcc/2
  float sensibilidad_mVporA = 37.0;
  float offset_mV = ADC_VREF_mV / 2.0;

  corriente = (mV_C - offset_mV) / sensibilidad_mVporA; // en Amperios

  // --- Ajustar divisor resistivo ---
  // Ejemplo: R1=30k, R2=7.5k (división 5:1)
  float factorDivisor = 5.0; 
  voltaje = (mV_V / 1000.0) * factorDivisor; // en Voltios
}

// --- Setup ---
void setup() {
  Serial.begin(115200);
  Serial.println("Inicializando medidor...");

  pinMode(SENSOR_HALL_PIN_1, INPUT_PULLUP);
  pinMode(SENSOR_HALL_PIN_2, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(SENSOR_HALL_PIN_1), pulsoDetectado1, FALLING);
  attachInterrupt(digitalPinToInterrupt(SENSOR_HALL_PIN_2), pulsoDetectado2, FALLING);
}

// --- Loop ---
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

    leerCorrienteVoltaje();

    Serial.print("S1: ");
    Serial.print(velocidad_kmh_1, 2);
    Serial.print(" km/h | S2: ");
    Serial.print(velocidad_kmh_2, 2);
    Serial.print(" km/h | PROM: ");
    Serial.print(velocidadPromedio_kmh, 2);
    Serial.print(" km/h || Voltaje: ");
    Serial.print(voltaje, 2);
    Serial.print(" V | Corriente: ");
    Serial.print(corriente, 2);
    Serial.println(" A");

    tiempoAnterior = millis();
  }
}
