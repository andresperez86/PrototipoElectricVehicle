/*
  ESP32 · 2x Hall por periodo robusto + Voltaje (divisor 15k/1k) + Corriente (ACS709)
  - Time-out de inactividad para forzar 0 km/h
  - Rechazo de glitches (período mínimo / máximo plausibles)
  - Flanco único en interrupción (FALLING) para evitar doble trigger
  - Auto-trim lento del VZERO del ACS709 cuando |I| ~ 0

  Ajusta:
  - radioLlanta_mm
  - IMANES_POR_VUELTA
  - SENS_mV_A (tu versión ACS709)
  - K_ADC_V / K_ADC_C para calibración con multímetro
*/

#include <Arduino.h>

// ---- Pines ----
const int PIN_HALL1 = 25;      // GPIO25
const int PIN_HALL2 = 26;      // GPIO26
const int PIN_I     = 34;      // ACS709 (ADC1)
const int PIN_V     = 35;      // Divisor (ADC1)

// ---- Físico rueda ----
float radioLlanta_mm      = 250.0f;        // ⚙️
const int IMANES_POR_VUELTA = 1;           // ⚙️
const float CIRC_m = 2.0f * PI * (radioLlanta_mm / 1000.0f);

// ---- Límites de velocidad para filtrar glitches ----
const float VEL_KMH_MAX_FISICA = 120.0f;   // ⚙️ velocidad máx. plausible del vehículo
// Período mínimo (más rápido que VEL_MAX) => descartar
// per_min_us = 1e6 * (circunf / (vel_m_s * imanes))
const uint32_t PER_MIN_US = (uint32_t)(1e6f * (CIRC_m / ((VEL_KMH_MAX_FISICA/3.6f) * IMANES_POR_VUELTA)));
// Período máximo (muy lento / parado) => si excede TIMEOUT ⇒ 0
const uint32_t PER_MAX_US = 2000000UL;     // 2 s (protección adicional)
const uint32_t TIMEOUT_MS = 800;           // si no hay pulso en 0.8 s ⇒ 0 km/h
const uint32_t ANTIRREBOTE_US = 3000;      // 3 ms (LM393)

// ---- ADC ----
#define ADC_RES 4095.0f
float K_ADC_V = 1.000f; // ⚙️ calib voltaje (ajusta con multímetro)
float K_ADC_C = 1.000f; // ⚙️ calib corriente

// Divisor 15k // 1k
const float R1 = 15000.0f, R2 = 1000.0f;
const float DIV_GAIN = (R1 + R2) / R2; // 16.0

// ---- ACS709 ----
float SENS_mV_A = 28.0f;   // ⚙️ mV/A (±75A≈28, ±50A≈40, ±30A≈66)
float VZERO_mV   = 1650.0f; // se calibra al inicio
// Auto-trim lento del cero cuando |I| es pequeño
const float I_TRIM_TH_A = 0.3f;     // umbral “casi 0 A”
const float VZERO_TRIM_ALPHA = 0.01f; // más pequeño = más lento

// ---- Estado velocidad (por período) ----
volatile uint32_t t1_prev=0, t1_per=0;
volatile uint32_t t2_prev=0, t2_per=0;
volatile uint32_t t1_last_ms=0, t2_last_ms=0;

void IRAM_ATTR isrH1() {
  const uint32_t now_us = micros();
  if (now_us - t1_prev > ANTIRREBOTE_US) {
    const uint32_t p = now_us - t1_prev;
    // guarda solo si no es absurdamente pequeño (glitch)
    if (p >= PER_MIN_US && p <= PER_MAX_US) t1_per = p;
    t1_prev = now_us;
    t1_last_ms = millis();
  }
}

void IRAM_ATTR isrH2() {
  const uint32_t now_us = micros();
  if (now_us - t2_prev > ANTIRREBOTE_US) {
    const uint32_t p = now_us - t2_prev;
    if (p >= PER_MIN_US && p <= PER_MAX_US) t2_per = p;
    t2_prev = now_us;
    t2_last_ms = millis();
  }
}

// ---- Utils ADC ----
int readAdcMedian(int pin, int samples=15) {
  samples = constrain(samples, 3, 31);
  int buf[31];
  for (int i=0;i<samples;i++) buf[i]=analogRead(pin);
  for (int i=0;i<samples-1;i++)
    for (int j=i+1;j<samples;j++)
      if (buf[j] < buf[i]) { int t=buf[i]; buf[i]=buf[j]; buf[j]=t; }
  return buf[samples/2];
}
inline float adcToMilliVolts(int raw) { return 3300.0f * (raw / ADC_RES); }

float medirVzero_mV() {
  const int N=120;
  long acc=0;
  for (int i=0;i<N;i++) { acc += (long)adcToMilliVolts(readAdcMedian(PIN_I,9)); delay(2); }
  return (acc/(float)N) * K_ADC_C;
}

// ---- Señales filtradas ----
float voltaje=0.0f, corriente=0.0f;

void leerCorrienteVoltaje() {
  // Voltaje
  float mvV  = adcToMilliVolts(readAdcMedian(PIN_V,15)) * K_ADC_V;
  float Vbat = (mvV/1000.0f) * DIV_GAIN;

  // Corriente
  float mvC  = adcToMilliVolts(readAdcMedian(PIN_I,15)) * K_ADC_C;
  float Iamp = (mvC - VZERO_mV) / SENS_mV_A;

  // Auto-trim lento del cero cuando estamos cerca de 0 A
  if (fabsf(Iamp) < I_TRIM_TH_A) {
    // mover VZERO hacia mvC muy lentamente
    VZERO_mV = VZERO_mV + VZERO_TRIM_ALPHA * (mvC - VZERO_mV);
  }

  // Filtro IIR suave para visualización
  static bool init=false;
  static float v_f=0, i_f=0;
  if(!init){ v_f=Vbat; i_f=Iamp; init=true; }
  const float alpha=0.15f;
  v_f += alpha*(Vbat - v_f);
  i_f += alpha*(Iamp - i_f);

  voltaje = v_f;
  corriente = i_f;
}

// ---- Velocidad segura ----
float velFromPeriodSafe(uint32_t per_us, uint32_t last_ms) {
  // Si no hay pulso reciente ⇒ 0
  if (millis() - last_ms > TIMEOUT_MS) return 0.0f;
  if (per_us < PER_MIN_US || per_us > PER_MAX_US) return 0.0f;
  const float rev_s = (1e6f / per_us) / IMANES_POR_VUELTA;
  const float m_s   = rev_s * CIRC_m;
  const float kmh   = m_s * 3.6f;
  // Filtro por si algo escapó
  if (kmh < 0 || kmh > (VEL_KMH_MAX_FISICA+20)) return 0.0f;
  return kmh;
}

float mediana3(float a, float b, float c) {
  float mn = min(a, min(b, c));
  float mx = max(a, max(b, c));
  return (a + b + c) - mn - mx;
}

// ---- Setup ----
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("Inicializando...");

  analogReadResolution(12);
  analogSetPinAttenuation(PIN_V, ADC_11db);
  analogSetPinAttenuation(PIN_I, ADC_11db);

  pinMode(PIN_HALL1, INPUT_PULLUP);
  pinMode(PIN_HALL2, INPUT_PULLUP);

  // Usa flanco único; si tu módulo da pulso positivo, cambia a RISING
  attachInterrupt(digitalPinToInterrupt(PIN_HALL1), isrH1, FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_HALL2), isrH2, FALLING);

  // Inicializa prev para no medir un período gigante al principio
  t1_prev = micros();
  t2_prev = micros();
  t1_last_ms = t2_last_ms = millis();

  Serial.println("Calibrando cero de corriente (debe estar en reposo)...");
  VZERO_mV = medirVzero_mV();
  Serial.print("VZERO_mV = "); Serial.println(VZERO_mV,1);
  Serial.print("PER_MIN_US = "); Serial.println(PER_MIN_US);
  Serial.println("OK.");
}

// ---- Loop ----
unsigned long t_last=0;
const unsigned long INTERVALO_MS=500;

void loop() {
  leerCorrienteVoltaje();

  uint32_t p1, p2, l1, l2;
  noInterrupts(); p1=t1_per; p2=t2_per; l1=t1_last_ms; l2=t2_last_ms; interrupts();

  float v1 = velFromPeriodSafe(p1, l1);
  float v2 = velFromPeriodSafe(p2, l2);
  float v_med = mediana3(v1, v2, 0.5f*(v1+v2));

  if (millis() - t_last >= INTERVALO_MS) {
    t_last = millis();
    Serial.print("S1: "); Serial.print(v1,2);
    Serial.print(" km/h | S2: "); Serial.print(v2,2);
    Serial.print(" km/h | MED: "); Serial.print(v_med,2);
    Serial.print(" km/h || Voltaje: "); Serial.print(voltaje,2);
    Serial.print(" V | Corriente: "); Serial.print(corriente,2);
    Serial.println(" A");
  }
}
