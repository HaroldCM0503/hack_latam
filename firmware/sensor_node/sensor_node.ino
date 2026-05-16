// HB100 doppler sensor node firmware for ESP32 - WiFi UDP version.
//
// Both nodes run this same firmware. Only NODE_ID differs (1 or 2).
// Each node connects to a shared WiFi network and broadcasts FFT-detected
// doppler events as JSON UDP packets to the laptop.
//
// Wiring (per ESP32):
//   HB100 IF output --[op-amp gain ~500-1000, AC-coupled, 50-3000 Hz bandpass]--> GPIO34 (ADC1_CH6)
//   HB100 VCC=5V, GND=GND
//   ESP32 powered from USB / power bank
//
// Setup on the laptop:
//   1) Start a WiFi hotspot (Windows Mobile Hotspot / macOS Internet Sharing) -
//      or use any WiFi network both the laptop and ESP32s can join.
//   2) Note the laptop's IP on that network (e.g. 192.168.137.1 on Windows hotspot).
//      Put it into LAPTOP_IP below and into fusion/config.py.
//   3) Set WIFI_SSID and WIFI_PASSWORD below.
//
// Dependencies (Arduino IDE / PlatformIO):
//   - arduinoFFT  (kosme/arduinoFFT >= 2.0)
//   - ESP32 board package (provides WiFi.h, WiFiUdp.h)
//
// Build with -DNODE_ID=1 or -DNODE_ID=2 (PlatformIO) or edit the #define below.

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <arduinoFFT.h>

// ---------- Build-time config ----------
#ifndef NODE_ID
#define NODE_ID 1            // 1 or 2 - must differ between the two ESP32s
#endif

// EDIT THESE FOR YOUR NETWORK ------------------------------------------------
static const char* WIFI_SSID     = "HACKLATAM_AP";
static const char* WIFI_PASSWORD = "debris2026";
static const char* LAPTOP_IP     = "192.168.137.1";    // Windows hotspot default
static const uint16_t LAPTOP_PORT = 5005;
// ---------------------------------------------------------------------------

// ---------- Signal chain constants ----------
static const int      HB100_ADC_PIN     = 34;          // ADC1_CH6, input-only
static const int      SAMPLE_RATE_HZ    = 8000;
static const uint16_t FFT_SIZE          = 256;         // -> 32 ms window, ~31 Hz bins
static const float    HB100_FREQ_HZ     = 10.525e9f;
static const float    SPEED_OF_LIGHT    = 3.0e8f;
static const float    HZ_PER_MPS        = 2.0f * HB100_FREQ_HZ / SPEED_OF_LIGHT;
static const float    F_MIN_HZ          = 50.0f;       // ~0.7 m/s
static const float    F_MAX_HZ          = 3000.0f;     // ~40  m/s
static const float    MIN_SNR_DB        = 10.0f;

// ---------- State ----------
static double vReal[FFT_SIZE];
static double vImag[FFT_SIZE];
static ArduinoFFT<double> FFT(vReal, vImag, FFT_SIZE, (double)SAMPLE_RATE_HZ);

static WiFiUDP udp;
static bool wifi_ready = false;

// ---------- WiFi connect with retry ----------
static void connect_wifi() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);                       // lower latency, slightly higher power
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) {
    delay(250);
    Serial.print(".");
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    wifi_ready = true;
    Serial.print("WiFi OK. IP: ");
    Serial.println(WiFi.localIP());
    Serial.printf("Sending UDP to %s:%u\n", LAPTOP_IP, LAPTOP_PORT);
  } else {
    Serial.println("WiFi FAILED - will retry in loop");
  }
}

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.printf("\n=== HB100 Sensor Node #%d ===\n", (int)NODE_ID);

  analogReadResolution(12);
  analogSetPinAttenuation(HB100_ADC_PIN, ADC_11db);

  connect_wifi();
}

// ---------- Send one event as a JSON UDP packet ----------
static void send_event(uint32_t t_us, float v_mps, float amp, float snr_db) {
  if (!wifi_ready) return;

  // Compact JSON, one line, fits well under 1500 bytes MTU.
  char buf[160];
  int n = snprintf(buf, sizeof(buf),
      "{\"node\":%u,\"t\":%lu,\"v\":%.3f,\"amp\":%.1f,\"snr\":%.1f}",
      (unsigned)NODE_ID,
      (unsigned long)t_us,
      v_mps, amp, snr_db);

  udp.beginPacket(LAPTOP_IP, LAPTOP_PORT);
  udp.write((const uint8_t*)buf, n);
  udp.endPacket();
}

// ---------- Sampling + FFT loop ----------
void loop() {
  // Reconnect if WiFi dropped.
  if (WiFi.status() != WL_CONNECTED) {
    wifi_ready = false;
    Serial.println("WiFi reconnecting...");
    connect_wifi();
    return;
  }

  // 1) Sample FFT_SIZE values at SAMPLE_RATE_HZ (blocking, ~32 ms)
  const uint32_t period_us = 1000000UL / SAMPLE_RATE_HZ;
  uint32_t t_next = micros();
  for (int i = 0; i < FFT_SIZE; i++) {
    while ((int32_t)(micros() - t_next) < 0) { /* spin */ }
    vReal[i] = (double)analogRead(HB100_ADC_PIN);
    vImag[i] = 0.0;
    t_next += period_us;
  }
  uint32_t t_capture = micros();

  // 2) Remove DC
  double mean = 0.0;
  for (int i = 0; i < FFT_SIZE; i++) mean += vReal[i];
  mean /= FFT_SIZE;
  for (int i = 0; i < FFT_SIZE; i++) vReal[i] -= mean;

  // 3) FFT
  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  FFT.compute(FFTDirection::Forward);
  FFT.complexToMagnitude();

  // 4) Peak search in doppler band
  const int bin_min = (int)(F_MIN_HZ * FFT_SIZE / SAMPLE_RATE_HZ);
  const int bin_max = (int)(F_MAX_HZ * FFT_SIZE / SAMPLE_RATE_HZ);
  int    peak_bin = bin_min;
  double peak_mag = 0.0;
  double sum_mag  = 0.0;
  for (int i = bin_min; i < bin_max; i++) {
    if (vReal[i] > peak_mag) { peak_mag = vReal[i]; peak_bin = i; }
    sum_mag += vReal[i];
  }
  double noise = (sum_mag - peak_mag) / (double)(bin_max - bin_min - 1);
  double snr_db = 20.0 * log10((peak_mag + 1e-9) / (noise + 1e-9));

  if (snr_db < MIN_SNR_DB) return;
  if (peak_bin <= bin_min || peak_bin >= bin_max - 1) return;

  // 5) Parabolic interpolation for sub-bin accuracy
  double y0 = vReal[peak_bin - 1];
  double y1 = vReal[peak_bin];
  double y2 = vReal[peak_bin + 1];
  double denom = (y0 - 2.0 * y1 + y2);
  double delta = (fabs(denom) > 1e-9) ? 0.5 * (y0 - y2) / denom : 0.0;
  double refined_bin = (double)peak_bin + delta;
  double freq_hz = refined_bin * (double)SAMPLE_RATE_HZ / (double)FFT_SIZE;
  double v_mps = freq_hz / HZ_PER_MPS;

  // 6) Send
  send_event(t_capture, (float)v_mps, (float)peak_mag, (float)snr_db);
}
