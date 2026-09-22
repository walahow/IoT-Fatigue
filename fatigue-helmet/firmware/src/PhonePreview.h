// PhonePreview.h -- Wi-Fi access point + small web server so a phone can
// watch arming, see the camera with the ESP's eye box, place the eye box,
// run the optional blink test and press the session button.
// Spec: docs/superpowers/specs/2026-09-23-phone-arming-preview-design.md
//
// Knows nothing about sessions, EAR or sensors. main.cpp pushes a status JSON
// string (setStatus) and camera frames (offerFrame) in, and pulls command
// lines out (takeCommand), which loop() runs through the same
// handleCommandLine() as the serial port. Nothing here writes firmware state,
// so the phone can do nothing the serial port cannot.
#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include "esp_http_server.h"
#include "esp_heap_caps.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "phone_page.h"

#ifndef PHONE_AP_PASS
#error "PHONE_AP_PASS must be set in platformio.ini (WPA2, 8+ characters)"
#endif
static_assert(sizeof(PHONE_AP_PASS) - 1 >= 8 && sizeof(PHONE_AP_PASS) - 1 <= 63,
              "PHONE_AP_PASS must be 8-63 characters: set the HELMET_AP_PASS environment variable before building esp32s3cam_sd");

namespace PhonePreview {

static const size_t   FRAME_MAX       = 32 * 1024;  // QVGA q10 JPEGs run ~5-15 KB; bigger ones are skipped
static const size_t   STATUS_MAX      = 512;
static const size_t   CMD_MAX         = 32;         // same as g_serialLineBuf
static const uint32_t FRAME_WANTED_MS = 2000;       // the camera copies frames only while the page pulls them
static const uint32_t FRAME_STALE_MS  = 1000;       // older than this is not served (recording, camera stall)
// Calibration knob. The phone is < 1 m away: low power means less ripple on
// the rail the pulse sensor shares and less brownout risk on battery. Raise
// it only if the page keeps dropping.
static const wifi_power_t TX_POWER    = WIFI_POWER_8_5dBm;

static httpd_handle_t    s_server   = nullptr;
static SemaphoreHandle_t s_frameMux = nullptr;
static uint8_t          *s_frame    = nullptr;  // PSRAM; latest frame, written by the camera task
static uint8_t          *s_send     = nullptr;  // PSRAM; the server's own copy while it sends
static size_t            s_frameLen = 0;
static uint32_t          s_frameMs  = 0;        // millis() when s_frame was written
static volatile uint32_t s_wantedMs = 0;        // millis() of the last /frame.jpg request

static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;  // guards s_status and s_cmd*
static char s_status[STATUS_MAX] = "{}";
static char s_cmd[CMD_MAX];
static bool s_cmdPending = false;

static bool running() { return s_server != nullptr; }

// loop(), a few times a second.
static void setStatus(const char *json) {
  portENTER_CRITICAL(&s_mux);
  strncpy(s_status, json, STATUS_MAX - 1);
  s_status[STATUS_MAX - 1] = '\0';
  portEXIT_CRITICAL(&s_mux);
}

// loop(): the next command the page sent, if any, as a serial-style line.
static bool takeCommand(char *out, size_t n) {
  bool got = false;
  portENTER_CRITICAL(&s_mux);
  if (s_cmdPending) {
    strncpy(out, s_cmd, n - 1);
    out[n - 1] = '\0';
    s_cmdPending = false;
    got = true;
  }
  portEXIT_CRITICAL(&s_mux);
  return got;
}

// Camera task, every frame outside RECORDING. Never waits: if the server is
// mid-copy this frame is skipped rather than holding up the camera task.
static void offerFrame(const uint8_t *jpg, size_t len) {
  if (!s_server || len > FRAME_MAX) return;
  if (millis() - s_wantedMs > FRAME_WANTED_MS) return;   // nobody is looking
  if (xSemaphoreTake(s_frameMux, 0) != pdTRUE) return;
  memcpy(s_frame, jpg, len);
  s_frameLen = len;
  s_frameMs  = millis();
  xSemaphoreGive(s_frameMux);
}

// One slot: a second command before loop() takes the first is refused (409).
static bool queueCommand(const char *line) {
  bool ok = false;
  portENTER_CRITICAL(&s_mux);
  if (!s_cmdPending) {
    strncpy(s_cmd, line, CMD_MAX - 1);
    s_cmd[CMD_MAX - 1] = '\0';
    s_cmdPending = true;
    ok = true;
  }
  portEXIT_CRITICAL(&s_mux);
  return ok;
}

static esp_err_t replyQueued(httpd_req_t *req, bool ok) {
  if (!ok) {
    httpd_resp_set_status(req, "409 Conflict");
    return httpd_resp_sendstr(req, "busy");
  }
  return httpd_resp_sendstr(req, "ok");
}

// Whole-string decimal in [0, 2000) -- the range the serial ROI: path accepts.
static bool parseCoord(const char *s, int &v) {
  char *end = nullptr;
  long n = strtol(s, &end, 10);
  if (end == s || *end != '\0' || n < 0 || n >= 2000) return false;
  v = (int)n;
  return true;
}

static esp_err_t onPage(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_sendstr(req, PHONE_PAGE_HTML);
}

static esp_err_t onStatus(httpd_req_t *req) {
  char out[STATUS_MAX];
  portENTER_CRITICAL(&s_mux);
  memcpy(out, s_status, STATUS_MAX);
  portEXIT_CRITICAL(&s_mux);
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  return httpd_resp_sendstr(req, out);
}

static esp_err_t onFrame(httpd_req_t *req) {
  s_wantedMs = millis();
  size_t len = 0;
  if (xSemaphoreTake(s_frameMux, pdMS_TO_TICKS(100)) == pdTRUE) {
    if (s_frameLen && millis() - s_frameMs <= FRAME_STALE_MS) {
      memcpy(s_send, s_frame, s_frameLen);
      len = s_frameLen;
    }
    xSemaphoreGive(s_frameMux);
  }
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  if (len == 0) {   // nothing fresh (first request, or recording): the page retries
    httpd_resp_set_status(req, "204 No Content");
    return httpd_resp_send(req, nullptr, 0);
  }
  httpd_resp_set_type(req, "image/jpeg");
  return httpd_resp_send(req, (const char *)s_send, len);
}

// A browser sends Origin on every POST. Refuse one from any other site (a
// page open in another tab, reaching us while the phone is on this AP) so
// only this page can arm or move the eye box. curl sends none: allowed.
static bool fromOurPage(httpd_req_t *req) {
  char origin[40];
  esp_err_t e = httpd_req_get_hdr_value_str(req, "Origin", origin, sizeof origin);
  if (e == ESP_ERR_NOT_FOUND) return true;
  return e == ESP_OK && strcmp(origin, ("http://" + WiFi.softAPIP().toString()).c_str()) == 0;
}

// ?expect=<the state the page was showing>. loop() drops the press if the
// helmet has moved on since (phoneTick), so a stale ABORT label cannot stop
// a recording that just started, and a double tap cannot arm-then-abort.
static esp_err_t onPress(httpd_req_t *req) {
  if (!fromOurPage(req)) return httpd_resp_send_err(req, HTTPD_403_FORBIDDEN, "wrong origin");
  char q[32], st[12], line[CMD_MAX];
  if (httpd_req_get_url_query_str(req, q, sizeof q) != ESP_OK ||
      httpd_query_key_value(q, "expect", st, sizeof st) != ESP_OK ||
      (strcmp(st, "IDLE") != 0 && strcmp(st, "ARMING") != 0 && strcmp(st, "RECORDING") != 0))
    return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "expected ?expect=IDLE|ARMING|RECORDING");
  snprintf(line, sizeof line, "PRESS:%s", st);
  return replyQueued(req, queueCommand(line));
}

static esp_err_t onRoi(httpd_req_t *req) {
  if (!fromOurPage(req)) return httpd_resp_send_err(req, HTTPD_403_FORBIDDEN, "wrong origin");
  char q[48], xs[8], ys[8], line[CMD_MAX];
  int x = 0, y = 0;
  if (httpd_req_get_url_query_str(req, q, sizeof q) != ESP_OK)
    return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "expected ?x=&y= or ?auto");
  if (strcmp(q, "auto") == 0) {
    strcpy(line, "ROI:auto");
  } else if (httpd_query_key_value(q, "x", xs, sizeof xs) == ESP_OK &&
             httpd_query_key_value(q, "y", ys, sizeof ys) == ESP_OK &&
             parseCoord(xs, x) && parseCoord(ys, y)) {
    snprintf(line, sizeof line, "ROI:%d,%d", x, y);
  } else {
    return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "expected ?x=&y= or ?auto");
  }
  return replyQueued(req, queueCommand(line));
}

// Brings up the access point and the server. Idempotent. Blocks for a few
// hundred ms while Wi-Fi starts: call from setup(), or from loop() at IDLE.
static void start() {
  if (s_server) return;
  if (!s_frameMux) s_frameMux = xSemaphoreCreateMutex();
  if (!s_frame) s_frame = (uint8_t *)heap_caps_malloc(FRAME_MAX, MALLOC_CAP_SPIRAM);
  if (!s_send)  s_send  = (uint8_t *)heap_caps_malloc(FRAME_MAX, MALLOC_CAP_SPIRAM);
  if (!s_frameMux || !s_frame || !s_send) {
    Serial.println(F("#ERROR: Phone preview: no memory for frame buffers -- preview off"));
    return;
  }

  char ssid[16];
  const uint64_t mac = ESP.getEfuseMac();   // little-endian: bits 32-47 are the last two MAC bytes
  snprintf(ssid, sizeof ssid, "HELMET-%02X%02X",
           (unsigned)((mac >> 32) & 0xFF), (unsigned)((mac >> 40) & 0xFF));
  WiFi.persistent(false);  // this AP is brought up/down every ride; don't wear the NVS flash for it
  WiFi.mode(WIFI_AP);
  if (!WiFi.softAP(ssid, PHONE_AP_PASS)) {
    Serial.println(F("#ERROR: Phone preview: Wi-Fi AP failed to start"));
    WiFi.mode(WIFI_OFF);
    return;
  }
  // WiFi.setTxPower() returns false here: it waits on the async AP_STARTED
  // event bit, which hasn't been set yet this soon after softAP(). Set the
  // power directly instead -- WiFi.mode(WIFI_AP) above already ran
  // esp_wifi_start(), and wifi_power_t is already in the 0.25 dBm units
  // esp_wifi_set_max_tx_power() takes.
  if (esp_wifi_set_max_tx_power((int8_t)TX_POWER) != ESP_OK)
    Serial.println(F("#WARN: Phone preview: could not set Wi-Fi TX power"));
  int8_t txq = 0;
  esp_wifi_get_max_tx_power(&txq);

  httpd_config_t cfg      = HTTPD_DEFAULT_CONFIG();
  cfg.core_id             = 0;     // never loop()'s core 1: the 500 Hz pulse sampler lives there
  cfg.task_priority       = 1;     // below the camera task (2): the server only gets its slack
  cfg.lru_purge_enable    = true;  // a phone that walks off without closing must not pin sockets
  cfg.stack_size          = 6144;  // handlers stack a 512 B local, newlib printf, String temporaries in fromOurPage
  // A 15 KB frame over < 1 m needs far less than the 5 s default; bounds how
  // long stop() can block loop() if httpd_stop() catches a handler mid-send.
  cfg.send_wait_timeout   = 2;
  cfg.recv_wait_timeout   = 2;
  if (httpd_start(&s_server, &cfg) != ESP_OK) {
    s_server = nullptr;
    Serial.println(F("#ERROR: Phone preview: web server failed to start"));
    WiFi.mode(WIFI_OFF);
    return;
  }
  static const httpd_uri_t routes[] = {
    {"/",          HTTP_GET,  onPage,   nullptr},
    {"/status",    HTTP_GET,  onStatus, nullptr},
    {"/frame.jpg", HTTP_GET,  onFrame,  nullptr},
    {"/press",     HTTP_POST, onPress,  nullptr},
    {"/roi",       HTTP_POST, onRoi,    nullptr},
  };
  for (const httpd_uri_t &r : routes) httpd_register_uri_handler(s_server, &r);
  Serial.printf("#STATUS: Phone preview up -- join Wi-Fi %s, open http://%s "
                "(internal heap free %u, largest internal DMA block %u, tx %.1f dBm)\n",
                ssid, WiFi.softAPIP().toString().c_str(),
                (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA),
                txq / 4.0f);
}

// Server and Wi-Fi down, for the ride. Buffers stay allocated for the next start().
static void stop() {
  if (!s_server) return;
  httpd_handle_t h = s_server;
  s_server = nullptr;              // offerFrame() stops copying before the server goes
  httpd_stop(h);
  // WiFi.mode(WIFI_OFF) alone stops/deinits the AP and drops the phone;
  // softAPdisconnect(true) would also rewrite the (persistent-off) NVS config.
  WiFi.mode(WIFI_OFF);
  portENTER_CRITICAL(&s_mux);
  s_cmdPending = false;  // don't let a command queued just before stop() apply after the ride
  portEXIT_CRITICAL(&s_mux);
}

}  // namespace PhonePreview
