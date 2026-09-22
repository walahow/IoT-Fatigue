# Phone arming preview

Date: 2026-09-23
Status: approved

## Problem

On the bike the helmet runs `esp32s3cam_sd`. After the button press, arming
progress exists only as the `#ARMING:` serial line printed every 2 s, and the
helmet has no indicator (GPIO 2 is SDA, the buzzer is disconnected; see
`signalState()`). The rider cannot tell whether arming is still waiting on the
IMU, the HR baseline or the eye, or whether the eye box is even on the eye, so
the start of a session is a guess against the 60 s timeout.

The PC tool that shows this (`live_ear_preview.py --arm`) runs on a different
build (`esp32s3cam_ear_preview`, video over USB) and needs a laptop.

## Approach

The SD build brings up its own Wi-Fi access point and serves one page. The
phone joins it and opens `http://192.168.4.1`: live arming checklist, the
camera frame with the eye box, tap-to-set-eye, ARM/ABORT, and the PC tool's
blink test. Works in any phone browser; no app.

## Requirements

1. **The phone is an extra control, never a required one.** The GPIO 21
   button arms, aborts and stops exactly as today, in every state, with or
   without a phone connected and with Wi-Fi up or off. The page's ARM/ABORT
   is one more caller of the same `sessionButtonPress()`, alongside the button
   and the serial `BUTTON` command. While recording, Wi-Fi is off, so the
   button is the way to stop.
2. **The blink test is optional**, as on the PC (SPACE there, a button here).
   Arming and recording never wait for it or read its result.
3. **The page shows where the ESP put the eye box** — the ROI the firmware is
   actually using, whatever its source (motion lock, stored, manual), or
   "searching" while there is none.
4. **The rider can place the eye box by hand**, as clicking on the PC does:
   tap the eye to store the coordinate, **Auto** to clear it and let the
   firmware search again. Same `ROI:` path, same NVS storage, so a coordinate
   set from the phone and one set from the PC are the same thing. A new box
   outside recording restarts the firmware's eye check (3 blinks within 30 s
   of the lock) from that box. While recording, the phone cannot change it.

## Rejected alternatives


- **BLE text mirror** — no camera frame, and no Web Bluetooth on iOS.
- **ESP joins the phone's hotspot** — needs the phone's credentials in
  firmware and the rider has to find the ESP's IP each time.
- **LED on GPIO 47/48** — shows state but nothing about why arming is
  stuck, and a light near the eye can read as a glint.

## Build

`-DPHONE_PREVIEW` on `esp32s3cam_sd` only. Without the flag the build behaves
exactly as today's firmware, which is also the A/B for the bench checks.

| Setting | Value | Why |
|---|---|---|
| SSID | `HELMET-xxxx` (last two MAC bytes) | several helmets can coexist |
| Security | WPA2; password from the `HELMET_AP_PASS` env var at build time | the page can arm and move the ROI; the repo is public, so it is not committed |
| TX power | low (calibration knob) | phone is < 1 m away; less rail noise, less brownout risk |

Pulse is on ADC1, which stays usable with Wi-Fi on (ADC2 does not).

## Server

`esp_http_server` (ESP-IDF, bundled with Arduino-ESP32), which runs in its own
task. Arduino `WebServer` is not usable: its `handleClient()` would send JPEGs
from inside `loop()` and stall the 500 Hz pulse sampler.

| Endpoint | Does |
|---|---|
| `GET /` | the page (`src/phone_page.h`, one string, no external assets — the AP has no internet) |
| `GET /status` | JSON snapshot, rebuilt by `loop()` |
| `GET /frame.jpg` | latest camera frame; 204 when none is fresh (the camera only offers frames in IDLE and ARMING) |
| `POST /roi?x=&y=` / `POST /roi?auto` | set / clear the stored eye coordinate |
| `POST /press?expect=<state>` | one button press, dropped by `loop()` if the helmet is no longer in the state the page showed (stale label, double tap) |

`POST`s carrying an `Origin` header other than the page's own are refused
(403), so another site open on the phone cannot arm or move the eye box.

`/status` fields: `state`, `arm_s`, `arm_timeout_s`, `imu`, `hr`, `hr_n`,
`hr_need`, `eye_check` (pending/pass/fail), `blinks_since_lock`,
`blinks_need`, `roi` (`x`, `y`, `size` or null), `roi_src`, `roi_conf`,
`hog_total`, `timed_out`.

Frames are pulled, not streamed: the page requests the next `/frame.jpg` only
after the previous one has loaded (~3 fps cap). That throttles itself to the
link and stops when the page closes. An MJPEG multipart stream holds a server
worker open and has no backpressure.

## Threading

This is the part the chip is sensitive to.

- **Frames.** The camera task copies the current JPEG (QVGA, ~15 KB) into a
  PSRAM buffer under a mutex, and only if `/frame.jpg` was requested in the
  last 2 s. The server never calls `esp_camera_fb_get()` itself; with
  `fb_count = 2` that would steal frames from the EAR pipeline.
- **Commands.** Handlers never write arming or EAR state. `/roi` and `/press`
  set a pending command; `loop()` applies it on its next pass through the same
  code as the serial `ROI:` and `BUTTON` commands. The serial `ROI:` branch in
  `loop()` becomes one function both paths call, so the phone can do nothing
  the serial port cannot, and every state write stays on `loop()`'s task.
  Coordinates are range-checked exactly as the serial path does.
- **Status.** `loop()` rebuilds the JSON into a buffer at 4 Hz under a
  critical section; the handler copies it out.
- **Core.** The server task runs on core 0 at priority 1 — below the camera
  task (2), so it only takes the camera task's slack, and never on `loop()`'s
  core 1. The Wi-Fi driver and lwIP tasks are on core 0 regardless.

`hog_total`: a new free-running `uint32_t`, incremented next to
`g_earBlinksSinceLock++` in `processEarFrame()`. `g_earBlinksSinceLock`
cannot serve: it saturates at 255 and resets at every lock. The page takes
deltas of `hog_total` for the blink flash and the blink test.

## Wi-Fi lifecycle

    boot --> AP up
    IDLE / ARMING        : status + frames
    RECORDING, first 15 s: status only (page shows RECORDING)
    RECORDING, after 15 s: server stopped, Wi-Fi off
    sessionStop() --> AP up again

Off during the ride keeps recording identical to today: no extra load on the
core that runs SD writes and EAR, no SD timing change, no power cost, no
radio next to the head. Turning the AP off blocks `loop()` for at least
100 ms (`httpd_stop`'s own wait; up to ~2 s if the server is mid-send, bounded
by 2 s send/recv timeouts) — 50+ pulse samples, once per session at the 15 s
mark, and the duration is logged. Turning it back on happens at IDLE, where
nothing records. Acceptable — move `stop()` to a one-shot task if the gap
shows in the data. Wi-Fi config is not persisted, so neither step writes flash.

## Page

- **Banner**: `IDLE` · `ARMING 23 / 60 s` · `RECORDING` · `RECORDING (arming timed out)`.
- **Checklist**: IMU calibrated · HR baseline n / 30 (`HrBaseline::NEEDED`) · eye ROI
  (source, confidence) · eye check blinks n / 3 · classifier crop inside frame.
- **Frame**: rotated 90° CCW, as the PC tool displays it; green ROI box,
  yellow classifier crop box (`hogCropRect` mirror), red flash when
  `hog_total` rises.
- **Controls**: tap the eye → confirm → `POST /roi`; **Auto** clears it;
  **ARM / ABORT** posts `/press`; **Blink test**.
- Polls `/status` at 1 Hz, 4 Hz while a blink test runs.

### Blink test

A port of `live_ear_preview.py --arm`, same constants. Runs whenever frames
are served and the ROI is locked — meant for IDLE, after the eye is set and
before ARM. A switch to RECORDING mid-test abandons it.

| Phase | Duration | Rider | Pass |
|---|---|---|---|
| still | 8 s | hold still, eyes open | ≤ 2 detections |
| blink | 20 s | blink 10 times | ≥ 8 detections |

Detections are `hog_total` deltas across each phase. At the 4 Hz poll a blink
within ~250 ms of the phase boundary can land in the wrong phase; negligible
against 8 s and 20 s phases.

The verdict, READY TO RECORD or NOT READY with the failing check's hint,
combines eye ROI (stored/manual, or motion lock conf ≥ 2.0), crop inside
frame, and both phases — the PC tool's checks minus exposure, which the rider
can judge from the live frame. It is advisory: it does not gate ARM. The
firmware's own eye check (3 blinks in 30 s) is still what decides
`blink_valid`.

Constants and `hogCropRect` are duplicated from Python/C++ into the page's
JS; each copy names its source.

## Checks

On the bench, before a ride:

1. **EAR rate with Wi-Fi.** `-DEAR_PROFILE` build: `ear=` and processed
   frames/s during ARMING with the page open vs closed. Wi-Fi tasks share
   core 0 with EAR; fewer frames means a slower motion lock and missed blinks.
   Knob: the page's frame poll rate.
2. **Pulse noise.** Arm 3× with and 3× without `PHONE_PREVIEW`; compare
   baseline BPM and `pulse_raw` spread. Knob: TX power.
3. **Lifecycle.** Arm → record → stop: Wi-Fi off at 15 s, `Wrote N frames`
   matches a no-flag build over the same duration, AP back at IDLE.
4. **Memory.** Flash/RAM from `pio run -e esp32s3cam_sd`; free internal heap
   (`heap_caps_get_free_size(MALLOC_CAP_INTERNAL)`) with the AP up.

In code: one runnable check for the tap → native-coordinate mapping (the
inverse of the display rotation), the piece easiest to get silently wrong.

## Out of scope

- Status or frames during the ride.
- Exposure check.
- Replacing `live_ear_preview.py` — it stays the tool for the preview build.
