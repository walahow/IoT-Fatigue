# Record-ready session gating

Date: 2026-09-08
Status: approved

## Problem

The button toggles `g_sessionActive` instantly. Recording begins whether or not
the eye ROI is locked, the HR baseline exists, or the IMU is calibrated for the
current pose, so a session can be captured that is unusable for training.

Three further defects sit on the same path:

- `closeSession()` is defined but never called. Stopping clears a flag; the CSV,
  MJPEG and IDX files are never flushed or closed, so pulling the card can lose
  the tail of a recording.
- The session folder and its three files are created at boot inside
  `initSDCard()`, binding a "session" to a power cycle rather than to the button.
- The SD-error handler drives `LED_BUILTIN` (GPIO 2), which is the IMU's SDA
  line, twenty lines below the comment warning never to do that.

## Flow

    IDLE --press--> ARMING --all ready | timeout--> RECORDING --press--> IDLE
                      |                                         (closeSession)
                      +--press = abort--> IDLE

`g_sessionActive` is retained as a derived flag (`state == RECORDING`) so the
five existing consumers need no change.

## ARMING

Three checks run concurrently. Each is reset on entry so every session gets its
own calibration rather than inheriting whatever the helmet's pose was at boot.

| Check | Reset on entry | Ready when |
|---|---|---|
| IMU calibration | offsets cleared | 200 samples at 100 Hz |
| HR baseline | sum, count, formed cleared | `g_baselineFormed` (20 valid 1 Hz samples) |
| Eye ROI lock | `g_earLockDone`, ROI, locator | `g_earLockDone` |

The IMU calibration is rewritten rather than reused. `calibrateMPU()` blocks for
about 4 s (a 3 s countdown plus 200 x `delay(5)`); calling it from `loop()` would
starve the 500 Hz pulse sampler and the watchdog feed. The replacement
accumulates from the loop cadence and restarts its accumulation if gyro
magnitude spikes, because a calibration taken while the helmet moves is garbage.

## Timeout

`ARMING_TIMEOUT_MS = 60000`. On expiry recording starts regardless, and
per-subsystem readiness is written to `metadata.txt` (`armed_imu`, `armed_hr`,
`armed_eye`, `arming_duration_ms`, `arming_timed_out`) so the session can be
judged afterward.

## Session rotation

Session creation moves out of `initSDCard()` (which keeps only the mount) into
`openSession()`, called when RECORDING begins. Each arm-record-stop cycle writes
its own `session_XXX`, preventing two recordings from being appended into one
file.

## Feedback

A single `signalState()` function, serial-only for now. GPIO 2 is unusable as an
indicator (it is SDA) and the buzzer is disconnected, so a hardware indicator is
deferred; `signalState()` is the one place to add it.

## Out of scope

Hardware status indicator (LED on a free GPIO, or reconnecting the buzzer).
