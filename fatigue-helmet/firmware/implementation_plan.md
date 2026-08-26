# Alert State Machine & EMI Avoidance Plan

This plan addresses the MPU-6050 I2C bus crashes caused by buzzer EMI. We will implement the user's proposed "Timed Alert + Cooldown" state machine.

## Background & Rationale
Currently, the firmware triggers a **continuous** buzzer when the system enters a `CRITICAL` fatigue state. Because the buzzer creates electromagnetic interference (EMI), it crashes the I2C bus, freezing the MPU-6050.
By moving to a timed state machine, we can:
1. Sound the buzzer for a fixed duration (e.g., 2 seconds).
2. Pause all I2C reads from the MPU-6050 during those 2 seconds (avoiding the crash entirely).
3. Enforce a cooldown period (e.g., 3 seconds) where the buzzer is forced OFF and the MPU resumes reading. This allows the system to gather fresh data, evaluating if the user has woken up and stabilized.
4. If the user is still in a critical state after the cooldown, the cycle repeats.

## Proposed Changes

### `main.cpp`

#### 1. Global Buzzer State
- Introduce a global variable `bool g_buzzerActive = false;` to easily track when the buzzer is physically emitting sound. This will encapsulate both the start/stop session beeps and the fatigue alerts.

#### 2. MPU I2C Suspension
- Wrap the MPU read block (around line 1012) with a check: `if (g_mpuEnabled && !g_buzzerActive)`.
- When `g_buzzerActive` is true, the firmware will simply skip `Wire.beginTransmission()` and `mpu.getMotion6()`, preserving the I2C bus integrity.

#### 3. Alert State Machine
- Replace the current continuous buzzer logic (around line 1150) with a non-blocking state machine using `millis()`.
- **States:**
  - `IDLE`: Checks if `g_alertLevel == ALERT_CRITICAL`. If so, transitions to `ALERT`.
  - `ALERT`: Sets `g_buzzerActive = true` and `digitalWrite(BUZZER_PIN, HIGH)`. Lasts for a set duration (e.g., 2000 ms).
  - `COOLDOWN`: Sets `g_buzzerActive = false` and `digitalWrite(BUZZER_PIN, LOW)`. Lasts for a set duration (e.g., 3000 ms), allowing the MPU to stabilize and provide fresh data to the Fuzzy logic system.

## Open Questions

> [!IMPORTANT]
> **Durations:** How long do you want the actual buzzer to sound (Alert Duration) and how long should the system wait before allowing another alert (Cooldown Duration)? 
> *Recommendation:* 2 seconds ON, followed by 3 seconds OFF (Cooldown). Does this sound good to you?

> [!NOTE]
> **Warning Level Beep:** The code mentions a `WARNING` level alert (a single 200ms beep every 3 seconds), but it looks like it was commented out or disabled (`setBuzzerState(false)`). Should we also implement the timed/cooldown logic for the `WARNING` level, or keep it strictly for `CRITICAL`?

## Verification Plan
1. Compile and flash the firmware.
2. Induce a critical alert and verify through the serial monitor that the buzzer runs for X seconds, then turns off for Y seconds.
3. Verify that `mpu.getMotion6()` is skipped when the buzzer is active, preventing the I2C error counter from rising.
4. Ensure the system can naturally exit the critical state if the sensor data normalizes during the cooldown phase.
