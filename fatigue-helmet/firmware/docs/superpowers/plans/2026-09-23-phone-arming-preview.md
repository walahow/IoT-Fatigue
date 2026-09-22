# Phone Arming Preview Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Let the rider watch arming, see the camera with the ESP's eye box, place the eye box by tap, run the optional blink test, and press the session button, from a phone browser over the helmet's own Wi-Fi.

**Architecture:** The `esp32s3cam_sd` build, with `-DPHONE_PREVIEW`, starts a WPA2 access point and an `esp_http_server` on core 0 at priority 1. `PhonePreview.h` knows nothing about sessions. `main.cpp` pushes a status JSON and camera frames into it and pulls command lines out of it. `loop()` runs those commands through the same `handleCommandLine()` the serial port uses, so the phone cannot do anything the serial port can't. The page is a single HTML string. Its pure logic is checked under node. Wi-Fi turns off 15 s into RECORDING and comes back at IDLE.

**Tech Stack:** Arduino-ESP32 2.0.17 (ESP-IDF 4.4) on PlatformIO, `esp_http_server`, `WiFi.h`, vanilla JS/canvas, node (`node:assert`, `node:vm`) for the page check.

**Spec:** `docs/superpowers/specs/2026-09-23-phone-arming-preview-design.md`

All paths below are relative to `fatigue-helmet/firmware/`, and all commands run from there.

---

## File structure

| File | Status | Responsibility |
|---|---|---|
| `src/main.cpp` | modify | `handleCommandLine()` (serial + phone), `g_earHogBlinkTotal`, frame hand-off in `cameraTask`, `buildPhoneStatus()` + `phoneTick()`, `PhonePreview::start()` in `setup()` |
| `src/PhonePreview.h` | create | AP + web server, frame/status/command hand-off buffers. No firmware state. |
| `src/phone_page.h` | create | The page: HTML/CSS/JS in one raw string. Pure functions at the top of its `<script>`. |
| `tools/phone_page_test.js` | create | Runs the page's pure functions under node: tap mapping, crop mirror, blink test, verdict. |
| `platformio.ini` | modify | `-DPHONE_PREVIEW`, `-DPHONE_AP_PASS` on `esp32s3cam_sd` |
| `../README.md`, `../../CLAUDE.md` | modify | How to use it |

---

### Task 1: One command path for serial and phone, plus a blink counter that never resets

This is a pure refactor plus one counter; behaviour doesn't change. The serial parser's per-line handling moves into `handleCommandLine()`, so the phone (Task 4) can reuse it word for word. `g_earHogBlinkTotal` is the counter the page takes deltas of. `g_earBlinksSinceLock` can't do that job: it saturates at 255 and resets at every lock.

**Files:**
- Modify: `src/main.cpp:444` (after `g_earBlinksSinceLock`), `src/main.cpp:837` (classifier blink), `src/main.cpp:2285-2344` (serial parser), and a new function just above `void loop()`

- [ ] **Step 1: Add the counter**

In `src/main.cpp`, directly below the line
`volatile uint8_t g_earBlinksSinceLock = 0;   // classifier blinks since the ROI lock (saturates at 255)` add:

```cpp
volatile uint32_t g_earHogBlinkTotal = 0;    // classifier blinks since boot, never reset -- the phone page takes deltas
```

- [ ] **Step 2: Count every classifier blink**

In `processEarFrame()`, change

```cpp
    if (g_earBlinksSinceLock < 255) g_earBlinksSinceLock++;
    earPrintf("#STATUS: blink (hog) t=%u score=%.2f\n", timestampMs, hogScore);
```

to

```cpp
    if (g_earBlinksSinceLock < 255) g_earBlinksSinceLock++;
    g_earHogBlinkTotal++;
    earPrintf("#STATUS: blink (hog) t=%u score=%.2f\n", timestampMs, hogScore);
```

- [ ] **Step 3: Add `handleCommandLine()` directly above `void loop() {`**

Insert it right after the closing `}` of `setup()`, i.e. immediately before the comment banner that begins `// Main loop — Core 1`:

```cpp
// ─────────────────────────────────────────────────────────────────────────
// handleCommandLine() — one text command, from the serial port (loop()'s 1 Hz
// parser) or from the phone page (phoneTick()). Both sources go through here,
// so the phone can do nothing the serial port cannot, and the two cannot
// drift apart. Call from loop()'s task only: it arms, stops and moves the ROI.
// ─────────────────────────────────────────────────────────────────────────
void handleCommandLine(const char *line, unsigned long now) {
  // "BUTTON" is one press of the GPIO 21 button, routed through the same
  // sessionButtonPress() dispatch. Lets the arm/record/stop flow be driven
  // over USB or from the phone without reaching into the helmet, and cannot
  // collide with the BLINK: format below.
  if (strcmp(line, "BUTTON") == 0) {
    sessionButtonPress();
    return;
  }
#if defined(STORAGE_MODE_SD) || defined(EAR_LIVE_DEBUG)
  // "ROI:<cx>,<cy>" stores the eye centre the PC or phone measured (kept in
  // NVS across reboots and reflashes) and applies it immediately; "ROI:auto"
  // clears it and goes back to searching. See earStoredRoi* and
  // live_ear_preview.py --arm.
  if (strncmp(line, "ROI:", 4) == 0) {
    int cx = 0, cy = 0;
    if (strcmp(line + 4, "auto") == 0) {
      earClearStoredRoi();
      g_earLockDone = false;
      g_earRoi.locked = false;
      g_earRoiSource = "none";
      g_earLocator.reset();
      g_earDriftState = EAR_DRIFT_IDLE;   // g_earLocator is about to be reused for the boot search
      g_earMotionTries = 0;
      Serial.println(F("#STATUS: EAR ROI cleared -- searching for the eye again"));
    } else if (sscanf(line + 4, "%d,%d", &cx, &cy) == 2 &&
               cx >= 0 && cy >= 0 && cx < 2000 && cy < 2000) {
      earSaveStoredRoi(cx, cy);
      g_earLockDone = false;      // re-applied from the stored value next frame
      g_earRoi.locked = false;
      g_earDriftState = EAR_DRIFT_IDLE;   // the ROI is about to jump; a cycle mid-accumulation is now stale
      Serial.printf("#STATUS: EAR ROI saved cx=%d cy=%d\n", cx, cy);
    } else {
      Serial.println(F("#ERROR: expected ROI:<cx>,<cy> or ROI:auto"));
    }
    return;
  }
#endif
  float blink_val = 0.0f;
  if (sscanf(line, "BLINK:%f", &blink_val) == 1
      && blink_val >= 0.0f && blink_val <= 60.0f) {
    g_blinkRate       = blink_val;
    g_lastValidBlink  = blink_val;
    g_lastBlinkRxTime = now;
    g_blinkEverRx     = true;
  }
}

```

- [ ] **Step 4: Make the serial parser call it**

In `loop()`, replace the whole `while (Serial.available()) { ... }` block (from `while (Serial.available()) {` through its closing `}` just before `#endif  // !FRAME_INJECT_MODE`) with:

```cpp
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      g_serialLineBuf[g_serialLineBufLen] = '\0';
      handleCommandLine(g_serialLineBuf, now);
      g_serialLineBufLen = 0;
    } else if (g_serialLineBufLen < (uint8_t)(sizeof(g_serialLineBuf) - 1)) {
      g_serialLineBuf[g_serialLineBufLen++] = c;
    } else {
      g_serialLineBufLen = 0;   // line too long — discard
    }
  }
```

Keep the comment above it (`// ── Serial BLINK parser ...`) and the surrounding `#if !defined(FRAME_INJECT_MODE)` / `#endif` unchanged.

- [ ] **Step 5: Build every ESP environment that compiles `main.cpp`**

Run: `pio run -e esp32s3cam_sd -e esp32s3cam -e esp32s3cam_ear_preview -e esp32s3cam_frame_inject`
Expected: all four report `SUCCESS`. The summary table lists each with `SUCCESS`.

- [ ] **Step 6: Host tests still pass**

Run: `pio test -e native`
Expected: all suites `PASSED` (they don't include `main.cpp`; this catches accidental header edits).

- [ ] **Step 7: Commit**

```bash
git add src/main.cpp
git commit -m "Route serial commands through handleCommandLine(), add g_earHogBlinkTotal

No behaviour change. Prepares a second caller (the phone page) that must
take exactly the serial path, and a blink counter that is never reset."
```

---

### Task 2: The page, test first

> **Superseded after review:** the code below is the first cut (commit `c605cb3f`). Two review rounds followed (`83f9e1f3`, `5f2a756f`). They added fetch timeouts and `/press?expect=`. A finished verdict is now cleared only by a real eye-box move (`withRoi`/`staleTest`, with drift tolerated), and the test's baseline comes from the first status after the tap. `verdict(checks, t)` changed signature, timing uses `performance.now()`, and the prompt/verdict became one element above a height-capped picture, plus boundary tests. The committed `src/phone_page.h` and `tools/phone_page_test.js` are the reference.

The page is one raw-string header, so the firmware needs no filesystem and no build step. The pure functions at the top of its `<script>` are the parts that are easy to get silently wrong: turning a tap into camera coordinates, the classifier-crop mirror, and the blink-test scoring. The node check extracts that script and runs it.

**Files:**
- Create: `tools/phone_page_test.js`
- Create: `src/phone_page.h`

- [ ] **Step 1: Write the failing test**

Create `tools/phone_page_test.js`:

```js
// Checks the pure logic in src/phone_page.h's <script>: the tap -> camera
// coordinate map, the classifier-crop mirror, and the blink test / verdict.
// Run from fatigue-helmet/firmware:  node tools/phone_page_test.js
'use strict';
const fs = require('fs');
const path = require('path');
const vm = require('vm');
const assert = require('assert');

const src = fs.readFileSync(path.join(__dirname, '..', 'src', 'phone_page.h'), 'utf8');
const html = src.match(/R"HTML\(([\s\S]*)\)HTML"/)[1];
const script = html.match(/<script>([\s\S]*)<\/script>/)[1];
const P = vm.runInNewContext(script +
  '\n;({displayToNative, hogCropRect, cropMargin, blinkTestStart, blinkTestStep, preflightChecks, verdict})', {});

// Values made inside the vm have that realm's prototypes; compare plain copies.
const eq = (actual, expected) => assert.deepStrictEqual(JSON.parse(JSON.stringify(actual)), expected);

// ── Tap mapping. Rotating 90 deg CCW (live_ear_preview.py's display) sends
// the native top-right corner to the display's top-left, native top-left to
// display bottom-left, native bottom-right to display top-right. The display
// is H wide and W tall.
const W = 320, H = 240;
eq(P.displayToNative(0.5, 0.5, W, H), [W - 1, 0]);
eq(P.displayToNative(0.5, W - 0.5, W, H), [0, 0]);
eq(P.displayToNative(H - 0.5, 0.5, W, H), [W - 1, H - 1]);
// Every pixel of a small frame comes back to itself through the forward map
// the canvas transform uses: native (x, y) -> display (y, W - x).
for (let x = 0; x < 5; x++) for (let y = 0; y < 3; y++)
  eq(P.displayToNative(y + 0.5, 5 - (x + 0.5), 5, 3), [x, y]);
// Taps past the edge clamp inside the frame.
eq(P.displayToNative(-3, -3, W, H), [W - 1, 0]);
eq(P.displayToNative(H + 3, W + 3, W, H), [0, H - 1]);

// ── Crop mirror, checked by hand against EyeBlinkEAR::hogCropRect():
// size 48 -> w = 48*15/8 = 90, h = 180, x = 136+24-45 = 115, y = 96+24-90 = 30.
eq(P.hogCropRect({ x: 136, y: 96, size: 48 }), { x: 115, y: 30, w: 90, h: 180 });
// size 50 -> w = 750/8 = 93 (truncated), h = 186, x = 100+25-46, y = 100+25-93.
eq(P.hogCropRect({ x: 100, y: 100, size: 50 }), { x: 79, y: 32, w: 93, h: 186 });
assert.strictEqual(P.cropMargin({ x: 136, y: 96, size: 48 }, W, H), 30);
assert.ok(P.cropMargin({ x: 136, y: 10, size: 48 }, W, H) < 0);   // hangs off the top

// ── Blink test: 8 s still, then 20 s of cued blinking; counts are hog_total deltas.
let t = P.blinkTestStart(0, 100);
t = P.blinkTestStep(t, 4, 101);
assert.strictEqual(t.phase, 'still');
assert.strictEqual(t.count, 1);
t = P.blinkTestStep(t, 8, 101);                  // still phase over: 1 false alarm
assert.strictEqual(t.phase, 'blink');
assert.strictEqual(t.stillFalse, 1);
assert.strictEqual(t.count, 0);
t = P.blinkTestStep(t, 20, 109);
assert.strictEqual(t.phase, 'blink');
assert.strictEqual(t.count, 8);
t = P.blinkTestStep(t, 28, 110);                 // blink phase over: 9 detected
assert.strictEqual(t.phase, 'done');
assert.strictEqual(t.blinkDetected, 9);
assert.strictEqual(P.blinkTestStep(t, 99, 200), t);                        // done is final
assert.strictEqual(P.blinkTestStep(P.blinkTestStart(0, 50), 1, 3), null);  // counter went back: ESP rebooted

// ── Verdict: READY only once every check is judged and passes (the PC tool's rule).
const status = { roi: { x: 136, y: 96, size: 48 }, roi_src: 'motion', roi_conf: 2.4 };
assert.strictEqual(P.verdict(P.preflightChecks(status, t, W, H)), true);
assert.strictEqual(P.verdict(P.preflightChecks(status, null, W, H)), null);   // test not run: no verdict
assert.strictEqual(P.verdict(P.preflightChecks({ ...status, roi_conf: 1.5 }, t, W, H)), false);
assert.strictEqual(P.verdict(P.preflightChecks({ ...status, roi_src: 'stored', roi_conf: -1 }, t, W, H)), true);
assert.strictEqual(P.verdict(P.preflightChecks({ ...status, roi: { x: 136, y: 10, size: 48 } }, t, W, H)), false);
assert.strictEqual(P.verdict(P.preflightChecks(status, { ...t, stillFalse: 3 }, W, H)), false);
assert.strictEqual(P.verdict(P.preflightChecks(status, { ...t, blinkDetected: 7 }, W, H)), false);

console.log('phone_page: all checks passed');
```

- [ ] **Step 2: Run it and watch it fail**

Run: `node tools/phone_page_test.js`
Expected: FAIL with `Error: ENOENT: no such file or directory, open '...src\phone_page.h'`

- [ ] **Step 3: Write the page**

Create `src/phone_page.h`:

```cpp
// phone_page.h -- the page PhonePreview.h serves at "/". One file, no
// external assets: the helmet's access point has no internet.
//
// The top of its <script> holds pure functions (tap mapping, crop mirror,
// blink test) that tools/phone_page_test.js runs under node -- keep them free
// of DOM access. Everything that touches the page lives in boot().
// Must not contain the raw-string terminator )HTML" anywhere.
#pragma once

static const char PHONE_PAGE_HTML[] = R"HTML(<!doctype html>
<html lang="en"><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Helmet arming</title>
<style>
:root { color-scheme: dark; --ok:#3ecf6a; --bad:#ff5a5a; --wait:#9aa0a6; --warn:#ffb020; --bg:#111; --fg:#eee; --panel:#1c1c1c; }
* { box-sizing: border-box; }
body { margin:0; padding:12px 16px 32px; background:var(--bg); color:var(--fg); font:16px/1.4 system-ui, sans-serif; }
#banner { font-size:28px; font-weight:700; text-align:center; padding:10px; border-radius:10px; background:var(--panel); margin-bottom:10px; }
#banner.arming { color:var(--warn); }
#banner.rec { color:var(--ok); }
#banner.lost { color:var(--bad); }
canvas { width:100%; display:block; border-radius:8px; background:#000; touch-action:manipulation; }
#eye { color:var(--wait); margin:6px 0 10px; font-size:14px; }
.row { display:flex; gap:8px; margin-bottom:8px; }
button { flex:1; font-size:17px; font-weight:600; padding:14px 8px; border-radius:10px; border:0; background:#2d2d2d; color:var(--fg); }
button:disabled { opacity:.4; }
#arm { background:#1f4d2e; }
#arm.abort { background:#5a3a12; }
#arm.stop { background:#6a1b1b; }
h2 { font-size:14px; text-transform:uppercase; letter-spacing:.06em; color:var(--wait); margin:16px 0 6px; }
ul { list-style:none; margin:0; padding:0; }
li { padding:6px 0; border-bottom:1px solid #262626; }
li b { display:inline-block; width:3.4em; }
li .hint { display:block; color:var(--wait); font-size:13px; margin-left:3.4em; }
.ok b { color:var(--ok); }
.bad b { color:var(--bad); }
.wait b { color:var(--wait); }
#prompt { font-size:22px; font-weight:700; text-align:center; color:var(--warn); margin:8px 0; min-height:1.4em; }
#verdict { font-size:22px; font-weight:700; text-align:center; margin:8px 0; }
#note { color:var(--warn); font-size:14px; min-height:1.2em; text-align:center; }
</style></head>
<body>
<div id="banner">CONNECTING...</div>
<canvas id="cv" width="240" height="320"></canvas>
<div id="eye"></div>
<div class="row">
  <button id="arm">ARM</button>
  <button id="auto">Auto eye</button>
  <button id="test">Blink test</button>
</div>
<div id="note"></div>
<h2>Arming (the helmet waits for these)</h2>
<ul id="arming"></ul>
<h2>Pre-flight blink test (optional)</h2>
<div id="prompt"></div>
<ul id="preflight"></ul>
<div id="verdict"></div>
<script>
'use strict';
// ── Mirrors of firmware / PC-tool constants: keep in step with the source named.
const MIN_LOCK_CONF = 2.0;       // main.cpp EAR_MOTION_MIN_CONF
const STILL_SECONDS = 8;         // live_ear_preview.py --arm, phase 1: hold still, eyes open
const BLINK_SECONDS = 20;        // phase 2: blink on cue
const BLINK_TARGET = 10;
const BLINK_MIN_DETECTED = 8;
const STILL_MAX_FALSE = 2;
const FRAME_PERIOD_MS = 333;     // ~3 fps cap on /frame.jpg -- raise it if EAR slows with the page open
const LOST_MS = 3000;            // no /status reply this long = connection lost

// ── Pure logic (tools/phone_page_test.js) ────────────────────────────────────
// The native frame (W x H) is shown rotated 90 deg CCW, as live_ear_preview.py
// shows it: native (x, y) -> display (y, W - x), in continuous coordinates.
// Drawing applies that map as a canvas transform; a tap needs the inverse.
function displayToNative(px, py, W, H) {
  const x = Math.floor(W - py), y = Math.floor(px);
  return [Math.min(W - 1, Math.max(0, x)), Math.min(H - 1, Math.max(0, y))];
}

// Mirror of EyeBlinkEAR::hogCropRect() -- the crop the classifier sees.
// Math.trunc stands in for C++ integer division.
function hogCropRect(roi) {
  const w = Math.trunc(roi.size * 15 / 8), h = 2 * w;
  return { x: roi.x + Math.trunc(roi.size / 2) - Math.trunc(w / 2),
           y: roi.y + Math.trunc(roi.size / 2) - Math.trunc(h / 2), w: w, h: h };
}

// Pixels between the classifier crop and the nearest frame edge; < 0 = it hangs off.
function cropMargin(roi, W, H) {
  const c = hogCropRect(roi);
  return Math.min(c.x, c.y, W - (c.x + c.w), H - (c.y + c.h));
}

// Blink test, a port of live_ear_preview.py --arm. Detections are deltas of the
// firmware's hog_total. Optional: nothing in the firmware waits on it.
function blinkTestStart(nowS, hog) {
  return { phase: 'still', start: nowS, base: hog, count: 0, stillFalse: null, blinkDetected: null };
}

// -> the updated test, or null if the counter went backwards (the ESP rebooted).
function blinkTestStep(t, nowS, hog) {
  if (!t || t.phase === 'done') return t;
  if (hog < t.base) return null;
  const count = hog - t.base;
  if (t.phase === 'still' && nowS - t.start >= STILL_SECONDS)
    return Object.assign({}, t, { phase: 'blink', start: nowS, base: hog, count: 0, stillFalse: count });
  if (t.phase === 'blink' && nowS - t.start >= BLINK_SECONDS)
    return Object.assign({}, t, { phase: 'done', count: count, blinkDetected: count });
  return Object.assign({}, t, { count: count });
}

// The PC tool's checks, minus exposure (the rider sees the live picture).
// -> [{ok, label, hint}]; ok null = cannot be judged yet.
function preflightChecks(s, t, W, H) {
  const roi = s ? s.roi : null;
  const given = !!s && (s.roi_src === 'stored' || s.roi_src === 'manual');
  const out = [{
    ok: roi ? (given || s.roi_conf >= MIN_LOCK_CONF) : false,
    label: !roi ? 'eye box: searching' :
           given ? 'eye box ' + s.roi_src :
           'eye box motion-locked (confidence ' + s.roi_conf.toFixed(1) + ', needs ' + MIN_LOCK_CONF + ')',
    hint: 'tap the eye to set it, or blink while it searches' }];
  const m = roi && W ? cropMargin(roi, W, H) : null;
  out.push({
    ok: m === null ? null : m >= 0,
    label: m === null ? 'classifier crop in frame (waiting)' : 'classifier crop in frame (margin ' + m + ' px)',
    hint: 'move the camera so the yellow box sits fully inside the picture' });
  if (t && t.phase === 'done') {
    out.push({ ok: t.blinkDetected >= BLINK_MIN_DETECTED,
      label: 'blink test: ' + t.blinkDetected + ' of ' + BLINK_TARGET + ' detected',
      hint: 'the camera sees the eye but not the lids -- reposition, or retrain for this rider' });
    out.push({ ok: t.stillFalse <= STILL_MAX_FALSE,
      label: 'false alarms while still: ' + t.stillFalse,
      hint: 'detections with the eye open: reposition, or retrain for this rider' });
  } else {
    out.push({ ok: null, label: 'blink test: tap Blink test to run it', hint: '' });
  }
  return out;
}

// READY only once every check has been judged and passed.
function verdict(checks) {
  if (checks.some(c => c.ok === null)) return null;
  return checks.every(c => c.ok);
}

// The firmware's own arming gate, as /status reports it. The IMU and HR
// checks only run during ARMING, so outside it they show as waiting.
function armingRows(s) {
  const arming = s.state === 'ARMING';
  const eye = s.eye_check === 'pass'
      ? { ok: true, label: 'eye check passed (' + s.blinks_since_lock + ' blinks after lock)' }
    : s.eye_check === 'fail'
      ? { ok: false, label: 'eye check failed -- blink channel off this session',
          hint: 'arming carries on; set the eye and re-arm to get it back' }
    : { ok: null, label: s.roi ? 'eye check: ' + s.blinks_since_lock + ' / ' + s.blinks_need + ' blinks after lock'
                               : 'eye check: waiting for the eye box' };
  return [
    { ok: arming ? !!s.imu : null,
      label: arming ? 'IMU calibrated' + (s.imu ? '' : ' (hold still)') : 'IMU calibration (starts at ARM)' },
    { ok: arming ? !!s.hr : null,
      label: arming ? 'heart-rate baseline ' + s.hr_n + ' / ' + s.hr_need : 'heart-rate baseline (starts at ARM)' },
    eye,
  ];
}

// ── Page (browser only) ──────────────────────────────────────────────────────
function boot() {
  const $ = id => document.getElementById(id);
  const cv = $('cv'), ctx = cv.getContext('2d');
  let S = null, lastOk = 0, frame = null, test = null, lastHog = null, flashUntil = 0, noteTimer = 0;

  function note(msg) {
    $('note').textContent = msg;
    clearTimeout(noteTimer);
    noteTimer = setTimeout(() => { $('note').textContent = ''; }, 4000);
  }

  function post(path) {
    return fetch(path, { method: 'POST' })
      .then(r => { if (!r.ok) note('helmet busy -- try again'); })
      .catch(() => note('no connection to the helmet'));
  }

  function rows(el, list) {
    el.innerHTML = '';
    for (const c of list) {
      const li = document.createElement('li');
      li.className = c.ok === null ? 'wait' : c.ok ? 'ok' : 'bad';
      const b = document.createElement('b');
      b.textContent = c.ok === null ? '...' : c.ok ? 'OK' : 'FAIL';
      li.appendChild(b);
      li.appendChild(document.createTextNode(c.label));
      if (c.ok === false && c.hint) {
        const h = document.createElement('span');
        h.className = 'hint';
        h.textContent = c.hint;
        li.appendChild(h);
      }
      el.appendChild(li);
    }
  }

  function draw() {
    if (!frame) return;
    const W = frame.width, H = frame.height;
    if (cv.width !== H || cv.height !== W) { cv.width = H; cv.height = W; }
    ctx.setTransform(0, -1, 1, 0, 0, W);          // native (x, y) -> display (y, W - x)
    ctx.drawImage(frame, 0, 0);
    if (S && S.roi) {
      const c = hogCropRect(S.roi);
      ctx.lineWidth = 2;
      ctx.strokeStyle = '#ffd400';                  // the classifier's crop
      ctx.strokeRect(c.x, c.y, c.w, c.h);
      ctx.strokeStyle = '#3ecf6a';                  // the ESP's eye box
      ctx.strokeRect(S.roi.x, S.roi.y, S.roi.size, S.roi.size);
    }
    ctx.setTransform(1, 0, 0, 1, 0, 0);
    if (Date.now() < flashUntil) {
      ctx.lineWidth = 8;
      ctx.strokeStyle = '#ff3030';
      ctx.strokeRect(4, 4, cv.width - 8, cv.height - 8);
      ctx.fillStyle = '#ff3030';
      ctx.font = 'bold 28px system-ui';
      ctx.fillText('BLINK', 12, 38);
    }
  }

  function render() {
    const now = Date.now();
    const lost = now - lastOk > LOST_MS;
    const st = S ? S.state : '';
    const ban = $('banner');
    if (!S || lost) {
      ban.className = 'lost';
      ban.textContent = st === 'RECORDING' ? 'RECORDING -- Wi-Fi now off' : 'NO CONNECTION';
    } else if (st === 'ARMING') {
      ban.className = 'arming';
      ban.textContent = 'ARMING ' + S.arm_s + ' / ' + S.arm_timeout_s + ' s';
    } else if (st === 'RECORDING') {
      ban.className = 'rec';
      ban.textContent = S.timed_out ? 'RECORDING (arming timed out)' : 'RECORDING';
    } else {
      ban.className = '';
      ban.textContent = 'IDLE';
    }

    const arm = $('arm');
    arm.textContent = st === 'ARMING' ? 'ABORT' : st === 'RECORDING' ? 'STOP' : 'ARM';
    arm.className = st === 'ARMING' ? 'abort' : st === 'RECORDING' ? 'stop' : '';
    arm.disabled = !S || lost;
    $('auto').disabled = !S || lost || st === 'RECORDING';
    const testing = !!test && test.phase !== 'done';
    $('test').textContent = testing ? 'Cancel test' : 'Blink test';
    $('test').disabled = !S || lost || st === 'RECORDING' || (!testing && !S.roi);

    $('eye').textContent = !S ? '' :
      !S.roi ? 'searching for the eye -- blink, or tap the eye to place the box' :
      'eye box (' + S.roi_src + ') at x=' + S.roi.x + ' y=' + S.roi.y + ' -- tap the picture to move it';

    if (S) rows($('arming'), armingRows(S));
    const checks = preflightChecks(S, test, frame ? frame.width : 0, frame ? frame.height : 0);
    rows($('preflight'), checks);
    const v = verdict(checks);
    $('verdict').textContent = v === null ? '' : v ? 'READY TO RECORD' : 'NOT READY -- see above';
    $('verdict').style.color = v ? 'var(--ok)' : 'var(--bad)';

    let p = '';
    if (test && test.phase === 'still')
      p = 'HOLD STILL, EYES OPEN  ' + Math.max(0, Math.ceil(STILL_SECONDS - (now / 1000 - test.start))) + ' s';
    else if (test && test.phase === 'blink')
      p = 'BLINK ' + BLINK_TARGET + ' TIMES  ' + Math.max(0, Math.ceil(BLINK_SECONDS - (now / 1000 - test.start))) +
          ' s  (counted ' + test.count + ')';
    $('prompt').textContent = p;
  }

  function pollStatus() {
    fetch('/status', { cache: 'no-store' })
      .then(r => r.json())
      .then(s => {
        S = s;
        lastOk = Date.now();
        if (lastHog !== null && s.hog_total > lastHog) flashUntil = Date.now() + 500;
        lastHog = s.hog_total;
        if (test && test.phase !== 'done' && s.state === 'RECORDING') {
          test = null;
          note('blink test abandoned -- recording started');
        } else if (test) {
          test = blinkTestStep(test, Date.now() / 1000, s.hog_total);
          if (!test) note('blink test abandoned -- the helmet restarted');
        }
      })
      .catch(() => {})
      .finally(() => {
        render();
        draw();
        setTimeout(pollStatus, test && test.phase !== 'done' ? 250 : 1000);
      });
  }

  // Pull, don't stream: the next frame is requested only after this one lands,
  // so the rate throttles itself to the link and stops when the page closes.
  function pullFrame() {
    const t0 = Date.now();
    if (!S || S.state === 'RECORDING' || t0 - lastOk > LOST_MS) { setTimeout(pullFrame, 1000); return; }
    fetch('/frame.jpg', { cache: 'no-store' })
      .then(r => r.status === 200 ? r.blob() : null)
      .then(b => b ? createImageBitmap(b) : null)
      .then(bm => { if (bm) { if (frame) frame.close(); frame = bm; draw(); } })
      .catch(() => {})
      .finally(() => setTimeout(pullFrame, Math.max(0, FRAME_PERIOD_MS - (Date.now() - t0))));
  }

  cv.addEventListener('click', e => {
    if (!frame || !S || S.state === 'RECORDING') return;
    const r = cv.getBoundingClientRect();
    const px = (e.clientX - r.left) * cv.width / r.width;
    const py = (e.clientY - r.top) * cv.height / r.height;
    const p = displayToNative(px, py, frame.width, frame.height);
    if (confirm('Put the eye box here? (x=' + p[0] + ', y=' + p[1] + ')')) post('/roi?x=' + p[0] + '&y=' + p[1]);
  });
  $('auto').addEventListener('click', () => {
    if (confirm('Clear the stored eye position and let the helmet search for the eye?')) post('/roi?auto');
  });
  $('arm').addEventListener('click', () => {
    if (S && S.state === 'RECORDING' && !confirm('Stop recording?')) return;
    post('/press');
  });
  $('test').addEventListener('click', () => {
    if (test && test.phase !== 'done') test = null;
    else if (S) test = blinkTestStart(Date.now() / 1000, S.hog_total);
    render();
  });

  pollStatus();
  pullFrame();
}

if (typeof document !== 'undefined') boot();
</script>
</body></html>
)HTML";
```

- [ ] **Step 4: Run the test and watch it pass**

Run: `node tools/phone_page_test.js`
Expected: `phone_page: all checks passed`

- [ ] **Step 5: Commit**

```bash
git add src/phone_page.h tools/phone_page_test.js
git commit -m "Add the phone arming page and a node check of its pure logic

Tap -> camera coordinate map, hogCropRect mirror, and the optional
blink test (a port of live_ear_preview.py --arm) with its verdict."
```

---

### Task 3: Access point and web server

`PhonePreview.h` is written so it can't reach firmware state: it only holds a status string, a frame, and one pending command line. This task gets it compiling and serving the page. Task 4 connects it to live data.

**Files:**
- Create: `src/PhonePreview.h`
- Modify: `platformio.ini` (`[env:esp32s3cam_sd]` build_flags)
- Modify: `src/main.cpp` (include block after the `SD_MMC` pin defines near line 84; end of `setup()`)

- [ ] **Step 1: Write `src/PhonePreview.h`**

```cpp
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
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "phone_page.h"

#ifndef PHONE_AP_PASS
#error "PHONE_AP_PASS must be set in platformio.ini (WPA2, 8+ characters)"
#endif
static_assert(sizeof(PHONE_AP_PASS) - 1 >= 8, "PHONE_AP_PASS: WPA2 needs 8+ characters");

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
  WiFi.mode(WIFI_AP);
  if (!WiFi.softAP(ssid, PHONE_AP_PASS)) {
    Serial.println(F("#ERROR: Phone preview: Wi-Fi AP failed to start"));
    WiFi.mode(WIFI_OFF);
    return;
  }
  WiFi.setTxPower(TX_POWER);

  httpd_config_t cfg   = HTTPD_DEFAULT_CONFIG();
  cfg.core_id          = 0;     // never loop()'s core 1: the 500 Hz pulse sampler lives there
  cfg.task_priority    = 1;     // below the camera task (2): the server only gets its slack
  cfg.lru_purge_enable = true;  // a phone that walks off without closing must not pin sockets
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
  Serial.printf("#STATUS: Phone preview up -- join Wi-Fi %s, open http://%s (internal heap free %u)\n",
                ssid, WiFi.softAPIP().toString().c_str(),
                (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
}

// Server and Wi-Fi down, for the ride. Buffers stay allocated for the next start().
static void stop() {
  if (!s_server) return;
  httpd_handle_t h = s_server;
  s_server = nullptr;              // offerFrame() stops copying before the server goes
  httpd_stop(h);
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);
}

}  // namespace PhonePreview
```

- [ ] **Step 2: Turn it on for the SD build**

In `platformio.ini`, under `[env:esp32s3cam_sd]`, append to `build_flags` (after `-DCAMERA_JPEG_QUALITY=10`):

```ini
    ; ── Phone arming preview: Wi-Fi AP "HELMET-xxxx", page at http://192.168.4.1
    ; (docs/superpowers/specs/2026-09-23-phone-arming-preview-design.md).
    ; Delete these two lines for the no-Wi-Fi build. WPA2 needs 8+ characters;
    ; change the password before taking the helmet anywhere shared.
    -DPHONE_PREVIEW
    -DPHONE_AP_PASS=\"helmet-arm\"
```

- [ ] **Step 3: Include it from `main.cpp`**

In `src/main.cpp`, directly after the `#endif` that closes the `SD_MMC` block (the one ending with `#define SD_MMC_D0_PIN 40`), add:

```cpp

// ── Phone arming preview (SD build, -DPHONE_PREVIEW) ─────────────────────
#if defined(PHONE_PREVIEW)
#if !defined(STORAGE_MODE_SD)
#error "PHONE_PREVIEW is for the SD build: it reports the arming state that only exists there"
#endif
#include "PhonePreview.h"
#endif
```

- [ ] **Step 4: Start it at the end of `setup()`**

At the very end of `setup()`, after `signalState(g_sessionState);`, add:

```cpp

#if defined(PHONE_PREVIEW)
  // Last, so a Wi-Fi start failure cannot hold up the sensors or the camera.
  PhonePreview::start();
#endif
```

- [ ] **Step 5: Build**

Run: `pio run -e esp32s3cam_sd -e esp32s3cam`
Expected: both `SUCCESS`. For `esp32s3cam_sd`, note the `Flash:` and `RAM:` lines. Flash was 494 KB before; expect roughly 1.1–1.3 MB of the 3 MB `huge_app` partition. Write both numbers down for Task 5.

- [ ] **Step 6: Commit**

```bash
git add src/PhonePreview.h src/main.cpp platformio.ini
git commit -m "Add PhonePreview: WPA2 AP + esp_http_server on the SD build

Serves the page, a status string, the latest frame and a one-slot command
queue. Holds no firmware state; main.cpp wires it up next."
```

---

> **Task 3 after review:** the code above is the first cut (`6e3dafec`). Review fixes followed:
> - TX power set with `esp_wifi_set_max_tx_power` and reported back.
> - `WiFi.persistent(false)` and no `softAPdisconnect`, so there are no flash writes on stop/start.
> - httpd send/recv timeouts of 2 s and a 6 KB stack.
> - `stop()` drops any queued command.
> - The heap and largest DMA block go in the `up` line.
> - The password comes from the `HELMET_AP_PASS` environment variable, since the repo is public.
>
> The committed `src/PhonePreview.h` and `platformio.ini` are the reference. Every build of `esp32s3cam_sd` from here on needs `HELMET_AP_PASS` set.

### Task 4: Wire it to the firmware

This task connects the frames, the status JSON, the commands and the Wi-Fi on/off timing. Everything the page asks for runs in `phoneTick()` on `loop()`'s task.

**Files:**
- Modify: `src/main.cpp` (`cameraTask` SD branch ~line 1377; new functions after `handleCommandLine()`; `loop()` after the ARMING block ~line 2148)

- [ ] **Step 1: Hand frames to the preview in `cameraTask`**

In the `#if defined(STORAGE_MODE_SD)` branch of `cameraTask`, change

```cpp
        saveJpegToSD(fb->buf, fb->len, ts);
#if defined(EAR_PROFILE)
        uint32_t tSd1 = (uint32_t)micros();
#endif
        if (++g_earFrameCounter % EAR_THROTTLE_DIV == 0) {
```

to

```cpp
        saveJpegToSD(fb->buf, fb->len, ts);
#if defined(EAR_PROFILE)
        uint32_t tSd1 = (uint32_t)micros();
#endif
#if defined(PHONE_PREVIEW)
        // Before the ~97 ms EAR pass, so the phone gets the freshest frame.
        // Never while recording: the ride path stays as it is without Wi-Fi.
        // (Under EAR_PROFILE this copy, ~0.5 ms, lands in ear=.)
        if (g_sessionState != SESSION_RECORDING) PhonePreview::offerFrame(fb->buf, fb->len);
#endif
        if (++g_earFrameCounter % EAR_THROTTLE_DIV == 0) {
```

- [ ] **Step 2: Add the status builder and `phoneTick()` right after `handleCommandLine()`**

```cpp
#if defined(PHONE_PREVIEW)
// ─────────────────────────────────────────────────────────────────────────
// Phone arming preview — the loop() side of PhonePreview.h.
// ─────────────────────────────────────────────────────────────────────────
// Wi-Fi stays up this long into RECORDING so the page can show that recording
// started, then goes off for the ride (spec: "Wi-Fi lifecycle").
static const uint32_t PHONE_WIFI_OFF_AFTER_MS = 15000;

// Indexed by SessionState. The page shows these and echoes one back with a
// press (/press?expect=), so the two must use the same names.
static const char *const PHONE_STATE_NAME[] = {"IDLE", "ARMING", "RECORDING"};

// The /status JSON. Readiness uses the same globals as the ARMING block in loop().
static void buildPhoneStatus(char *out, size_t n, unsigned long now) {
  static const char *const CHECK[] = {"pending", "pass", "fail"};
  char roi[48] = "null";
  if (g_earLockDone && g_earRoi.locked)
    snprintf(roi, sizeof roi, "{\"x\":%d,\"y\":%d,\"size\":%d}",
             g_earRoi.x, g_earRoi.y, g_earRoi.size);
  const bool arming = (g_sessionState == SESSION_ARMING);
  snprintf(out, n,
           "{\"state\":\"%s\",\"arm_s\":%lu,\"arm_timeout_s\":%lu,\"timed_out\":%d,"
           "\"imu\":%d,\"hr\":%d,\"hr_n\":%u,\"hr_need\":%u,"
           "\"eye_check\":\"%s\",\"blinks_since_lock\":%u,\"blinks_need\":%u,"
           "\"roi\":%s,\"roi_src\":\"%s\",\"roi_conf\":%.2f,\"hog_total\":%lu}",
           PHONE_STATE_NAME[g_sessionState],
           arming ? (unsigned long)((now - g_armStartMs) / 1000) : 0UL,
           (unsigned long)(ARMING_TIMEOUT_MS / 1000),
           g_armTimedOut ? 1 : 0,
           g_armCalib.done ? 1 : 0,
           g_baselineFormed ? 1 : 0,
           (unsigned)g_hrBase.progress(), (unsigned)HrBaseline::NEEDED,
           CHECK[g_eyeCheck],
           (unsigned)g_earBlinksSinceLock, (unsigned)EAR_CHECK_MIN_BLINKS,
           roi, g_earRoiSource, g_earLockConfidence,
           (unsigned long)g_earHogBlinkTotal);
}

// Every loop() pass: the Wi-Fi lifecycle, commands from the page, and the
// status snapshot. Whatever the page asks for happens here, on loop()'s task.
static void phoneTick(unsigned long now) {
  static SessionState prev = SESSION_IDLE;
  static unsigned long recSince = 0;
  if (g_sessionState != prev) {
    if (g_sessionState == SESSION_RECORDING) recSince = now;
    // Back from a ride. Blocks a few hundred ms, but nothing is recording.
    if (g_sessionState == SESSION_IDLE) PhonePreview::start();
    prev = g_sessionState;
  }
  if (g_sessionState == SESSION_RECORDING && PhonePreview::running() &&
      now - recSince >= PHONE_WIFI_OFF_AFTER_MS) {
    // ponytail: blocks loop() >= 100 ms mid-recording (httpd_stop's own wait;
    // up to ~2 s if the server is mid-send) -- 50+ pulse samples, once per
    // session. Logged below; move stop() to a one-shot task if the gap shows in
    // the CSV (start() would then need a guard against a stop still in flight).
    const unsigned long t0 = millis();
    PhonePreview::stop();
    Serial.printf("#STATUS: Phone preview off for the ride (took %lu ms) -- back on when the session stops\n",
                  (unsigned long)(millis() - t0));
  }
  if (!PhonePreview::running()) return;

  char cmd[PhonePreview::CMD_MAX];
  if (PhonePreview::takeCommand(cmd, sizeof cmd)) {
    if (strncmp(cmd, "PRESS:", 6) == 0) {
      // A press means what the rider saw only if the helmet is still in the
      // state the page showed; otherwise drop it (stale label, double tap).
      if (strcmp(cmd + 6, PHONE_STATE_NAME[g_sessionState]) == 0) {
        handleCommandLine("BUTTON", now);
      } else {
        Serial.printf("#STATUS: Phone press ignored -- page showed %s, helmet is %s\n",
                      cmd + 6, PHONE_STATE_NAME[g_sessionState]);
      }
    } else {
      handleCommandLine(cmd, now);
    }
  }

  static unsigned long lastStatus = 0;
  if (now - lastStatus >= 250) {
    lastStatus = now;
    char json[PhonePreview::STATUS_MAX];
    buildPhoneStatus(json, sizeof json, now);
    PhonePreview::setStatus(json);
  }
}
#endif
```

- [ ] **Step 3: Call it from `loop()`**

In `loop()`, directly after the closing `}` of the `if (g_sessionState == SESSION_ARMING) { ... }` block and before the `// ── Pulse sensor: 500 Hz` comment, add:

```cpp
#if defined(PHONE_PREVIEW)
  phoneTick(now);
#endif
```

- [ ] **Step 4: Build all ESP environments**

Run: `pio run -e esp32s3cam_sd -e esp32s3cam -e esp32s3cam_ear_preview -e esp32s3cam_frame_inject`
Expected: all four `SUCCESS`.

- [ ] **Step 5: Flash and smoke-test from a laptop**

The Wi-Fi password comes from the `HELMET_AP_PASS` environment variable at build time (8+ characters; the repo is public, so it isn't committed). Set it once with `setx HELMET_AP_PASS your-pass-here` and open a new terminal.

Run: `pio run -e esp32s3cam_sd -t upload`, then `pio device monitor -e esp32s3cam_sd`
Expected in the monitor: `#STATUS: Phone preview up -- join Wi-Fi HELMET-XXXX, open http://192.168.4.1 (...)`, which also reports the TX power, free internal heap and largest internal DMA block.

Join `HELMET-XXXX` from the laptop (password: your `HELMET_AP_PASS`), then in PowerShell (use `curl.exe`; plain `curl` there is `Invoke-WebRequest`):

```
curl.exe -s http://192.168.4.1/status
```
Expected: one JSON object starting `{"state":"IDLE",` with `"hr_need":30`, `"blinks_need":3`.

```
curl.exe -s -o frame.jpg -w "%{http_code} %{size_download}\n" http://192.168.4.1/frame.jpg ; Start-Sleep -Milliseconds 300 ; curl.exe -s -o frame.jpg -w "%{http_code} %{size_download}\n" http://192.168.4.1/frame.jpg
```
Expected: the first line may print `204 0` (the first request only tells the camera someone is looking). The second must print `200` and a size of roughly 5000–15000. Both requests are on one line because a frame is only kept for 2 s after the last request. `frame.jpg` opens as the camera picture (unrotated).

```
curl.exe -s -X POST "http://192.168.4.1/roi?x=abc&y=1" -w " %{http_code}\n"
```
Expected: `400`, and nothing new in the monitor.

```
curl.exe -s -X POST "http://192.168.4.1/press?expect=IDLE"
```
Expected: `ok`. The monitor shows `#STATE: ARMING` and `#STATUS: Arming -- hold still, ...`. Repeat the same command; expected: `ok`, but the monitor shows `#STATUS: Phone press ignored -- page showed IDLE, helmet is ARMING`. Then send `?expect=ARMING`; expected: `#STATUS: Arming aborted`.

```
curl.exe -s -X POST -H "Origin: http://evil.example" "http://192.168.4.1/press?expect=IDLE" -w " %{http_code}\n"
```
Expected: `403`, and nothing new in the monitor.

Delete `frame.jpg` afterwards.

- [ ] **Step 6: Smoke-test from the phone**

Join `HELMET-XXXX` on the phone. If the phone warns the network has no internet, choose to stay connected. Open `http://192.168.4.1`.
Expected: the banner reads `IDLE`, the camera picture updates about 3 times a second, rotated the same way as `live_ear_preview.py`. Once the monitor prints `#STATUS: EAR ROI ... at x=.. y=..`, a green box and a yellow crop box appear, and the text under the picture shows the same x/y.

- [ ] **Step 7: Commit**

```bash
git add src/main.cpp
git commit -m "Wire the phone preview: frames, status JSON, commands, Wi-Fi lifecycle

Page commands go through handleCommandLine() on loop()'s task, same as
serial. Frames only outside RECORDING; Wi-Fi off 15 s into a recording,
back on at IDLE."
```

---

### Task 5: Bench verification on the hardware

Each check below is a spec requirement or one of its risks. Record the numbers in Step 8. If a check fails, stop and bring the numbers back before changing anything.

**Files:**
- Modify: `docs/superpowers/specs/2026-09-23-phone-arming-preview-design.md` (append results)

Monitor with timestamps throughout: `pio device monitor -e esp32s3cam_sd -f time`

- [ ] **Step 1: Three ways to press, same result (Requirement 1)**

With the page open on the phone:
1. Press the **physical GPIO 21 button**. Expected: the monitor shows `#STATE: ARMING`, and the page banner changes to `ARMING n / 60 s`. Press it again. Expected: `#STATUS: Arming aborted`, and the page shows `IDLE`.
2. Type `BUTTON` + Enter in the monitor. Expected: same as 1. Type it again to abort.
3. Tap **ARM** on the page. Expected: same as 1. Tap **ABORT**.
4. Turn the phone's Wi-Fi off. Press the physical button. Expected: `#STATE: ARMING`, and arming runs to `#STATE: RECORDING` (or the timeout) with no phone connected.

- [ ] **Step 2: The ESP's eye box, and placing it by tap (Requirements 3 and 4)**

1. Tap **Auto eye** and confirm. Expected: `#STATUS: EAR ROI cleared -- searching for the eye again`, and the page reads `searching for the eye`. Blink normally until `#STATUS: EAR ROI motion-locked at x=X y=Y ...`. Expected: the page's text reads `eye box (motion) at x=X y=Y` with the same X/Y, and the green box sits on that spot in the picture.
2. Tap the centre of the pupil in the picture and confirm. Expected: `#STATUS: EAR ROI saved cx=.. cy=..`, then `#STATUS: EAR ROI stored at x=.. y=..`. The green box is now centred where you tapped, within a few pixels. If the box lands mirrored or on the wrong axis, the tap mapping is wrong; stop here.
3. Power-cycle the helmet and reload the page. Expected: `#STATUS: stored eye coordinate cx=.. cy=..` at boot, and the box comes back in the same place.

- [ ] **Step 3: The blink test is optional (Requirement 2)**

1. Without touching **Blink test**: press ARM, and let it reach `#STATE: RECORDING` or the timeout. Expected: recording starts; the test's panel only ever shows `blink test: tap Blink test to run it`.
2. Stop the session. In IDLE, with the eye box on the eye, tap **Blink test**. Expected: `HOLD STILL, EYES OPEN 8 s` counting down, then `BLINK 10 TIMES 20 s (counted n)` with n rising as you blink, and a red BLINK flash on each blink. At the end, `blink test: N of 10 detected` and `false alarms while still: M`, then `READY TO RECORD` or `NOT READY`.
3. Start the test again and tap **Cancel test**. Expected: the prompt clears and nothing else changes.

- [ ] **Step 4: Wi-Fi lifecycle**

Press ARM and let it record for at least 2 minutes.
Expected:
- The page shows `RECORDING` for ~15 s.
- The monitor then prints `#STATUS: Phone preview off for the ride (took N ms) -- ...`, and the page shows `RECORDING -- Wi-Fi now off` within ~4 s. Record N; expect roughly 100–300 ms.
- Stop with the physical button. Expected: `#STATUS: Wrote N frames ...`, then `#STATUS: Phone preview up -- ...`. The phone can rejoin and reload.

Repeat the same 2-minute recording with a build that has the two `PHONE_PREVIEW` lines commented out in `platformio.ini`. Expected: its `Wrote N frames` is within 5% of the Wi-Fi build's. Restore the two lines afterwards.

- [ ] **Step 5: Eye-detection frame rate with the page open**

```powershell
$env:PLATFORMIO_BUILD_FLAGS='-DEAR_PROFILE'; pio run -e esp32s3cam_sd -t upload; Remove-Item Env:PLATFORMIO_BUILD_FLAGS
```

In IDLE, with the eye locked, watch `#PROF:` lines for 60 s with the page closed, then 60 s with the page open. Each `#PROF` line covers 40 frames, so frames/s = 40 ÷ the seconds between consecutive `#PROF` timestamps.
Pass: frames/s with the page open is at least 80% of the rate with it closed, **and** the Step 3 blink test detects at least 8 of 10 with the page open.
If it fails: raise `FRAME_PERIOD_MS` in `src/phone_page.h` to `1000`, rerun `node tools/phone_page_test.js`, reflash, and measure again.
Reflash the normal build afterwards: `pio run -e esp32s3cam_sd -t upload`

- [ ] **Step 6: Pulse noise with Wi-Fi on**

Arm 3 times with the phone page open (normal build), and 3 times with the `PHONE_PREVIEW` lines commented out. Keep the finger/sensor placement the same each time. For each arm, record the seconds from `#STATE: ARMING` to `#STATUS: Baseline HR formed: X BPM`, and the value X.
Pass: the median time to form the baseline with Wi-Fi is within 10 s of the median without it.
If it fails: set `TX_POWER` in `src/PhonePreview.h` to `WIFI_POWER_2dBm`, and repeat the 3 Wi-Fi arms (the `Phone preview up` line reports the TX power actually applied).
Restore the `PHONE_PREVIEW` lines afterwards.

Cleaner second A/B from the same session, with identical sensor placement: Wi-Fi is on for the first 15 s of every recording and off after. In the Step 4 recording's `sensor_data.csv`, compare the spread (standard deviation) of `pulse_raw` and the share of `signal_quality` = 1 over seconds 0–15 against seconds 15–30. Also look for a step in `pulse_raw` at the moment Wi-Fi stops.

- [ ] **Step 7: Memory**

From the Task 3 build output and the `Phone preview up` lines, record Flash %, RAM %, internal heap free and the largest internal DMA block. Take them from the 1st (boot), 2nd and 3rd `Phone preview up` lines, i.e. after two record/stop cycles.
Pass: internal heap free ≥ 30000 bytes after the AP is up. The 2nd and 3rd values must not keep falling; a steady drop means a leak across start/stop.

- [ ] **Step 8: Record the results and commit**

Append to the spec:

```markdown
## Bench results (YYYY-MM-DD)

| Check | Result |
|---|---|
| Button / serial / phone press | pass / fail — notes |
| Eye box matches serial; tap places it | pass / fail — notes |
| Blink test optional; runs and scores | pass / fail — N of 10, M false |
| Wi-Fi off at 15 s, back at IDLE; frames written vs no-Wi-Fi build | N vs N |
| EAR frames/s page closed → open | a → b (c %) |
| Median time to HR baseline, Wi-Fi off → on | a s → b s |
| Flash / RAM / internal heap free | a % / b % / c bytes |
```

Fill in the real values. Then:

```bash
git add docs/superpowers/specs/2026-09-23-phone-arming-preview-design.md
git commit -m "Record phone arming preview bench results"
```

---

### Task 6: Docs

**Files:**
- Modify: `../README.md` (section 3 table row for `esp32s3cam_sd`; new subsection at the end of section 6)
- Modify: `../../CLAUDE.md` (firmware table)

- [ ] **Step 1: README environment table**

In `../README.md`, replace the `esp32s3cam_sd` row of the section 3 table with:

```markdown
| `esp32s3cam_sd` | **Production mode** — everything (CSV + MJPEG video) saved to microSD, no PC needed; GPIO 21 button starts/stops a session. Also runs the phone arming preview (Wi-Fi `HELMET-xxxx`, see section 6) unless the two `PHONE_PREVIEW` lines are removed | 115200 |
```

- [ ] **Step 2: README usage section**

At the end of section 6 (just before `## 7. Labeling a Session`), add:

```markdown
### Phone arming preview (no laptop)

The `esp32s3cam_sd` build brings up its own Wi-Fi, `HELMET-xxxx`. Its password
is read at build time from the `HELMET_AP_PASS` environment variable (8+
characters, letters/digits/dashes); it is not in the repo, which is public.
Set it once, then open a new terminal before building:

```bash
setx HELMET_AP_PASS your-pass-here
```

Without it, building `esp32s3cam_sd` stops with a message saying so. Join the
network from the phone (if the phone says it has no internet, choose to stay
connected) and open **http://192.168.4.1**:

- **Banner** — `IDLE`, `ARMING n / 60 s`, `RECORDING`.
- **Picture** — the helmet camera, rotated as in `live_ear_preview.py`, with
  the ESP's eye box (green) and the classifier's crop (yellow). Red flash on
  each detected blink.
- **Tap the eye** to place the eye box — the same `ROI:<cx>,<cy>` the PC tool
  sends, stored in NVS the same way. **Auto eye** clears it.
- **ARM / ABORT / STOP** — one more way to press the GPIO 21 button. The
  button itself works exactly as before, phone or no phone.
- **Arming** checklist — the firmware's own gate: IMU, HR baseline, eye check.
- **Blink test** (optional, as SPACE in `live_ear_preview.py --arm`) — hold
  still 8 s, then blink 10 times; READY / NOT READY. Arming never waits for it.

Wi-Fi goes off 15 s into a recording, so the ride records exactly as without
it, and comes back when the session stops. Stop a recording with the button.
The page's pure logic is checked with `node tools/phone_page_test.js`.
```

- [ ] **Step 3: CLAUDE.md firmware table**

In `../../CLAUDE.md`, in the `### Firmware (fatigue-helmet/firmware/)` table, add this row after `SD Card Mode`:

```markdown
| **Phone Arming Preview** | ✅ Implemented | `-DPHONE_PREVIEW` on `esp32s3cam_sd`: Wi-Fi AP + page at 192.168.4.1 — arming checklist, camera with eye box, tap-to-set eye, optional blink test, ARM button; Wi-Fi off 15 s into recording |
```

- [ ] **Step 4: Commit**

```bash
git add ../README.md ../../CLAUDE.md
git commit -m "Document the phone arming preview"
```
