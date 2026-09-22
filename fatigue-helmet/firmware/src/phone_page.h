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
canvas { display:block; margin:0 auto; width:100%; max-width:calc(60vh * 3 / 4); border-radius:8px; background:#000; touch-action:manipulation; }
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
#note { color:var(--warn); font-size:14px; min-height:1.2em; text-align:center; }
</style></head>
<body>
<div id="banner">CONNECTING...</div>
<div id="prompt"></div>
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
<ul id="preflight"></ul>
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
const FETCH_TIMEOUT_MS = 2500;  // without it a request to a helmet whose Wi-Fi went off hangs ~1-2 min on the OS connect timeout

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
  // S can be ~1 s stale at the moment of the tap; take the baseline from the
  // first status that lands after it instead of counting pre-tap blinks.
  if (t.base === null) return Object.assign({}, t, { base: hog, start: nowS, count: 0 });
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

// READY only once a blink test has actually finished and every check passes;
// a "waiting" check after a finished test now reads NOT READY, not blank.
function verdict(checks, t) {
  if (!t || t.phase !== 'done') return null;
  return checks.every(c => c.ok === true);
}

// Attaches the reference ROI a finished test is judged against, taken when
// the test FINISHES rather than at the tap (see staleTest).
function withRoi(t, s) {
  return Object.assign({}, t, { roi: s.roi, roiSrc: s.roi_src });
}

// True when a FINISHED test's verdict no longer applies. Drift correction
// (earDriftTick, runs in IDLE too) nudges the ROI by a few px all the time,
// and the test's 10 deliberate blinks are exactly what makes a drift cycle
// confident -- so a few px of drift must NOT clear the verdict. A quarter of
// the box (12 px at size 48) is a real move: re-tap, Auto, or a relock.
function staleTest(t, s) {
  return !!t && t.phase === 'done' && t.roi !== undefined && (
    s.state === 'RECORDING' || !s.roi || !t.roi || s.roi_src !== t.roiSrc ||
    Math.abs(s.roi.x - t.roi.x) > t.roi.size / 4 || Math.abs(s.roi.y - t.roi.y) > t.roi.size / 4);
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

  // fetch with a timeout: without it a request to a helmet whose Wi-Fi just
  // went off hangs ~1-2 min on the OS connect timeout instead of failing fast.
  function get(url, opt) {
    const ac = new AbortController();
    setTimeout(() => ac.abort(), FETCH_TIMEOUT_MS);
    return fetch(url, Object.assign({ cache: 'no-store', signal: ac.signal }, opt));
  }

  function post(path) {
    return get(path, { method: 'POST' })
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
    if (performance.now() < flashUntil) {
      ctx.lineWidth = 8;
      ctx.strokeStyle = '#ff3030';
      ctx.strokeRect(4, 4, cv.width - 8, cv.height - 8);
      ctx.fillStyle = '#ff3030';
      ctx.font = 'bold 28px system-ui';
      ctx.fillText('BLINK', 12, 38);
    }
  }

  function render() {
    const now = performance.now();
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
    const v = verdict(checks, test);

    // One element for both: a running test's prompt and a finished test's
    // verdict never show at the same time, so there is nothing to stack.
    const prompt = $('prompt');
    let p = '';
    if (test && test.phase === 'still') {
      p = 'HOLD STILL, EYES OPEN  ' + Math.max(0, Math.ceil(STILL_SECONDS - (now / 1000 - test.start))) + ' s';
      prompt.style.color = 'var(--warn)';
    } else if (test && test.phase === 'blink') {
      p = 'BLINK ' + BLINK_TARGET + ' TIMES  ' + Math.max(0, Math.ceil(BLINK_SECONDS - (now / 1000 - test.start))) +
          ' s  (counted ' + test.count + ')';
      prompt.style.color = 'var(--warn)';
    } else if (v !== null) {
      p = v ? 'READY TO RECORD' : 'NOT READY -- see below';
      prompt.style.color = v ? 'var(--ok)' : 'var(--bad)';
    }
    prompt.textContent = p;
  }

  function pollStatus() {
    get('/status')
      .then(r => r.json())
      .then(s => {
        if (typeof s.state !== 'string' || typeof s.hog_total !== 'number') throw new Error('bad status');
        S = s;
        lastOk = performance.now();
        if (lastHog !== null && s.hog_total > lastHog) flashUntil = performance.now() + 500;
        lastHog = s.hog_total;
        if (test && test.phase !== 'done' && s.state === 'RECORDING') {
          test = null;
          note('blink test abandoned -- recording started');
        } else if (test) {
          test = blinkTestStep(test, performance.now() / 1000, s.hog_total);
          if (!test) note('blink test abandoned -- the helmet restarted');
        }
        // Attach the reference ROI on the first status after the test finishes.
        if (test && test.phase === 'done' && test.roi === undefined) test = withRoi(test, s);
        if (staleTest(test, s)) {
          test = null;
          // Arming drops and re-acquires the ROI by design (the firmware's
          // eye check takes over), so it isn't a stale-test note either.
          if (s.state !== 'RECORDING' && s.state !== 'ARMING') note('eye box moved -- rerun the blink test');
        }
      })
      .catch(() => {})
      .finally(() => {
        // Schedule the next poll before rendering, so an exception in render
        // can't stop polling forever.
        setTimeout(pollStatus, test && test.phase !== 'done' ? 250 : 1000);
        render();
        draw();
      });
  }

  // Pull, don't stream: the next frame is requested only after this one lands,
  // so the rate throttles itself to the link and stops when the page closes.
  function pullFrame() {
    const t0 = performance.now();
    if (!S || S.state === 'RECORDING' || t0 - lastOk > LOST_MS) { setTimeout(pullFrame, 1000); return; }
    get('/frame.jpg')
      .then(r => r.status === 200 ? r.blob() : null)
      .then(b => b ? createImageBitmap(b) : null)
      .then(bm => { if (bm) { if (frame) frame.close(); frame = bm; draw(); } })
      .catch(() => {})
      .finally(() => setTimeout(pullFrame, Math.max(0, FRAME_PERIOD_MS - (performance.now() - t0))));
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
    post('/press?expect=' + S.state);
  });
  $('test').addEventListener('click', () => {
    if (test && test.phase !== 'done') test = null;
    else if (S) test = blinkTestStart(performance.now() / 1000, null);
    render();
  });

  pollStatus();
  pullFrame();
}

if (typeof document !== 'undefined') boot();
</script>
</body></html>
)HTML";
