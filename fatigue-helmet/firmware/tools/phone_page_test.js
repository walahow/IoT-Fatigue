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
  '\n;({displayToNative, hogCropRect, cropMargin, blinkTestStart, blinkTestStep, preflightChecks, verdict, armingRows, withRoi, staleTest})', {});

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
assert.strictEqual(P.cropMargin({ x: 136, y: 66, size: 48 }, W, H), 0);   // crop y = 66+24-90 = 0
assert.strictEqual(P.cropMargin({ x: 250, y: 96, size: 48 }, W, H), 1);   // right edge (catches a W/H swap)

// ── Preflight status, reused below.
const status = { roi: { x: 136, y: 96, size: 48 }, roi_src: 'motion', roi_conf: 2.4 };

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

// A null hog means the tap fired before the next /status landed; the first
// status that follows sets the baseline instead of counting pre-tap blinks.
let u = P.blinkTestStart(0, null);
u = P.blinkTestStep(u, 0.4, 120);
assert.strictEqual(u.base, 120);
assert.strictEqual(u.start, 0.4);
assert.strictEqual(u.count, 0);
assert.strictEqual(u.phase, 'still');
assert.strictEqual(P.blinkTestStep(u, 3, 121).count, 1);

// ── Verdict: READY only once a blink test has finished and every check passes.
// `t` above never had a reference ROI attached (withRoi), which verdict does
// not care about -- only staleTest does.
assert.strictEqual(P.verdict(P.preflightChecks(status, t, W, H), t), true);
assert.strictEqual(P.verdict(P.preflightChecks(status, null, W, H), null), null);   // test not run: no verdict
assert.strictEqual(P.verdict(P.preflightChecks({ ...status, roi_conf: 1.5 }, t, W, H), t), false);
assert.strictEqual(P.verdict(P.preflightChecks({ ...status, roi_conf: 2.0 }, t, W, H), t), true);   // right at the lock threshold
assert.strictEqual(P.verdict(P.preflightChecks({ ...status, roi_src: 'stored', roi_conf: -1 }, t, W, H), t), true);
assert.strictEqual(P.verdict(P.preflightChecks({ ...status, roi: { x: 136, y: 10, size: 48 } }, t, W, H), t), false);
assert.strictEqual(P.verdict(P.preflightChecks(status, { ...t, stillFalse: 2 }, W, H), { ...t, stillFalse: 2 }), true);
assert.strictEqual(P.verdict(P.preflightChecks(status, { ...t, stillFalse: 3 }, W, H), { ...t, stillFalse: 3 }), false);
assert.strictEqual(P.verdict(P.preflightChecks(status, { ...t, blinkDetected: 8 }, W, H), { ...t, blinkDetected: 8 }), true);
assert.strictEqual(P.verdict(P.preflightChecks(status, { ...t, blinkDetected: 7 }, W, H), { ...t, blinkDetected: 7 }), false);
// No frame yet (W = 0): the crop check can't be judged, so it's NOT READY,
// not a blank verdict -- a mutant that turns this false into null must fail.
assert.strictEqual(P.verdict(P.preflightChecks(status, t, 0, H), t), false);
// Crop margin exactly 0 (the box's classifier crop is exactly inside the
// frame): the CHECK must read true here, not just cropMargin() itself.
assert.strictEqual(P.verdict(P.preflightChecks({ ...status, roi: { x: 136, y: 66, size: 48 } }, t, W, H), t), true);

// ── withRoi / staleTest: the reference ROI is captured when the test
// FINISHES, not at the tap. Drift correction (earDriftTick, runs in IDLE
// too) nudges the box by a few px continuously, and the test's 10
// deliberate blinks are exactly what makes a drift cycle confident -- taking
// the reference at the tap made the verdict vanish ~1 s after it appeared.
const doneWithRoi = P.withRoi(t, status);
eq({ roi: doneWithRoi.roi, roiSrc: doneWithRoi.roiSrc }, { roi: status.roi, roiSrc: status.roi_src });
assert.strictEqual(P.staleTest(doneWithRoi, { ...status, roi: { x: 136, y: 98, size: 48 } }), false);   // 2 px drift
assert.strictEqual(P.staleTest(doneWithRoi, { ...status, roi: { x: 136, y: 109, size: 48 } }), true);   // 13 px move
assert.strictEqual(P.staleTest(doneWithRoi, { ...status, roi_src: 'stored' }), true);   // same x/y, source changed
assert.strictEqual(P.staleTest(doneWithRoi, { ...status, roi: null }), true);
assert.strictEqual(P.staleTest(doneWithRoi, { ...status, state: 'RECORDING' }), true);
assert.strictEqual(P.staleTest(t, status), false);   // done but not yet marked with a reference ROI
assert.strictEqual(P.staleTest({ ...doneWithRoi, phase: 'blink' }, { ...status, state: 'RECORDING' }), false);   // still running

// ── armingRows: IMU/HR only judged during ARMING; eye check tracks the
// firmware's own state regardless of phase.
eq(P.armingRows({ state: 'IDLE', imu: 0, hr: 0, hr_n: 0, hr_need: 30, eye_check: 'pending', roi: null, blinks_since_lock: 0, blinks_need: 3 })
  .slice(0, 2).map(r => r.ok), [null, null]);
const arming = P.armingRows({ state: 'ARMING', imu: 1, hr: 0, hr_n: 5, hr_need: 30, eye_check: 'pending', roi: null, blinks_since_lock: 0, blinks_need: 3 });
assert.strictEqual(arming[0].ok, true);
assert.strictEqual(arming[1].ok, false);
assert.strictEqual(P.armingRows({ state: 'ARMING', imu: 1, hr: 1, hr_n: 30, hr_need: 30, eye_check: 'fail', roi: null, blinks_since_lock: 0, blinks_need: 3 })[2].ok, false);
assert.strictEqual(P.armingRows({ state: 'ARMING', imu: 1, hr: 1, hr_n: 30, hr_need: 30, eye_check: 'pass', roi: null, blinks_since_lock: 3, blinks_need: 3 })[2].ok, true);

console.log('phone_page: all checks passed');
