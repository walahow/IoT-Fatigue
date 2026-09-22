// sensor_sim.cpp
// ============================================================================
// Host-side stand-in for the firmware's 1 Hz loop in SD mode. Rebuilds the
// sensor_data.csv the ESP32 would have written for a recorded session, using
// the firmware's own FuzzyFatigue.h and EyeBlinkEAR::BlinkRateWindow, and
// mirroring loop()'s baseline-HR / fuzzy-model / SD-row logic in main.cpp.
// session_replay is the camera task on the PC; this is the 1 Hz loop.
//
// Usage:
//   sensor_sim <sensor_data.csv> <out.csv> [--blinks=blinks.csv] [--baseline=BPM]
//
//   --blinks    a CSV with timestamp_ms and blink columns, e.g. session_replay's
//               --hog-csv output. blink_rate becomes the 60 s rolling count of
//               those blinks -- what the ESP would record if that detector
//               drove blink_rate. Without it the RECORDED blink_rate is used,
//               which must reproduce the recorded risk_pct/alert_level: that
//               is how this tool is checked against the real device.
//   --baseline  HR baseline in BPM. The ESP forms it (HrBaseline.h: median of 20
//               good readings after skipping the first 10) partly during arming,
//               before the CSV starts -- so it can't always be rebuilt from the
//               CSV alone. Without the flag the same rule runs on the recorded rows.
//
// Build (from this folder):
//   g++ -O2 -std=gnu++14 -Ihost_shim -o sensor_sim.exe sensor_sim.cpp
// ============================================================================

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "../../src/FuzzyFatigue.h"
#include "../../src/AlertGate.h"
#include "../../src/HrBaseline.h"
#include "../../src/EyeBlinkEAR.h"


static std::vector<std::string> split(const std::string &line) {
  std::vector<std::string> out;
  size_t start = 0;
  while (true) {
    size_t c = line.find(',', start);
    out.push_back(line.substr(start, c == std::string::npos ? std::string::npos : c - start));
    if (c == std::string::npos) return out;
    start = c + 1;
  }
}

static bool readLine(FILE *f, std::string &line) {
  line.clear();
  int ch;
  while ((ch = fgetc(f)) != EOF && ch != '\n') if (ch != '\r') line += (char)ch;
  return ch != EOF || !line.empty();
}

int main(int argc, char **argv) {
  if (argc < 3) {
    fprintf(stderr, "Usage: %s <sensor_data.csv> <out.csv> [--blinks=blinks.csv] [--baseline=BPM]\n", argv[0]);
    return 1;
  }
  const char *blinksPath = nullptr;
  float fixedBaseline = 0.0f;
  for (int i = 3; i < argc; i++) {
    if (strncmp(argv[i], "--blinks=", 9) == 0) blinksPath = argv[i] + 9;
    else if (strncmp(argv[i], "--baseline=", 11) == 0) fixedBaseline = (float)atof(argv[i] + 11);
  }

  // Blink events, in time order.
  std::vector<uint32_t> blinkTs;
  if (blinksPath) {
    FILE *bf = fopen(blinksPath, "r");
    if (!bf) { fprintf(stderr, "ERROR: cannot open %s\n", blinksPath); return 1; }
    std::string line;
    readLine(bf, line);
    std::vector<std::string> hdr = split(line);
    int cTs = -1, cBlink = -1;
    for (int i = 0; i < (int)hdr.size(); i++) {
      if (hdr[i] == "timestamp_ms") cTs = i;
      if (hdr[i] == "blink") cBlink = i;
    }
    if (cTs < 0 || cBlink < 0) { fprintf(stderr, "ERROR: %s needs timestamp_ms and blink columns\n", blinksPath); return 1; }
    while (readLine(bf, line)) {
      std::vector<std::string> v = split(line);
      if ((int)v.size() > cBlink && v[cBlink] == "1") blinkTs.push_back((uint32_t)strtoul(v[cTs].c_str(), nullptr, 10));
    }
    fclose(bf);
  }

  FILE *in = fopen(argv[1], "r");
  FILE *out = fopen(argv[2], "w");
  if (!in || !out) { fprintf(stderr, "ERROR: cannot open input/output\n"); return 1; }

  FuzzyFatigue fis;
  AlertGate gate;
  int gatedSeconds[3] = {0, 0, 0};
  EyeBlinkEAR::BlinkRateWindow rate;
  size_t nextBlink = 0;
  float baselineBPM = fixedBaseline;
  HrBaseline hrBase;
  bool baselineFormed = fixedBaseline > 0.0f;
  int alertSeconds[3] = {0, 0, 0};
  int rows = 0, alertMismatch = 0;
  float maxRiskDiff = 0.0f, blinkSum = 0.0f;

  uint32_t firstNow = 0;
  bool haveFirst = false;
  std::string line;
  while (readLine(in, line)) {
    std::vector<std::string> v = split(line);
    // SD CSV has no header; skip anything malformed. 18 columns = sessions
    // recorded before blink_valid existed (blink assumed valid), 19 = with
    // blink_valid, 20 = current (adds alert_gated, recomputed here from the raw alert).
    if (v.size() < 18 || v.size() > 20) continue;
    uint32_t now = (uint32_t)strtoul(v[0].c_str(), nullptr, 10);
    if (!haveFirst) { firstNow = now; haveFirst = true; }
    int hrOut = atoi(v[1].c_str());
    int signalQuality = atoi(v[10].c_str());
    const bool blinkValid = (v.size() >= 19) ? v[18] == "1" : true;

    float blinkRate;
    if (blinksPath) {
      while (nextBlink < blinkTs.size() && blinkTs[nextBlink] <= now) rate.push(blinkTs[nextBlink++]);
      // Same 3-minute smoothed rate the device feeds the model, measured from the
      // first CSV row (the device measures from the eye lock, a little earlier).
      blinkRate = rate.smoothedBpm(now, firstNow);
    } else {
      blinkRate = (float)atof(v[11].c_str());
    }

    // Baseline HR: main.cpp loop(), count-based, frozen once formed. The CSV
    // carries hr_out (integer) where the device used the float currentBPM.
    if (!baselineFormed && signalQuality == 1 && hrOut > 0) {
      if (hrBase.push((float)hrOut)) { baselineBPM = hrBase.bpm; baselineFormed = true; }
    }
    float hrDiffPct = 0.0f;
    if (baselineFormed && hrOut > 0) hrDiffPct = (hrOut - baselineBPM) / baselineBPM * 100.0f;

    float risk;
    int alert;
    fis.update(hrDiffPct, blinkRate,
               (float)atof(v[13].c_str()),    // gyro_var
               (float)atof(v[12].c_str()),    // pitch_deg
               (float)atof(v[14].c_str()),    // nod_score
               risk, alert, blinkValid);

    const int gated = gate.update(alert, now);
    if (gated >= 0 && gated <= 2) gatedSeconds[gated]++;

    // Same columns and formats as main.cpp's SD row; untouched fields copied.
    fprintf(out, "%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%.2f,%s,%s,%s,%.2f,%d,%s,%d,%d\n",
            v[0].c_str(), v[1].c_str(), v[2].c_str(), v[3].c_str(), v[4].c_str(),
            v[5].c_str(), v[6].c_str(), v[7].c_str(), v[8].c_str(), v[9].c_str(),
            v[10].c_str(), blinkRate, v[12].c_str(), v[13].c_str(), v[14].c_str(),
            risk, alert, v[17].c_str(), blinkValid ? 1 : 0, gated);

    rows++;
    blinkSum += blinkRate;
    if (alert >= 0 && alert <= 2) alertSeconds[alert]++;
    if (alert != atoi(v[16].c_str())) alertMismatch++;
    float d = risk - (float)atof(v[15].c_str());
    if (d < 0) d = -d;
    if (d > maxRiskDiff) maxRiskDiff = d;
  }
  fclose(in);
  fclose(out);

  printf("rows                 : %d\n", rows);
  printf("blink source         : %s\n", blinksPath ? blinksPath : "recorded blink_rate column");
  printf("HR baseline          : %.1f BPM (%s)\n", baselineBPM,
         fixedBaseline > 0.0f ? "given" : "median of good rows 11-30");
  printf("mean blink_rate      : %.1f /min\n", rows ? blinkSum / rows : 0.0f);
  printf("seconds SAFE/WARN/CRIT: %d / %d / %d\n", alertSeconds[0], alertSeconds[1], alertSeconds[2]);
  printf("gated SAFE/WARN/CRIT  : %d / %d / %d s (dwell 5/8 s, hold 3 s)\n", gatedSeconds[0], gatedSeconds[1], gatedSeconds[2]);
  printf("vs recorded CSV      : alert differs on %d rows, max |risk diff| %.2f\n", alertMismatch, maxRiskDiff);
  return 0;
}
