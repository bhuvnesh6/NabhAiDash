# NabhAI RTS-04 — Complete Technical Documentation

> Version 2.0 · Includes Thrust Estimation Module  
> System: Browser-native, no server required  
> Sensors: Webcam (vision) + Microphone (audio)

---

## Table of Contents

1. [System Overview](#1-system-overview)
2. [Architecture](#2-architecture)
3. [Calibration Pipeline](#3-calibration-pipeline)
4. [Vision Analysis Pipeline](#4-vision-analysis-pipeline)
5. [Audio Analysis Pipeline](#5-audio-analysis-pipeline)
6. [Thrust Estimation Model](#6-thrust-estimation-model)
7. [Burn State Machine](#7-burn-state-machine)
8. [Stability Scoring](#8-stability-scoring)
9. [Charting Engine](#9-charting-engine)
10. [UI System](#10-ui-system)
11. [CPU/GPU Load Reporting](#11-cpugpu-load-reporting)
12. [Performance Design](#12-performance-design)
13. [Known Limitations & Calibration Guide](#13-known-limitations--calibration-guide)
14. [Variables Quick-Reference](#14-variables-quick-reference)

---

## 1. System Overview

NabhAI RTS-04 is a **browser-native telemetry dashboard** for solid-motor static fire tests. It derives every metric live from two browser-accessible sensors:

| Sensor | Browser API | What it feeds |
|---|---|---|
| Webcam | `getUserMedia` → `<video>` | Brightness, flame geometry, smoke, vibration, visual stability |
| Microphone | `getUserMedia` → `Web Audio API` | dB SPL, dominant frequency, FFT, acoustic stability |

No backend, no WebSockets, no native app. Everything runs in a single HTML file inside a `requestAnimationFrame` loop.

### What is and isn't measured

| Metric | Source | Is it a calibrated SI value? |
|---|---|---|
| Brightness | Camera luminance average | No — relative (0–100 %) |
| Flame area | Pixel count after HSV threshold | No — relative (px²) |
| Flame length | Bounding-box height | No — relative (px) |
| Smoke density | Desaturated-gray pixel fraction | No — relative (%) |
| Vibration frequency | Zero-crossing rate of motion energy | Approximate (Hz) |
| Audio dB SPL | RMS of mic time-domain signal | Relative — offset-calibrated against ambient |
| Dominant frequency | FFT peak with sub-bin interpolation | Yes (Hz) — accurate to within ~1 Hz |
| Stability score | Motion-energy derivative | No — dimensionless index |
| Acoustic stability | Frequency standard deviation | No — dimensionless index |
| **Estimated Thrust** | **Weighted fusion of above** | **No — relative-N. Scale with load cell to get SI** |

---

## 2. Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│  Browser Tab                                                    │
│                                                                 │
│  getUserMedia()                                                 │
│    ├─ Video stream ──────→ <video> element ────→ workCanvas     │
│    └─ Audio stream ──────→ AudioContext                         │
│                              └─ AnalyserNode                    │
│                                                                 │
│  requestAnimationFrame loop (mainLoop)                          │
│    │                                                            │
│    ├─ [every 70 ms]  runVisionAnalysis()                        │
│    │    ├─ drawImage(video) → 128×72 px workCanvas              │
│    │    ├─ getImageData() → pixel array                         │
│    │    ├─ Per-pixel HSV classification → flame / smoke         │
│    │    ├─ Frame-diff motion energy → vibration proxy           │
│    │    ├─ computeThrust() → T_est                              │
│    │    └─ updateVisionUI() + updateThrustUI()                  │
│    │                                                            │
│    ├─ [every 45 ms]  runAudioAnalysis()                         │
│    │    ├─ getByteTimeDomainData() → RMS → dB                   │
│    │    ├─ getByteFrequencyData() → FFT → dominant Hz           │
│    │    └─ Acoustic stability via freq std-dev                  │
│    │                                                            │
│    └─ [every 130 ms] Chart sampling + render                   │
│                                                                 │
│  setInterval(60 ms)   Burn timer display                        │
│  setInterval(1000 ms) System clock + CPU/GPU load               │
└─────────────────────────────────────────────────────────────────┘
```

---

## 3. Calibration Pipeline

### Why calibration?

Every camera and microphone is different. A phone camera in a bright lab produces very different baseline luminance than a laptop camera in a dim room. Without calibration, fixed thresholds would either miss the burn entirely or trigger on ambient noise.

### How it works

When the stream starts, the system enters **calibration mode** (`burnStage = -1`) for **1400 ms**.

During this window, every frame calls `sampleCalibrationFrame()`:

```
calib.lum = running_average(luminance of each frame)
calib.db  = running_average(dB of each audio tick)
```

**Luminance average (per frame):**
```
lum_pixel = 0.299·R + 0.587·G + 0.114·B   (ITU-R BT.601 luma)
avgLum    = Σ(lum_pixel) / (W × H)
calib.lum = (calib.lum × N + avgLum) / (N + 1)   [online mean]
```

**Audio RMS (per tick):**
```
sample_normalised = (sample_byte - 128) / 128
rms = sqrt( Σ(sample²) / N )
dB  = 20 · log10(max(rms, 0.0001)) + 100   [+100 offset keeps value positive]
calib.db = online mean of dB samples
```

After 1400 ms, `finishCalibration()` sets `calib.ready = true`, transitions to **Standby** (`burnStage = 0`), and logs the locked baseline values.

### Why 1400 ms?

At 14 fps analysis rate, that gives ~20 frames — enough to average out any startup transients in the camera's auto-exposure and the mic's AGC settling.

---

## 4. Vision Analysis Pipeline

Called every **70 ms** (≈ 14 fps analysis rate — the camera may capture at 30/60 fps but we only need 14 for reliable flame tracking).

### Step 1 — Frame capture

```javascript
workCtx.save();
workCtx.scale(-1, 1);                     // Mirror to cancel the CSS transform on <video>
workCtx.drawImage(video, -W, 0, W, H);   // Downscale to 128 × 72
workCtx.restore();
const data = workCtx.getImageData(0, 0, W, H).data;  // RGBA flat array
```

The video element uses `transform: scaleX(-1)` for the user-facing display. We apply the same mirror in the offscreen canvas so coordinates are consistent.

### Step 2 — Per-pixel classification

For each pixel, we compute HSV (Hue/Saturation/Value) from the RGB bytes. HSV is used because it separates the *colour* of the flame from its *brightness*, making detection robust across exposure levels.

**RGB → HSV conversion:**
```
maxC = max(R, G, B)  ;  minC = min(R, G, B)  ;  d = maxC − minC
val  = maxC / 255
sat  = (maxC == 0) ? 0 : d / maxC
hue  = 60 · sector_formula (see below)   [0–360°]
```

**Flame classification:**
```
IS_FLAME if:
  (hue ≤ 55° OR hue ≥ 345°)   ← warm colour band: red → amber → yellow
  AND sat > 0.32               ← reasonably saturated (rules out white overexposure)
  AND val > 0.42               ← sufficiently bright
```

**Smoke classification:**
```
IS_SMOKE if:
  sat < 0.13    ← near-grey
  AND val > 0.16 AND val < 0.62   ← mid-brightness band (not black, not white)
```

For each flame pixel:
- Increment `flameCount`
- Track bounding box: `flameMinX`, `flameMaxX`, `flameMinY`, `flameMaxY`

### Step 3 — Motion energy (vibration proxy)

Frame differencing compares the current frame against the previous one stored in `bufA`:

```
motionEnergy = Σ |lum_current(i) − lum_previous(i)| / (W × H)
```

This gives a per-frame average absolute luminance change. High values = lots of motion = likely vibration or rapid brightness oscillation.

### Step 4 — Vibration frequency (zero-crossing rate)

Motion energy is stored in a **48-sample ring buffer** (`vibHistory`). To extract a vibration frequency:

1. Compute the mean of the buffer.
2. Count zero-crossings (transitions of the signal across the mean).
3. `vibFreq ≈ (crossings / 2) / window_seconds`

This gives the dominant oscillation frequency of the motion signal in Hz. The EMA smoothing constant of 0.3 prevents jitter.

### Step 5 — Smooth outputs

All raw values are smoothed with **Exponential Moving Averages (EMA)** to prevent jumpy UI:

```
smoothBright    = lerp(smoothBright,    avgLum,      0.20)
smoothFlameArea = lerp(smoothFlameArea, flameCount,  0.28)
smoothFlameLen  = lerp(smoothFlameLen,  flameLenPx,  0.28)
smoothSmoke     = lerp(smoothSmoke,     smokeFrac×100, 0.16)
```

`lerp(a, b, t) = a + (b − a) × t` — larger `t` = faster response, more noise.

### Step 6 — Frame reticle positioning

If `flameCount > 5`, the cyan bounding-box reticle and centroid dot are shown. Their screen positions are computed by mapping the analysis-grid coordinates (0–128, 0–72) back to the CSS-pixel position of the rendered video element, accounting for `object-fit: contain` letterboxing:

```
scale = min(window.innerWidth / vidWidth, window.innerHeight / vidHeight)
offsetX = (window.innerWidth  - vidWidth  × scale) / 2
offsetY = (window.innerHeight - vidHeight × scale) / 2
screen_x = offsetX + flame_x × (scale × vidWidth / W)
screen_y = offsetY + flame_y × (scale × vidHeight / H)
```

---

## 5. Audio Analysis Pipeline

Called every **45 ms** (≈ 22 Hz).

### dB SPL (relative)

```javascript
analyser.getByteTimeDomainData(timeData);  // 1024 samples, 0–255 bytes
sample_norm = (byte − 128) / 128           // map to −1 … +1
rms = sqrt( Σ(sample_norm²) / 1024 )
dB  = 20 · log10(max(rms, 0.0001)) + 100
```

The `+100` offset keeps values comfortably positive (a silent mic might read ~60 dB on this scale; loud exhaust might read ~95–115 dB). The peak is tracked as a running maximum (`dbPeakVal`).

The meter fill percentage:
```
fill% = clamp((dB / 130) × 100, 0, 100)
```

### FFT — Dominant Frequency

```javascript
analyser.getByteFrequencyData(freqData);  // 512 bins, 0–255 amplitude
```

The FFT size is 1024, giving 512 frequency bins. Each bin `i` represents the frequency:
```
f_i = (i / 512) × (sampleRate / 2)   [Hz]
```

We find the peak bin, then apply **parabolic interpolation** for sub-bin precision:

```
y1 = freqData[peak−1]
y2 = freqData[peak]
y3 = freqData[peak+1]
interp_bin = peak + 0.5 × (y1 − y3) / (y1 − 2y2 + y3)
domHz = (interp_bin / 512) × nyquist
```

This gives ~1 Hz accuracy on a 44100 Hz sample rate system.

### Acoustic Stability

A 32-sample ring buffer stores the dominant frequency history. The **standard deviation** of that buffer is computed:

```
mean  = Σ(freq_hist) / N
stdev = sqrt( Σ((freq_i − mean)²) / N )
acoustic_target = clamp(100 − stdev × 0.4, 0, 100)
acoustic = lerp(acoustic, acoustic_target, 0.12)
```

A low stdev (stable frequency) → high acoustic stability score. A rapidly oscillating dominant frequency → low score. This is a proxy for whether the exhaust tone is stable or chaotic.

### FFT Visualisation

The 512 FFT bins are grouped into **32 display bars** (16 bins per bar). Each bar's height is:

```
bar_height = (Σ(freqData[bin]) / binsPerBar) / 255 × canvas_height
```

Bars use a gradient from cyan (top) to near-transparent (bottom) to give a glow effect.

### Waveform Visualisation

The raw 1024-byte time-domain buffer is plotted as a line directly:

```
y = (height / 2) + (sample_norm × height / 2 × 0.9)
```

---

## 6. Thrust Estimation Model

### Physical rationale

For a solid-propellant motor, thrust correlates with three observable proxy signals:

| Signal | Physical link | Sensor source |
|---|---|---|
| Flame area | Combustion surface area → burn rate → mass flow rate → thrust | Camera (HSV pixel count) |
| Acoustic SPL above ambient | Exhaust gas kinetic energy radiates as sound. Louder = more energy. | Microphone (RMS → dB) |
| Vibration frequency | Structural response to impulsive combustion pressure → motor mount strain | Camera (motion zero-crossing rate) |

None of these are pressure transducers or load cells. They are *correlated proxies*, not direct measurements. **The output is in relative-Newton units (rel-N), not calibrated SI Newtons.**

### Model equation

```
T_raw = α · A_norm + β · dB_norm + γ · V_norm

where:
  A_norm  = clamp(smoothFlameArea / FLAME_MAX_REF,  0, 1)
  dB_norm = clamp((smoothDb − calib.db) / DB_RANGE, 0, 1)
  V_norm  = clamp(vibFreq / VIB_FREQ_MAX,            0, 1)

  α = 0.55   (flame area weight)
  β = 0.32   (acoustic weight)
  γ = 0.13   (vibration weight)

T_est = T_raw × THRUST_SCALE    [rel-N]
```

**Constants:**

| Constant | Default | Meaning |
|---|---|---|
| `FLAME_MAX_REF` | 500 px | Flame pixel count at "full burn" — tune per camera/distance |
| `DB_RANGE` | 45 dB | dB above ambient considered max thrust acoustic level |
| `VIB_FREQ_MAX` | 25 Hz | Upper vibration proxy limit |
| `THRUST_SCALE` | 150 | Maps 0–1 composite → 0–150 rel-N |
| `THRUST_ALPHA` | 0.55 | Flame area contribution |
| `THRUST_BETA` | 0.32 | Acoustic contribution |
| `THRUST_GAMMA` | 0.13 | Vibration contribution |

### EMA smoothing

```
smoothThrust = lerp(smoothThrust, T_raw_N, 0.22)
```

An EMA of 0.22 gives a time constant of approximately **4 analysis frames = 280 ms**, which prevents impulse noise from spiking the reading while still responding fast enough to track burn dynamics.

### Peak tracking

```javascript
if (burnStage >= 1 && smoothThrust > thrustPeak) thrustPeak = smoothThrust;
```

Peak is only tracked after ignition is detected (stage ≥ 1).

### Impulse integration (Total Impulse, N·s)

Impulse is the time-integral of thrust:

```
I = ∫ T dt  ≈  Σ (smoothThrust_k × Δt)
```

Where `Δt = VISION_INTERVAL / 1000 = 0.070 s`.

This runs only during active burn (`burnStage >= 1`):

```javascript
thrustAccumulator += smoothThrust * (VISION_INTERVAL / 1000);
```

### How to calibrate to real Newtons

1. Run a static fire where you have **both** a load cell and the NabhAI dashboard running simultaneously.
2. Note the load cell's peak thrust `F_real` (Newtons).
3. Note NabhAI's `thrustPeak` at the same moment (rel-N).
4. Set:
   ```
   THRUST_SCALE_calibrated = F_real / (T_raw_peak_at_that_moment / THRUST_SCALE)
   ```
5. Update `THRUST_SCALE` in the source code. All subsequent runs at similar camera distance and framing will now read in real Newtons.

### Why flame area has the highest weight (α = 0.55)

In a solid motor, the instantaneous thrust is:

```
F = ṁ · Ve + (Pe − Pa) · Ae
```

`ṁ` (mass flow) is proportional to the burning surface area of the grain. The camera is directly observing the exhaust plume area, which is the closest visual proxy to burning surface. Acoustic and vibration signals are secondary effects of the same phenomenon but are more affected by environment (room acoustics, mounting stiffness) so they receive lower weights.

---

## 7. Burn State Machine

The system maintains a `burnStage` integer:

| Value | State | Trigger to enter | Visual |
|---|---|---|---|
| -1 | Calibrating | App startup | Amber pulsing dot |
| 0 | Standby | Calibration complete | Grey ring |
| 1 | Ignition | Flame/brightness flash detected | Amber ring |
| 2 | Stable Burn | Flame sustained > 650 ms | Green ring |
| 3 | Shutdown | Flame lost > 1400 ms | Ice ring |

### Ignition detection

```javascript
lumDelta = avgLum − calib.lum
hasIgnitionFlash = (lumDelta > max(18, calib.lum × 0.3) AND flameFrac > 0.0006)
                   OR flameFrac > 0.003
```

The adaptive threshold `max(18, calib.lum × 0.3)` means:
- In a dark environment: requires 18 units of brightness increase (absolute).
- In a bright environment: requires 30% brightness increase (relative).

This prevents false triggers in any ambient lighting.

### Stable burn confirmation

After ignition is detected, a **650 ms hold** is required before declaring Stable Burn. This prevents a brief flash (igniter, not propellant) from being classified as a full burn.

### Shutdown detection

If the flame disappears (`flameFrac < 0.0014`) during or after ignition, a **1400 ms loss timer** starts. If flame does not reappear within that window, the burn is declared complete. This tolerates momentary occlusion (smoke, camera shake) without premature shutdown calls.

### Thrust during state transitions

Thrust estimation only activates at `burnStage >= 1`. In Standby, `computeThrust()` returns 0, preventing the impulse accumulator from running up on ambient noise.

---

## 8. Stability Scoring

### Visual Stability (0–100 %)

Measures how "calm" the visual scene is — a proxy for structural vibration severity or unexpected events.

```
motionDelta = |motionEnergy − motionEnergy_previous|
volatility  = clamp(motionDelta × 8, 0, 60)
presenceBonus = (flameFrac > 0.002) ? 25 : 0
target = clamp(100 − volatility + presenceBonus × 0.3, 0, 100)
stability = lerp(stability, target, 0.10)
```

The `presenceBonus` slightly rewards the presence of a stable flame (a detected, steady flame means *organised* motion, not random vibration).

Colour coding: 
- > 70 % → Cyan "NOMINAL RANGE"  
- 40–70 % → Amber "MARGINAL"  
- < 40 % → Red "UNSTABLE"

### Acoustic Stability (0–100 %)

```
stdev = standard deviation of last 32 dominant frequency samples
target = clamp(100 − stdev × 0.4, 0, 100)
acoustic = lerp(acoustic, target, 0.12)
```

A stable exhaust tone → low frequency drift → low stdev → high score.

---

## 9. Charting Engine

All charts use **fixed-size Float32Array ring buffers** — no array shifting, no garbage collection on push.

```javascript
chart.buf[chart.head] = val;
chart.head = (chart.head + 1) % chart.n;   // wrap around
```

**Drawing algorithm:**
1. Find `min` and `max` of the current buffer.
2. Add 15% padding above and below.
3. Map each value to a Y coordinate: `y = h − ((v − min) / (max − min)) × h × 0.84 − h × 0.05`
4. Draw a filled gradient area below the line.
5. Draw the line on top.
6. Draw a glowing dot at the latest point.

**Why not use Chart.js?**  
Zero dependencies = zero network requests, zero library version conflicts, and full control over render timing. The custom engine adds < 200 lines of code and redraws in < 0.5 ms per chart.

**Chart update rate:** ~7–8 fps (every 130 ms). This is intentionally lower than analysis rate — human eyes don't benefit from faster chart updates, and it saves significant canvas compositing work.

---

## 10. UI System

### Layout

```
┌─────────────────────────────────────────────────────────┐
│  Topnav (54 px)                                         │
├──────────────┬─────────────────────────┬────────────────┤
│ Left column  │  Center spacer          │ Right column   │
│ (264 px)     │  (fills remaining)      │ (280 px)       │
│              │                         │                │
│ Burn status  │  HUD chips (top)        │ Thrust card    │
│ Timer        │  Status banner          │ Audio analysis │
│ Stability    │  Flame reticle          │ Dom. frequency │
│ Health       │  Centroid dot           │ Acoustic stab. │
│ Events       │  HUD chips (bottom)     │                │
├──────────────┴─────────────────────────┴────────────────┤
│  Bottom bar (198 px): 5 charts + console                │
└─────────────────────────────────────────────────────────┘
```

### CSS variable system

All colours are defined as CSS custom properties on `:root`, allowing a complete dark-mode palette with no overrides needed:

| Variable | Hex | Role |
|---|---|---|
| `--void` | `#040608` | Page background |
| `--cyan` | `#00E5FF` | Primary accent, vision metrics |
| `--emerald` | `#3CF0A8` | Healthy / OK states |
| `--amber` | `#FFB627` | Warning / ignition states |
| `--red` | `#FF4757` | Alert / unstable states |
| `--ice` | `#7FDBFF` | Secondary accent, frequency |
| `--violet` | `#B48EFF` | **Thrust metrics (new)** |
| `--text-dim` | `#5E7E8F` | Label text |
| `--text-mid` | `#A9C7D4` | Body text |

### Glass morphism

Every panel uses the `.glass` class:

```css
background: linear-gradient(165deg, rgba(10,17,24,0.74), rgba(4,7,10,0.82));
border: 1px solid rgba(127,219,255,0.12);
backdrop-filter: blur(14px);
```

This creates the frosted-glass effect while remaining transparent enough to see the camera feed behind.

### HUD overlay positioning

The flame reticle (`crosshair-frame`) is positioned in CSS pixels using `getVideoRenderRect()` output — see Section 4, Step 6. It uses `position: absolute` within the center spacer.

### Typography

| Role | Font | Size |
|---|---|---|
| Brand / headings | Fraunces (serif, italic) | 13–23 px |
| Data values | JetBrains Mono | 10–32 px |
| Labels | JetBrains Mono (caps) | 7–10 px |
| Body text | Inter | 12–13 px |

---

## 11. CPU/GPU Load Reporting

These values are derived from the pipeline's own frame timing — not from browser APIs (which don't expose real CPU load to JavaScript).

```javascript
// In runVisionAnalysis:
const t0 = performance.now();
// ... all analysis ...
reportLoad(performance.now() − t0);

// Displayed every 1 second:
avgFrameMs = Σ(frameTimes) / frameTimesLen
cpuLoad% = clamp((avgFrameMs / 16.6) × 100, 4, 97)
gpuLoad% = clamp(cpuLoad × 1.25, 4, 97)
```

`16.6 ms = 1 frame at 60 fps`. If analysis takes 8 ms out of each 16.6 ms frame budget, that reads as 48% CPU.

GPU is estimated as 1.25× CPU because canvas 2D compositing has additional GPU overhead roughly proportional to CPU work on most implementations.

---

## 12. Performance Design

Every design decision was made to minimise main-thread CPU usage:

| Technique | Why |
|---|---|
| 128×72 analysis grid | 9,216 pixels vs 921,600 for 1280×720. 100× less data to process. |
| Throttled analysis intervals (70/45/130 ms) | Analysis doesn't need 60 fps. Charts need even less. |
| Ring buffer push (O(1)) | No `Array.shift()` or `.push()` — no GC pressure. |
| Pre-allocated `Uint8ClampedArray` for pixel buffers | Zero allocation in the hot loop. |
| `willReadFrequently: true` on workCanvas | Hints the browser to keep pixel data CPU-side. |
| Lazy chart draw (only when `dirty` flag set) | Charts only redraw when new data arrives. |
| `devicePixelRatio` clamped to 2 | Prevents 3× canvas size on high-DPI devices. |
| DOM refs cached in `UI` object | No `getElementById` inside hot loops. |
| Separate timer interval (60 ms) | Burn timer display doesn't need to run inside rAF. |

---

## 13. Known Limitations & Calibration Guide

### Flame detection caveats

- **Works best in dim environments** where the flame colour contrast against the background is high.
- **Bright sunlight or white backgrounds** can cause false negatives (flame blends with background) or reduce sensitivity (saturation threshold too tight).
- **Recommended framing:** Centre the motor nozzle in the lower half of the frame. The flame should occupy at least 1–2% of the frame at peak thrust.
- **Distance matters:** The `FLAME_MAX_REF = 500` constant assumes a camera positioned such that full-burn flame fills roughly 0.5% of the 128×72 grid. If you're further away, reduce this value.

### Audio caveats

- **Disable all microphone processing** (`echoCancellation: false`, `noiseSuppression: false`, `autoGainControl: false`) — already set in the code. If your OS or browser overrides these, the dB calibration will drift.
- **Point the microphone toward the motor**, not away. A directional mic improves the acoustic correlation to thrust.
- The **2-second impulse response of the acoustic stability** (EMA 0.12, 32-sample buffer) means it won't catch sub-second frequency instabilities.

### Thrust estimation caveats

| Issue | Impact | Fix |
|---|---|---|
| Camera angle changes between runs | A_norm changes non-linearly | Fix camera mount, recalibrate FLAME_MAX_REF |
| Ambient light changes (cloud cover, doors opening) | calib.lum is set once at startup | Restart stream to re-calibrate |
| Microphone saturation at high SPL | dB readout clamps at 130 | Use a lower-sensitivity mic or increase distance |
| Multiple ignition flashes | Stage may transition 0→1→0→1 | 650 ms hold before Stable prevents most false positives |

### Improving thrust accuracy

The current model is a **linear proxy**. Accuracy can be improved by:

1. **Empirical calibration:** Run 3–5 fires with a simultaneous load cell. Fit a regression of `T_real` vs `T_raw` and replace the linear model with the regression curve.
2. **Luminosity weighting:** Add `smoothBright / 255` as a fourth term. Flame brightness correlates with combustion temperature, which correlates with exhaust velocity and thus specific impulse.
3. **Frequency band gating:** Only count acoustic dB in the 200–2000 Hz band (solid motor exhaust fundamental) to reject low-frequency wind and high-frequency electrical noise.

---

## 14. Variables Quick-Reference

### Analysis constants

| Constant | Value | Description |
|---|---|---|
| `W` | 128 | Analysis canvas width (px) |
| `H` | 72 | Analysis canvas height (px) |
| `CALIB_MS` | 1400 | Calibration window duration (ms) |
| `VISION_INTERVAL` | 70 | ms between vision analysis ticks |
| `AUDIO_INTERVAL` | 45 | ms between audio analysis ticks |
| `CHART_INTERVAL` | 130 | ms between chart data samples |
| `VIB_HIST_N` | 48 | Vibration ring buffer length (frames) |
| `analyser.fftSize` | 1024 | FFT size → 512 frequency bins |
| `analyser.smoothingTimeConstant` | 0.35 | FFT temporal smoothing |

### Flame detection thresholds

| Constant | Value | Description |
|---|---|---|
| Hue range | ≤ 55° or ≥ 345° | Warm colour band (red→yellow) |
| Min saturation | 0.32 | Rules out white/overexposed pixels |
| Min value | 0.42 | Rules out dark pixels |
| Min flame fraction | 0.0014 | Below this = no flame present |
| Ignition brightness delta | max(18, lum×0.3) | Adaptive ignition threshold |

### Thrust model constants

| Constant | Value | Description |
|---|---|---|
| `THRUST_ALPHA` | 0.55 | Flame area weight |
| `THRUST_BETA` | 0.32 | Acoustic weight |
| `THRUST_GAMMA` | 0.13 | Vibration weight |
| `THRUST_SCALE` | 150 | rel-N full-scale output |
| `FLAME_MAX_REF` | 500 | px count = full thrust flame |
| `DB_RANGE` | 45 | dB above ambient = max thrust level |
| `VIB_FREQ_MAX` | 25 | Hz upper vibration limit |

### EMA smoothing constants

| Signal | Alpha (t) | Approx. time constant |
|---|---|---|
| Brightness | 0.20 | ~5 frames = 350 ms |
| Flame area | 0.28 | ~3.5 frames = 245 ms |
| Flame length | 0.28 | ~3.5 frames = 245 ms |
| Smoke | 0.16 | ~6 frames = 420 ms |
| Vibration | 0.30 | ~3 frames = 210 ms |
| Stability | 0.10 | ~10 frames = 700 ms |
| Acoustic | 0.12 | ~8 frames = 560 ms (audio ticks) |
| Thrust | 0.22 | ~4 frames = 280 ms |

*Time constant = `VISION_INTERVAL / alpha` for vision-tick signals.*

---

*End of documentation — NabhAI RTS-04 v2.0*