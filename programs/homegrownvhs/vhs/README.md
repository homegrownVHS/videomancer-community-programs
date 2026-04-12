# VHS

VHS simulates the playback artifacts of a worn VHS tape for Videomancer. It
combines horizontal chroma blur, luma noise, per-scanline tracking jitter,
head-switch noise, random scanline dropouts, and color desaturation into a
single processing pipeline with a dry/wet mix fader.

## OVERVIEW

VHS recreates the characteristic degradation of analog VHS tape playback by
layering six distinct artifact types:

- **Chroma Blur** — An IIR lowpass filter on the U and V channels simulates
  the limited chroma bandwidth of the VHS format. At high settings the color
  information smears horizontally while luma stays sharp, exactly like real
  tape playback.

- **Noise** — Pseudo-random values from a 16-bit LFSR are scaled and added to
  the signal. At low levels this mimics the fine grain of tape hiss. At high
  levels it overwhelms the image with static.

- **Tracking Jitter** — A BRAM scanline buffer stores luma and reads it back
  at a per-line random offset, displacing pixels horizontally. Two modes are
  available: Scattered (every line jitters independently) and Band (jitter
  concentrates in a rolling band that drifts across the frame).

- **Head Switch** — A noise band appears at the bottom (or top) of the frame,
  simulating the distortion where the VCR head drum transitions between
  fields. The band width is adjustable.

- **Dropout** — Random scanlines are replaced entirely with snow (LFSR noise)
  or black, simulating oxide flaking and tape damage.

- **Color Loss** — The U and V channels are blended toward neutral (512),
  desaturating the image as if the tape's chroma signal has degraded with age.

> **NOTE**
> VHS uses 5 block RAMs for the Y scanline buffer (10-bit × 2048 entries)
> and approximately 5,270 logic cells. The design achieves ~43 MHz Fmax.

## QUICK START

1. Push the **Mix** fader (Fader 12) to full.
2. Turn **Chroma Blur** (Knob 1) to around 50%. You should immediately see
   color smearing on sharp edges.
3. Add a touch of **Noise** (Knob 2) at 10–25% for background tape hiss.
4. Bring up **Tracking** (Knob 3) slowly from zero. Even a small amount
   introduces visible horizontal jitter.
5. Set **Head Switch** (Knob 4) to 10–15% to add a noise band at the bottom
   of the frame.
6. Sprinkle in **Dropout** (Knob 5) at 5–10% for occasional line dropouts.
7. Optionally increase **Color Loss** (Knob 6) to fade the colors.

## PARAMETERS

### Knob 1 — Chroma Blur

| | |
|---|---|
| Range | 0.0% – 100.0% |
| Default | 50.0% |

Chroma Blur controls the cutoff frequency of a pair of IIR lowpass filters
running on the U and V chroma channels. At 0% no filtering is applied and
chroma passes through unmodified. As the knob increases, the filter smooths
more aggressively, spreading color information across adjacent pixels. At 100%
the chroma is heavily smeared, recreating the look of worn-out tape or
multi-generation dubbing.

The luma (Y) channel is not affected by this control — it passes through the
BRAM scanline buffer path instead.

### Knob 2 — Noise

| | |
|---|---|
| Range | 0.0% – 100.0% |
| Default | 25.0% |

Noise sets the amplitude of pseudo-random values added to the signal. The noise
source is a free-running 16-bit LFSR. At 0% no noise is added. Higher values
increase the peak deviation of each pixel. Whether noise is applied to luma
only or to all three channels depends on the Color Noise switch (Switch 9).

### Knob 3 — Tracking

| | |
|---|---|
| Range | 0.0% – 100.0% |
| Default | 0.0% |

Tracking controls the magnitude of per-scanline horizontal displacement. Each
active line samples a random offset from the LFSR, scaled by this knob, and
uses it to shift the BRAM read address. The result is that luma pixels are
displaced left per line by a random amount.

At 0% there is no displacement. Low values (5–15%) produce subtle jitter
reminiscent of a slightly misadjusted tracking knob. High values produce
dramatic diagonal tearing.

The distribution of jitter across lines depends on the Track Mode switch
(Switch 7): Scattered applies jitter to every line, while Band concentrates
it in a rolling region that drifts frame-to-frame.

### Knob 4 — Head Switch

| | |
|---|---|
| Range | 0.0% – 100.0% |
| Default | 12.5% |

Head Switch controls the height of a noise band at the edge of the frame. On
a real VCR, the head switch occurs as the spinning drum transitions between
heads, producing a visible distortion bar typically at the bottom of the
picture. At 0% no head-switch effect is applied. Higher values expand the
noisy region to cover more lines.

The position of the band (top or bottom of frame) is controlled by the Head
Pos switch (Switch 10).

### Knob 5 — Dropout

| | |
|---|---|
| Range | 0.0% – 100.0% |
| Default | 6.3% |

Dropout controls the probability of random full-line dropouts. At the start of
each active scanline, the LFSR is compared against this threshold. Lines that
trigger a dropout are replaced entirely — either with white noise (snow) or
black, depending on the Drop Style switch (Switch 8).

At 0% no dropouts occur. At low values (5–10%) occasional lines flash to snow,
simulating oxide flaking on aged tape. At high values the image disintegrates
into mostly noise.

### Knob 6 — Color Loss

| | |
|---|---|
| Range | 0.0% – 100.0% |
| Default | 0.0% |

Color Loss blends the U and V chroma channels toward their neutral midpoint
(512), progressively desaturating the image. At 0% colors are unmodified. At
100% the chroma is fully suppressed and the image is monochrome. This simulates
deterioration of the tape's chroma signal over time or through repeated dubbing.

### Switch 7 — Track Mode

| | |
|---|---|
| Scattered | Every line gets independent random jitter |
| Band | Jitter concentrates in a rolling band |
| Default | Scattered |

Track Mode selects how tracking jitter is distributed across scanlines.

- **Scattered** — Each line samples its own random offset. The result is a
  uniform noise-like displacement across the entire frame.
- **Band** — Only lines within approximately 32 lines of a slowly drifting
  band center receive jitter. The band position advances by a small random
  amount each frame, producing the classic "rolling bar" tracking artifact.

### Switch 8 — Drop Style

| | |
|---|---|
| Snow | Dropout lines filled with LFSR noise |
| Black | Dropout lines filled with black |
| Default | Snow |

Drop Style selects the replacement content for dropout lines. Snow fills the
line with random luma values from the LFSR (white noise), which is the more
common VHS dropout appearance. Black fills with Y=0, U=512, V=512.

### Switch 9 — Color Noise

| | |
|---|---|
| Mono | Noise added to luma (Y) only |
| Color | Noise added to Y, U, and V channels |
| Default | Mono |

Color Noise controls whether the noise signal also affects the chroma channels.
In Mono mode, only the luma channel receives noise, producing monochrome grain
like real tape hiss. In Color mode, noise is also added to U and V (using
different LFSR bit fields), producing colored speckle reminiscent of heavily
degraded or multi-generation tape.

### Switch 10 — Sync Warp

| | |
|---|---|
| Off | No sync warp |
| On | Smooth raster displacement active |
| Default | Off |

Sync Warp adds a smooth, coherent horizontal displacement that varies across
the frame in a triangle-wave pattern, simulating the "flagging" or raster bow
that occurs when a VCR's horizontal timebase is losing sync lock. The
displacement amplitude scales with the Tracking knob (Knob 3) and is additive
with whatever jitter mode is active.

The wave phase drifts randomly frame-to-frame, creating an organic rolling
motion. At moderate Tracking values (10–20%) with Sync Warp On and Track Mode
set to Scattered, the result closely resembles a tape played on a VCR with
tenuous sync.

> **NOTE**
> Head-switch noise (Knob 4) always appears at the bottom of the frame.

### Switch 11 — Bypass

| | |
|---|---|
| Off | Effect active |
| On | Input passed through unchanged |
| Default | Off |

Bypass passes the input video directly to the output with no processing. The
signal is delayed by the same number of clocks as the processing pipeline (9)
to maintain sync alignment.

### Fader 12 — Mix

| | |
|---|---|
| Range | 0.0% – 100.0% |
| Default | 100.0% |

Mix crossfades between the dry (unprocessed) and wet (VHS-degraded) signal. At
0% the output is the clean original. At 100% the output is fully processed.
Intermediate positions blend the two, useful for dialing in subtle tape coloring
without committing to full degradation.

## SIGNAL FLOW

The processing pipeline runs at 9 clocks total (5 inline stages + 4 for the
output interpolators).

```
data_in ──┬──────────────────────────────────────────────────────┐
           │                                                      │
           ├──► IIR LPF (U) ──┐                                  │
           ├──► IIR LPF (V) ──┤                                  │
           │                   │                                  │
           ▼                   │                                  │
   ┌──────────────┐            │                                  │
   │   Stage 0    │  Register input, edge detect,                │
   │   1 clock    │  sample jitter offset & dropout flag         │
   └──────┬───────┘            │                                  │
          │                    │                                  │
          ▼                    │                                  │
   ┌──────────────┐            │                                  │
   │   Stage 1    │  BRAM write (Y at pixel_count)               │
   │   1 clock    │  BRAM read (Y at pixel_count − jitter)       │
   │              │  Capture IIR filter outputs (U, V)  ◄────────┘
   └──────┬───────┘                                               │
          │                                                       │
          ▼                                                       │
   ┌──────────────┐                                               │
   │   Stage 2    │  Latch jittered Y from BRAM                  │
   │   1 clock    │  Compute Y noise, UV noise                   │
   │              │  Head-switch zone detection                   │
   └──────┬───────┘                                               │
          │                                                       │
          ▼                                                       │
   ┌──────────────┐                                               │
   │   Stage 3    │  Apply dropout (snow/black) or:              │
   │   1 clock    │    add noise, head-switch noise,             │
   │              │    color loss desaturation                    │
   └──────┬───────┘                                               │
          │                                                       │
          ▼                                                       │
   ┌──────────────┐                                               │
   │   Stage 4    │  Output register (wet signal)                │
   │   1 clock    │                                               │
   └──────┬───────┘                                               │
          │                 ┌─────────┐                           │
          ├──── wet ───────▶│         │                           │
          │                 │ interp  │                           │
          │                 │  ×3     │◀── dry tap ──────────────┤
          │                 │ 4 clks  │     (5-clock delay)      │
          │                 └────┬────┘                           │
          │                      │                                │
          │                      ▼                                │
          │              ┌──────────────┐                         │
          │              │  Output Mux  │◀── bypass ─────────────┘
          │              │              │     (9-clock delay)
          │              └──────┬───────┘
          │                     │
          │                     ▼
          │                 data_out
          │
          └──── sync signals (hsync_n, vsync_n, field_n, avid)
                  delayed 9 clocks via shift register
```

### Signal Flow Notes

1. **IIR filters run ahead** — The `variable_filter_s` instances feed directly
   from `data_in.u` and `data_in.v` (combinational input). Their registered
   state updates on the same clock edge as Stage 0, so the low-pass output is
   available for capture in Stage 1.

2. **BRAM single-port** — The scanline buffer writes the current pixel's Y and
   reads the jittered address in the same clock (Stage 1). The read data
   emerges one clock later in Stage 2.

3. **Dry tap alignment** — The original Y/U/V data is delayed by 5 clocks (the
   inline processing depth) so it arrives at the interpolator inputs at the
   same time as the wet signal from Stage 4.

4. **Dropout replaces everything** — When a line is flagged for dropout, all
   per-pixel processing (noise, blur, color loss) is bypassed and the entire
   line is filled with snow or black.

5. **Tracking band drift** — In Band mode, the band position accumulates a
   small random offset per frame. This produces a slowly wandering distortion
   zone reminiscent of a VCR with marginal tracking lock.

6. **Sync warp** — When the Sync Warp toggle is On, a triangle-wave horizontal
   offset (approximately two undulations per field) is added to the per-line
   jitter. The wave's phase drifts by a random amount each frame, creating
   the characteristic rolling bow of tenuous sync.

## EXERCISES

### Exercise 1: Subtle VHS Coloring

#### Learning Outcomes

Add a gentle tape-era quality to clean video without destroying the image.

#### Video Source

Any clean video signal.

#### Steps

1. Set **Chroma Blur** (Knob 1) to about 40% for mild color smearing.
2. Set **Noise** (Knob 2) to 10% for background grain.
3. Leave **Tracking** (Knob 3) at 0%.
4. Leave **Head Switch** (Knob 4) at 0%.
5. Leave **Dropout** (Knob 5) at 0%.
6. Set **Color Loss** (Knob 6) to about 15% to slightly desaturate.
7. Push **Mix** (Fader 12) to 100%.

#### Settings

| | |
|---|---|
| Chroma Blur | 40% |
| Noise | 10% |
| Tracking | 0% |
| Head Switch | 0% |
| Dropout | 0% |
| Color Loss | 15% |
| Track Mode | Scattered |
| Drop Style | Snow |
| Color Noise | Mono |
| Sync Warp | Off |
| Bypass | Off |
| Mix | 100% |

### Exercise 2: Worn Rental Tape

#### Learning Outcomes

Simulate the look of a heavily-used rental VHS tape with visible tracking
problems, dropout lines, and head-switch noise.

#### Video Source

Any video signal. Works particularly well with footage that has visual interest
across the full frame.

#### Steps

1. Set **Chroma Blur** (Knob 1) to 70% for heavy color smearing.
2. Set **Noise** (Knob 2) to 25%.
3. Set **Tracking** (Knob 3) to 10% and set **Track Mode** (Switch 7) to
   **Band** for a rolling distortion bar.
4. Set **Head Switch** (Knob 4) to 20% for a visible noise band at the bottom.
5. Set **Dropout** (Knob 5) to 8% for occasional line dropouts.
6. Set **Color Loss** (Knob 6) to 30% for washed-out color.
7. Toggle **Color Noise** (Switch 9) to **Color** for chromatic speckle.
8. Push **Mix** to 100%.

#### Settings

| | |
|---|---|
| Chroma Blur | 70% |
| Noise | 25% |
| Tracking | 10% |
| Head Switch | 20% |
| Dropout | 8% |
| Color Loss | 30% |
| Track Mode | Band |
| Drop Style | Snow |
| Color Noise | Color |
| Sync Warp | On |
| Bypass | Off |
| Mix | 100% |

### Exercise 3: Tape Damage / Glitch Art

#### Learning Outcomes

Push the VHS artifacts to extremes for intentional glitch art.

#### Video Source

Any signal — graphic patterns and high-contrast footage are especially
effective.

#### Steps

1. Crank **Chroma Blur** (Knob 1) to 90%+ to completely dissolve color detail.
2. Set **Noise** (Knob 2) to 50% or higher.
3. Push **Tracking** (Knob 3) to 30–50% in **Scattered** mode for chaotic
   displacement.
4. Set **Head Switch** (Knob 4) to 40%.
5. Set **Dropout** (Knob 5) to 25%+ for frequent line dropouts.
6. Set **Color Loss** (Knob 6) to taste (0% keeps wild color, 100% goes mono).
7. Toggle **Color Noise** to **Color** for maximum chromatic chaos.
8. Use **Mix** (Fader 12) to back off if it becomes too much.

#### Settings

| | |
|---|---|
| Chroma Blur | 95% |
| Noise | 50% |
| Tracking | 40% |
| Head Switch | 40% |
| Dropout | 25% |
| Color Loss | 0% |
| Track Mode | Scattered |
| Drop Style | Snow |
| Color Noise | Color |
| Sync Warp | On |
| Bypass | Off |
| Mix | 100% |

## GLOSSARY

- **BRAM** — Block RAM. Dedicated memory blocks on the iCE40 FPGA used here as
  a scanline buffer for horizontal pixel displacement.
- **Chroma** — The color information in a video signal, carried by the U (Cb)
  and V (Cr) channels in YUV/YCbCr color space.
- **Dropout** — A defect where a section of tape fails to reproduce signal,
  resulting in a missing scanline.
- **Dry** — The unprocessed input signal before any VHS effects are applied.
- **Head Switch** — The transition point where a VCR's rotating head drum
  switches between heads during playback, often producing a visible noise bar.
- **IIR Filter** — Infinite Impulse Response filter. A recursive digital filter
  used here to blur chroma channels with adjustable bandwidth.
- **Interpolator** — A linear blending unit that crossfades between two values.
  Used here to mix dry and wet signals based on the Mix fader.
- **LFSR** — Linear Feedback Shift Register. A hardware-efficient pseudo-random
  number generator used as the noise source.
- **Luma** — The brightness component (Y) of the video signal.
- **Tracking** — The alignment between the VCR playback head and the recorded
  tracks on tape. Misalignment causes horizontal displacement artifacts.
- **Wet** — The fully processed signal after all VHS degradation stages.
