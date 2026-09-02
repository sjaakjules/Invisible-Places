# Encoding the thin surface export

The Surface two-export workflow's detail (thin) pass is near-pixel
speckle over black at 4K - the hardest possible content for a hardware
video encoder. This page records the 2026-09-02 measurements behind the
encoder defaults and the recommended settings.

## What the noise was

Ground truth: a 120-frame PNG-stack export of Surface_05a frames
1800-1919 with `Surface_05_Thin_v2`, produced by the `export-segment-lab`
smoke (real export pipeline; specs via
`INVISIBLE_PLACES_EXPORT_SEGMENT_SPECS`, optional `tsN` field enables
temporal supersampling). Scoring the delivered "MP4 HQ VideoToolbox"
output against it:

* Colour (hevc_videotoolbox 4:2:2 10-bit, fixed 300 Mbps): per-frame
  SSIM starts at ~0.99 on each IDR and collapses to **0.27** mid-GOP
  (mean 0.74). The rate controller cannot hold the speckle; quality
  sawtooths with the GOP - the visible blotching and smearing.
* Alpha matte (90 Mbps): SSIM **0.56**. The matte is destroyed.
* Raising VideoToolbox to 450 Mbps only reaches SSIM 0.83.

The content itself also "boils": consecutive lossless frames differ by
mean |delta| ~39/255 under camera motion (thin points are ~1-2 px).
That shimmer is render-side aliasing, not compression.

## Measured options (same 4 s segment, full-render sizes extrapolated to 6301 frames)

| Encode | SSIM colour | SSIM matte | Full render size | Encode speed |
|---|---|---|---|---|
| VideoToolbox 300M (old default) | 0.736 | 0.560 | 7.9 + 2.4 GB | 36 fps |
| VideoToolbox 450M | 0.828 | - | 11 GB | 30 fps |
| **x265 fast CRF16 (new CPU HQ)** | **0.977** | **0.986** | ~18 + 13 GB | 3.4 fps |
| x265 slow CRF14 (old CPU HQ) | 0.983 | 0.994 | ~22 + 18 GB | 0.5 fps |
| ProRes 422 HQ | 0.963 | - | ~24 GB | 2 fps (sw) |
| ProRes 4444 | 0.99995 | 0.999997 (real alpha) | **~282 GB** | 4 fps (sw) |

GPU capture runs ~2.4 fps on this content, so any encoder at >=2.4 fps
adds no export time. That is why the software HQ MP4 paths now use
x265 `fast` (quality identical to `slow` here: 0.9768 vs 0.9771) - the
CPU encoder became free, and it is the recommended choice for the thin
pass: untick "VideoToolbox" on the MP4 HQ preset.

## Other levers

* Temporal supersampling (`ts4`): integrates 4 sub-frame samples per
  output frame. Render-side boil drops ~33% (39 -> 26) and even the
  VideoToolbox encode improves (0.74 -> 0.84 colour, 0.56 -> 0.76
  matte). Capture cost ~4x. A look choice - motion-blurred grain -
  worth a test render; combined with x265 it is effectively clean.
* Bigger/softer points encode better under any codec (0.7x size +
  sharpness 24 scored 0.85 vs 0.74 under VideoToolbox and halves the
  boil) but give up the composite detail that made Thin_v2 the pick;
  unnecessary once the encoder is x265.
* ProRes 4444 is bit-exact including real alpha in one file, but this
  content pushes it to ~10.7 Gbps (~282 GB per render): only for short
  hero segments.
* The base (smooth) profile compresses fine on VideoToolbox; these
  findings apply to speckle-like content.
