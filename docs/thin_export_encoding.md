# Encoding the thin surface export

The Surface two-export workflow's detail (thin) pass is near-pixel
speckle over black at 4K - the hardest possible content for a hardware
video encoder. This page records the 2026-09-02 measurements behind the
encoder defaults and the recommended settings.

## Current recommendation (2026-09-04)

The export panel now offers three MP4 quality tiers: **Normal**, **HQ**
and **Max**. They change encoder settings only - the render itself is
identical - and every tier encodes at least as fast as the GPU captures,
so the choice does not change export time. Use **HQ** with VideoToolbox
for smooth or solid content such as the Base pass. For sparse fine-point
content such as Thin, use **HQ** with VideoToolbox off (x265) when file
size matters, or **Max** with VideoToolbox for the same quality from the
hardware encoder with a much larger file (constant quality 75 plus a
15-frame GOP, so speckle survives the encoder's rate control).

The panel now also shows the measured SSIM of the selected tier against
the ProRes 4444 / lossless gold standard (colour and alpha, at the
smooth and fine-point content anchors from the tables below), plus a
time and file-size estimate projected from this machine's most similar
completed export (stored in Saved/.invisible_places/export_history.json).

The latest 120-frame, full-pipeline comparison used the real 4x
supersampled export and lossless PNG ground truth:

| Visual / encode | SSIM colour | SSIM matte | 12,601-frame estimate |
|---|---:|---:|---:|
| Thin, MP4 HQ VideoToolbox | 0.923 | 0.904 | 20 GB |
| **Thin, MP4 HQ CPU (x265)** | **0.972** | **0.997** | **35 GB** |
| Thin, ProRes 422 HQ | 0.952 | 0.985 | 90 GB |
| Thin, ProRes 4444 | 0.9996 | 0.9998 | 573 GB |
| **Base, MP4 HQ VideoToolbox** | **0.988** | **0.996** | **16 GB** |
| Base, ProRes 422 HQ | 0.984 | 0.998 | 64 GB |

For the hardware Max tier, a controlled 48-frame re-encode of
the same Thin PNG ground truth selected VideoToolbox constant-quality 75
and a 15-frame GOP. It measured 0.9719 colour and 0.9984 matte SSIM, versus
0.972/0.997 for MP4 HQ CPU. The trade-off is approximately 85 GB for the
complete 12,601-frame colour-plus-matte render. Encoder throughput was
about 35 fps, so it remains hidden behind GPU capture. A 12-frame
end-to-end check also improved over hardware HQ (0.944 versus 0.920 colour,
0.962 versus 0.922 matte), although such a short run is sensitive to the
render's cold first frame and is not the primary codec score.

VideoToolbox does not expose frame-thread controls for this HEVC encoder.
The useful temporal lever was the shorter 15-frame GOP: it limits error
propagation across moving fine points. More CPU threads cannot accelerate
the GPU capture; its default full-frame submission executes serially on the
graphics queue.

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

## Historical 2026-09-02 measurements

These earlier measurements used a 6,301-frame duration and precede the
current renderer and encoder settings. They are retained to show the
failure mode rather than to supersede the current table above.

### Measured options (same 4 s segment, full-render sizes extrapolated to 6301 frames)

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
