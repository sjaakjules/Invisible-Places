# Surface profile lab

Tooling for tuning the Surface_05 two-export After Effects workflow — a
smooth continuous **base** pass (alpha applied as a luma matte) and a
small-point **detail** pass (luma matte + multiply blend) — plus the
Projector x-ray look used by the Proj_A/B_09S01 renders.

## Why a lab

The live view normally shows the 5 mm display bundle; MP4/ProRes exports
render the full 1 mm bundle through the same GPU pipeline as the live
viewport (supersampled `DrawFrame` + readback — adaptive HQ never runs
during exports). Point size, falloff, and opacity behave very differently
at 25x point density, so profiles must be measured against the exact
export content, not the live 5 mm view.

## Running it

```bash
./build/macos-release/invisible_places.app/Contents/MacOS/invisible_places \
    <data-root> \
    --gui-smoke surface-profile-lab \
    --smoke-project <duplicated project copy> \
    --smoke-output <output dir>
```

The smoke loads the project copy, switches Scene3 to the complete 1 mm
bundle (198.7M points), loads the Surface_05 animation, and measures each
candidate profile at three cameras (normalized 0.15 / 0.50 / 0.85):

* median GPU frame time (`gpuTotal` timestamps),
* mean luma (fill / matte level),
* a 9x9 high-pass luma standard deviation (micro-detail: high for the
  detail pass, low for the smooth base),
* one PPM capture per candidate/camera.

It is read-only towards caches and project data; always point
`--smoke-project` at a duplicated copy while a live session is editing.

The built-in candidate matrix explores one change at a time around the
saved `Surface_05_Base` / `Surface_05_Thin` / `Projector-01-Wind`
profiles. To measure exact composed profiles instead (a confirmation
run), set `INVISIBLE_PLACES_PROFILE_LAB_CANDIDATES` to a JSON array of
`{"name": ..., "base_visual": ...}` naming library visuals — for example
freshly merged `*_v2` entries.

## 2026-09-02 findings (run 1, full 1 mm bundle)

* The flicker-validated stable-culling stack (RockOccluder prepass, rock
  DrawAll, sand SoftSeparation, influence 1.0, FullAnimation sort) is
  also **18–31% faster** than the authored Base with byte-identical luma
  and detail — the culled points were fully hidden. Partial influence
  (0.54) is slower than 1.0; use full influence.
* Scaling the Base surfelDiameter Density map by 1.5x (4→2 mm becomes
  6→3 mm) fills and smooths the sand plains (high-pass 15.3→12.9 at the
  mid camera) for a mixed cost; 2.0x adds little over 1.5x.
* Stable culling on Thin costs ~2.3x GPU for no detail gain — the
  detail pass ships without it.
* Rim falloff floods luma and erases detail; SoftDisc reads grainier
  than Gaussian on the Base. Both rejected.
* Sorted + Saturated on the x-ray profile costs +82% GPU and flattens
  the accumulated glow; the authored unsorted Accumulated response is
  the right rendering for that look.

## Composite-aware assessment (the metric that actually matters)

The detail export is applied over the base in After Effects with its own
luma as alpha (luma matte) and Multiply blend:

    out = base * (1 - luma(thin) * (1 - thin))

Pure black detail pixels vanish (alpha 0) and pure white multiplies by
one — only midtones carve into the base. An isolated high-pass metric on
the thin frame therefore counts the black gaps between small points as
"detail" even though they never reach the composite: measured on the lab
captures, **83–114% of the isolated detail reading was false** in that
sense. `scripts/analyze_surface_composites.py` implements the composite
model and reports the surviving detail gain, the false-detail fraction,
and the mean darkening of the base; base candidates are scored on
speckle (want low), mid-band structure (want high — smooth must not mean
blurry) and coverage consistency.

Ranked on composite gain, the thin winner moved from 0.7x+sharp24 to
**0.5x size (3→2 mm Density map) + Gaussian sharpness 32**: about twice
the surviving detail of the 0.7x pick with the least base-darkening, and
faster than the authored Thin. The base pick was confirmed by the
composite axes (size 1.5x lowers speckle while *raising* structure).

The composed profiles live as `Surface_05_Base_v2`, `Surface_05_Thin_v2`
and `Projector-01-Wind_v2` (the x-ray v2 only enables the normal-cull
back-face fade at its 75/105 degree defaults).

## Merging improved profiles into the artist's project

Profiles are iterated on duplicated project copies while the artist keeps
working; land them afterwards with:

```bash
python3 scripts/merge_point_visual_profiles.py \
    --project <latest saved project> \
    --profiles <profiles JSON>
```

The tool appends to `point_visuals` only, refuses to run while an app
instance is open, refuses to overwrite existing names without
`--replace`, writes a timestamped backup, and replaces the file
atomically.
