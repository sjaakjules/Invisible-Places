# Scene-Wide Point-Cloud Density Switching

## Scene Contract

A grouped LiDAR scene is a folder containing role-named point clouds. The switching workflow currently recognises the `ROCK`, `SAND`, and `VEG` roles and infers physical point spacing from filename tokens such as `1mm`, `2mm`, `3mm`, and `5mm`.

Discovery quantizes spacing to integer micrometres before comparing variants. A spacing is offered in **Visuals > Visible Point Cloud** only when the scene folder contains exactly one file at that spacing for every required role. A missing role or duplicate `(role, spacing)` file makes that spacing unavailable. With the shared OneDrive source set as the primary data root (which intentionally carries only the 1 mm and 5 mm bundles; the locally generated 2 mm/3 mm variants are retired and suppressed for Scene3) Scene3 exposes 1 and 5 mm; the local-only Scene1 likewise exposes its 1 and 5 mm bundles. A scene group the project has never recorded stages its coarsest complete bundle first, so a freshly added local scene costs its 5 mm set on first load rather than the multi-GB finest bundle. Standalone point clouds and generated water overlays remain independent layers and are not included in the selector.

The selector sits between **Cloud** and **Saved Visuals** and is scene-wide. Changing it replaces ROCK, SAND, and VEG as one display bundle; it is not a per-role variant control. The former role-level Variant controls are read-only analysis-source status. The UI reports the spacing, total point count, loading progress, and any error. The old committed bundle remains visible while all three target roles load into CPU memory. Once the bundle is complete, rendering is settled once, the old GPU bundle is retired, and the target bundle is uploaded hidden before one atomic visibility commit. Every uploaded layer receives a fresh descriptor generation containing its base, highlight, EXR, and compact Seepage bindings; retired descriptor pools are released before any buffer they reference. A partial allocation or upload restores the previous bundle from retained CPU clouds without rescanning PLY files. Switching is disabled during an active export. Final exports ignore the live density switch: they render the finest complete scene bundle (loaded on demand by the export gate), while the viewport can stay on a coarse bundle for interactive framing.

New scenes select the finest complete bundle. A scene with no complete bundle retains its existing role selection as a non-switchable `Mixed` display.

## Analysis And Display Residency

Each project-loaded scene has two independent source classifications:

- **Analysis sources:** the complete ROCK/SAND/VEG 1 mm bundle. These canonical files are catalogued at startup but load CPU-only on demand for explicit Bake Path operations; they are not a startup prerequisite.
- **Committed display sources:** the complete density bundle selected in the Visuals tab. Only these scene sources are renderable and GPU-resident after a switch completes.

When one file belongs to both sets, ready CPU data can be reused. Obsolete display CPU/GPU resources are released after commit when no active purpose owns them. A staged target is CPU-only, neither a committed display source nor renderable. The transaction never holds complete old and new point buffers at the same time, bounding measured point-buffer bytes to the larger bundle plus fixed upload/descriptor staging. Native diagnostics report the transaction's settle count and byte high-water mark rather than estimating residency from point counts.

Ordinary framing, placement, and editing can use the committed runtime display support. Explicit canonical operations queue their required analysis roles and resume when those CPU sources are ready; missing analysis never blocks the first visible display. The display-independent shared `WaterSurfaceCache` separately streams the exact complete 2 mm ROCK/SAND/VEG bundle into one persisted 10 mm Rain/Flow/Seepage cache after the display upload completes. A scene without 2 mm uses its nearest complete bundle and reports the fallback; role spacings are never mixed.

Point-cloud loading and shared-surface build/load plus GPU preprocessing use one exclusive high-memory slot. The active display commits first; the shared surface cache then takes the slot before inactive queued loads, and the remaining work resumes only after preprocessing completes. Mesh Flow consumes the Ground table in that cache and has no separate dynamic-mesh warmup.

## Linked A/B HQ Live Override

The session-only **HQ** control beside **Seam / A / B** is deliberately separate
from the saved Visible Point Cloud selector. For the active reciprocal pair it
resolves one common grouped scene and its exact complete 1 mm and 5 mm bundles.
If 5 mm is not the committed display, those three sessions load hidden and are
owned only by the linked live override; the saved selection is unchanged. HQ
off then presents complete 5 mm ROCK/SAND/VEG.

After the baseline and shared-water cache settle, a utility-priority,
cancellable worker evaluates A and B at normalized `0.5`. Each member uses its
authored live-view aspect ratio or the current live aspect as a legacy
fallback. The two frusta keep authored near/far depth and extend X/Y by 5% of
the full viewport on every side. The filtered PLY reader streams only 1 mm
ROCK and VEG, preserves source order and original 32-bit point indices, and
materialises only points whose centres pass either padded frustum. It does not
stat, open, or allocate 1 mm SAND. The identical predicate partitions the
resident 5 mm ROCK/VEG indices, so each role is exactly 1 mm inside plus 5 mm
outside with no boundary overlap or gap.

The two compact patch uploads complete before either complementary mask can be
selected. A resource-mutation batch publishes masks on toggle and clears them
on disable/failure, so preparation and cancellation always leave a complete
5 mm frame. Empty outside sets omit that base draw rather than interpreting an
empty renderer mask as “draw all.” Prepared CPU/GPU data stays in memory for
instant Seam/A/B toggling and is released when the pair, midpoint matrices or
fallback aspect, project path, or any of the five sources actually read (all
three 5 mm roles plus 1 mm ROCK/VEG) changes.

Patch layers resolve the same role Visual, field names, Timing Colourise,
Rain, Seepage, Flow/roughness motion, renderer mode, depth of field, and EDL as
their 5 mm surroundings. Their values are 1 mm values, while automatic
field-map ranges come from the complete 5 mm role to avoid a patch-local
renormalization seam. Existing full-role density compensation remains on the
5 mm remainder; a 1 mm patch uses identity compensation.

The patch spacing selector beside the HQ button (`linked_hq_patch_spacing_um`
in the project, 1000 by default) trades patch density for speed without
touching the 5 mm remainder or exports. At 2 mm or 3 mm the scanned 1 mm
points inside the union are thinned by `DecimatePointCloudSubsetByGrid`, the
density-preserving cell stratification of the display-density cache: points
group into half-offset cubic cells of the chosen spacing, a cell with `n`
parents keeps `round(n / 4)` (2 mm) or `round(n / 9)` (3 mm) of them with the
fraction dithered by a stable cell hash (so totals are unbiased and sparse
cells are not cut off), a single output is the real parent nearest the cell's
parent centroid, and further outputs follow a stable position/colour hash.
This is orientation independent, deterministic, and independent of scan
threading. A plain per-point random hash was tried first and rejected: it
matched the mean brightness but its Poisson density fluctuations rendered as
visible speckle, because the sharp gaussian kernel paints only ~0.2 of the
sprite diameter. The patch declares the nominal spacing together with its
exact kept/scanned counts, so `ResolvePointCloudDensityCompensation` yields
footprint 2 or 3 with coverage 1 — the same rule that keeps the 5 mm display
bundle at 1 mm brightness. Changing the spacing rescans the sources and
re-enables a live HQ view when the new patches publish. On
`Proj_A_09S01`/`Proj_B_09S01` the 2 mm patch holds 10.6 M points and adds
roughly a third to the 5 mm frame time instead of tripling it; in A-0.5
captures its mean luminance is within 0.2% of the 1 mm patch and its 9x9
high-pass texture measure matches (22.7 vs 22.5), while 3 mm visibly thins
fine vegetation. `--gui-smoke linked-hq-frame-capture` writes the frames used
for this comparison. All fields already
referenced by saved Visuals and Timing Takes load during extraction. Ordered
source indices allow a newly referenced scalar field to be gathered later in
the background without retaining the complete 1 mm source cloud.

Only the main live render-state call opts into these internal ids. The default
render-state builder, frozen snapshots, full-density export gate, project and
animation serializers remain unaware of them. Still images, frame previews,
video and EXR therefore continue to render complete canonical 1 mm
ROCK/SAND/VEG, and this feature requires no project or animation schema change.

The opt-in native diagnostics are `--gui-smoke linked-hq-sample-scene` and
`--gui-smoke linked-hq-scene3`. The SampleScene scenario also changes B's
midpoint camera during the first scan to exercise cancellation and
invalidation. Both scenarios verify monotonic per-selection progress, zero
1 mm SAND session reads/fingerprints, atomic two-patch publication, instant
Seam/A/B-safe toggling, Visual/Timing/Rain/Seepage parity, density
compensation, serialization isolation, and canonical 1 mm export selection.

`--gui-smoke linked-hq-frame-timing` is the performance diagnostic. It loads
the authored workspace project, waits for the HQ patches, and samples frame
timings at the A and B normalized-0.5 cameras with HQ off and on, with rain,
seepage, and all water ablated, and with a backwards-facing camera so every
patch point is frustum-culled. For `Proj_A_09S01`/`Proj_B_09S01` the union
holds 28.3 M 1 mm ROCK and 14.2 M 1 mm VEG points (all 2–8 m from the
cameras, so no distance cap would help); at 2880×1800 on an M1 Max the live
frame goes from ~100 ms (5 mm) to ~290 ms (HQ) at A-0.5, of which ~130 ms is
the base sprite material on the extra ~22 M visible points, ~65 ms rain plus
seepage on them, and ~20 ms the residual of the ~20 M frustum-culled points.
The cost is therefore proportional to the legitimately visible 1 mm points;
the frustum-culled residual is the only remaining lossless lever.

## Deterministic Local Display-Density Cache

Scene3 can transparently replace the payload bytes behind its discovered 5 mm ROCK/SAND/VEG display paths with one validated local bundle. The default cache root is `Saved/.invisible_places/cache/display_density/Scene3` and has this layout:

```text
Scene3/
  active-bundle.json
  <bundle-fingerprint>/
    display-density-manifest.json
    Scene3/
      Site3-ROCK-5mm.ply
      Site3-SAND-5mm.ply
      Site3-VEG-5mm.ply
```

`active-bundle.json` schema 1 records the bundle fingerprint and the SHA-256 of the exact manifest bytes. The schema-1 manifest identifies the deterministic algorithm and complete scene, then records each role's canonical source path, size, nanosecond modification time, point count, property-schema digest, and full SHA-256 together with the equivalent identity for its cached output. The bundle fingerprint hashes the complete algorithm object plus the ordered ROCK/SAND/VEG source and output identities. The active pointer is replaced only after all three outputs and the finalized manifest are durable; a partial or rejected build never changes the settled pointer.

At activation the application verifies the small pointer/manifest binding, supported algorithm, complete role set, safe local paths, current file sizes and modification times, canonical/output PLY headers and property-schema digests, and the full SHA-256 of all three local outputs. Canonical 1 mm sources take the fast immutable-reference path at startup: path identity, size, modification time, vertex count, and schema are checked without rehashing roughly 30 GB. A changed source or cached output rejects the whole overlay. No role is redirected until every role passes.

The catalog and project retain the logical shared `Site3-ROLE-5mm.ply` paths. Point-cloud readers resolve those three paths to the active local payload only at the I/O boundary, so scene grouping, saved paths, density switching, and the canonical full-density export choice remain unchanged. Set `INVISIBLE_PLACES_DISABLE_DISPLAY_DENSITY_CACHE=1` before launch to bypass the overlay and read the shared 5 mm files directly.

Build the local bundle explicitly with `scripts/build_scene3_display_density_cache.py`. The builder opens the exact canonical 1 mm sources read-only, rejects a cache root that overlaps or aliases them, preflights temporary disk space, derives exact role budgets with deterministic density-proportional strata and local attribute prefiltering, independently verifies every output, then atomically activates the local bundle. Its default RGB mean deliberately matches the renderer's current byte-domain interpretation of the untouched 1 mm reference; linear-light filtering is an opt-in diagnostic recorded in the manifest.

Local activation does not modify or promote any shared file. Replacing shared 5 mm assets is a separate, explicit operation after matched 5 mm/1 mm still and motion validation, with verified rollback copies and atomic role replacement. The canonical 1 mm sources are never promotion targets; changing them requires separate advance approval because they drive final output.

## Density-Compensated Rendering

Visuals values remain authored against a 1 mm point-spacing baseline, regardless of the selected display density. The renderer derives a transient compensation for each displayed role; this state is not written back into the authored point style.

For a displayed role:

```text
gNominal = displaySpacing / 0.001
C = (displayPointCount / referencePointCount)
    x (displaySpacing / referenceSpacing)^2
areaCorrection = clamp(1 / C, 1/16, 16)
if areaCorrection > 1:   g = gNominal x sqrt(areaCorrection),  k = 1
else:                    g = gNominal,                         k = areaCorrection
```

The appearance reference is the role's 1 mm variant when it exists; otherwise it is that role's canonical analysis source or densest known variant. Invalid or zero point-count data uses `areaCorrection = 1`.

`g` scales the authored raster footprint, including field-mapped sizes and water/Shoreline size additions. The antialias support is a fixed screen-space margin shared by every display density (scaling it padded coarse-bundle sprites with a constant pixel band that inflated distant points and made the live view disagree with the fine-bundle export). Two further rules keep world-sized points consistent at every camera distance: the falloff kernel is normalised to the authored (plus depth-of-field) footprint inside the padded sprite rather than stretching across the antialias margin, and a point clamped up to the minimum raster size carries the squared size deficit in its alpha so the clamp adds no integrated brightness. Above the floor, authored opacity (including opaque solid centres) is untouched. The `scene3-visual-parity` GUI smoke measures both rules: it renders Surface_03 and Projector-01 across authored sizes, opacity/emissive variants, and near/mid/far cameras on the committed 5 mm bundle and the full 1 mm export bundle, reporting per-configuration luma-matte means over black (the After Effects external-alpha workflow). The nominal spacing ratio first restores the 1 mm-authored footprint. An under-covered source (fewer points than its spacing implies) then grows its footprint by `sqrt(areaCorrection)` so its covered area matches the reference. An over-covered source never shrinks below `gNominal`: a grid-decimated bundle keeps its nominal pitch regardless of how many cells the irregular reference filled, and a kernel narrower than that pitch renders as discrete dots with dark gaps (the Scene3 5 mm "speckle" regression of 2026-08-19, whose footprint fell to 4.13x for ROCK and 2.48x for VEG). The residual over-coverage is instead applied per fragment as `k`. Camera depth-of-field is a later image-space effect and is therefore added after density scaling. For an ideal 5 mm decimation, an authored 2 mm point size renders as 10 mm and a mapped 1.5–2.2 mm range renders as 7.5–11 mm. Measured point-count deviations adjust those diameters or alpha without changing the values displayed in the Visuals tab.

Beauty rendering applies `k` after falloff, stylisation, depth fade, water effects, and coverage:

```text
alphaFinal = clamp(alphaRaw x k)
emission   = alphaRaw x k x emissive x exposure
```

Growing an under-covered source in area rather than per-fragment alpha avoids alpha clamping and keeps the reference cloud's revealage and weighted-blend depth weights. Keeping an over-covered source at its nominal footprint preserves surface continuity at close range; its accumulated coverage is matched through `k`. Density-compensated Beauty layers stay on the shared accumulation material so the live viewport, still, animation, EXR, and CPU offline paths cannot diverge through an opaque depth-only shortcut. Fast Basic uses the same compensated footprint but remains an explicitly opaque approximation that ignores authored opacity, emission, and `k`.

Scalar-field bindings follow field names across variants because numeric field slots may be reordered. Resolution uses an exact name first, then one unique case-insensitive match. A slot is used only for a legacy binding with no field name. If a named field is absent, the authored binding is retained, its constant fallback is rendered, and a warning is shown so a field such as `Interest` cannot silently bind to `Roughness`.

## Water And Field Routing

Display switching changes presentation only. Explicit Flow/Field analysis uses CPU-ready canonical sources, while placement and responsive runtime editing can use committed support. Rain, surface-guided Flow, and connected-cell Seepage use the scene's static 10 mm shared surface cache, whose source signature and GPU upload revision remain stable while display density changes.

Display-dependent payloads are handled separately:

- A display commit performs no Flow or Seepage topology scan. Settled Flow stays active, while compact connected cache-cell Seepage support is shared by scene role and surface-cache identity and only attached to each newly uploaded density layer.
- Seepage node **Visible** state lives in the compact parameter ring. Disabled viewport nodes remain in the semantic spatial topology with a zero enabled factor, so off/on changes neither connected support, hash cells, nor descriptors. Export enablement retains its independent snapshot filtering.
- Generated Flow sessions are unchanged by the selector; dedicated Rain particles reuse the same shared surface cache and world-space impacts.
- SAND Shoreline settings remain authored once and are evaluated on the committed SAND display source.

Viewport rendering and framing use the committed-display predicate. Still/animation snapshots, frustum masks, and offline export use the full-density export predicate, which resolves each visible scene to its finest complete bundle regardless of the committed display density. Video and PNG-stack modes always render that full source (with frustum masking when useful); the HQ EXR mode renders it unless the Export tab's "Playback Density (fast preview)" checkbox — off by default — explicitly requests decimated playback draw counts. CPU-only analysis sources and staged switch targets are excluded.

## Project Schema 44

The authoritative `scene_point_cloud_groups` array records committed display state and per-role analysis/display paths. Schema 44 stores an optional compact `water_surface_cache` manifest per group:

```json
{
  "scene_point_cloud_groups": [
    {
      "scene_group": "Scene3",
      "display_spacing_meters": 0.005,
      "display_loaded": true,
      "display_visible": true,
      "water_surface_cache": {
        "relative_path": "../Data/Scene3/.invisible_places/cache/water/example.surfacecache",
        "cache_schema": 4,
        "algorithm_id": "water-surface-10mm-normal-average-ground-v4",
        "requested_rebuild_generation": 1,
        "built_rebuild_generation": 1
      },
      "roles": [
        {
          "scene_role": "ROCK",
          "analysis_source_path": "Data/Scene3/Site3-ROCK-1mm.ply",
          "display_source_path": "Data/Scene3/Site3-ROCK-5mm.ply"
        }
      ]
    }
  ]
}
```

The example abbreviates the `roles` array and cache fingerprint/checksum fields; a normal complete scene stores ROCK, SAND, and VEG records plus the active scene's 5 mm `MESHSampled` Ground source when available. Schema-4 payloads live at `<scene>/.invisible_places/cache/water/<signature>.surfacecache`, fall back beside the project when scene storage is unavailable, and can read schema-3 `.surfacecache` and schema-2 `.raincache` files as legacy inputs. The current Ground algorithm makes an older sidecar stale, so it is rebuilt atomically while the last settled GPU cache remains active. Requested/built generations make **Rebuild Cache** durable.

Clean generated Flow branches are similarly externalized to scene-local `.invisible_places/cache/flow/*.flowpathcache` sidecars with a compact `water_path_cache_manifest`; stale or orphaned derived arrays are not embedded in schema-45 project JSON.

The Ground tier is independent of the selected display density. A scene-wide 1/2/3/5 mm ROCK/SAND/VEG switch reuses the same resident 10 mm Ground hash and fixed-capacity GPU Mesh Flow resources. Mesh Flow style, activity, Rain response, spawning, noise, wind, and contact-response edits update parameters only; they do not warm a triangle `MeshSurfaceCache`, scan the display cloud, or create a generated CPU point-cloud layer.

When loading a schema-32-or-earlier project, legacy selected paths are preserved as analysis-source candidates. The loader derives the display spacing from the visible primary/ROCK selection when that spacing forms a complete bundle. Otherwise it chooses the nearest complete bundle, preferring the denser bundle on a tie. If no complete bundle exists, it retains a non-switchable `Mixed` selection. Missing saved paths fall back through the same catalog validation instead of substituting a sparse display source for analysis.

Only the last successfully committed display spacing is saved. A pending or failed switch never becomes project state.
