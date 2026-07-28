# Scene-Wide Point-Cloud Density Switching

## Scene Contract

A grouped LiDAR scene is a folder containing role-named point clouds. The switching workflow currently recognises the `ROCK`, `SAND`, and `VEG` roles and infers physical point spacing from filename tokens such as `1mm`, `2mm`, `3mm`, and `5mm`.

Discovery quantizes spacing to integer micrometres before comparing variants. A spacing is offered in **Visuals > Visible Point Cloud** only when the scene folder contains exactly one file at that spacing for every required role. A missing role or duplicate `(role, spacing)` file makes that spacing unavailable. Scene3 therefore exposes exactly 1, 2, 3, and 5 mm. Standalone point clouds and generated water overlays remain independent layers and are not included in the selector.

The selector sits between **Cloud** and **Saved Visuals** and is scene-wide. Changing it replaces ROCK, SAND, and VEG as one display bundle; it is not a per-role variant control. The former role-level Variant controls are read-only analysis-source status. The UI reports the spacing, total point count, loading progress, and any error. The old committed bundle remains visible while all three target roles load into CPU memory. Once the bundle is complete, rendering is settled once, the old GPU bundle is retired, and the target bundle is uploaded hidden before one atomic visibility commit. Every uploaded layer receives a fresh descriptor generation containing its base, highlight, EXR, and compact Seepage bindings; retired descriptor pools are released before any buffer they reference. A partial allocation or upload restores the previous bundle from retained CPU clouds without rescanning PLY files. Switching is disabled during an active export. Final exports ignore the live density switch: they render the finest complete scene bundle (loaded on demand by the export gate), while the viewport can stay on a coarse bundle for interactive framing.

New scenes select the finest complete bundle. A scene with no complete bundle retains its existing role selection as a non-switchable `Mixed` display.

## Analysis And Display Residency

Each project-loaded scene has two independent source classifications:

- **Analysis sources:** ROCK 1 mm, SAND 2 mm, and VEG 1 mm. These canonical files are catalogued at startup but load CPU-only on demand for explicit Bake Path and analysis-based Ripple/Field operations; they are not a startup prerequisite.
- **Committed display sources:** the complete density bundle selected in the Visuals tab. Only these scene sources are renderable and GPU-resident after a switch completes.

When one file belongs to both sets, ready CPU data can be reused. Obsolete display CPU/GPU resources are released after commit when no active purpose owns them. A staged target is CPU-only, neither a committed display source nor renderable. The transaction never holds complete old and new point buffers at the same time, bounding measured point-buffer bytes to the larger bundle plus fixed upload/descriptor staging. Native diagnostics report the transaction's settle count and byte high-water mark rather than estimating residency from point counts.

Ordinary framing, placement, and editing can use the committed runtime display support. Explicit canonical operations queue their required analysis roles and resume when those CPU sources are ready; missing analysis never blocks the first visible display. The display-independent shared `WaterSurfaceCache` separately streams the exact complete 2 mm ROCK/SAND/VEG bundle into one persisted 10 mm Rain/Flow/Seepage cache after the display upload completes. A scene without 2 mm uses its nearest complete bundle and reports the fallback; role spacings are never mixed.

Point-cloud loading and shared-surface build/load plus GPU preprocessing use one exclusive high-memory slot. The active display commits first; the shared surface cache then takes the slot before inactive queued loads, and the remaining work resumes only after preprocessing completes. Mesh Flow consumes the Ground table in that cache and has no separate dynamic-mesh warmup.

## Density-Compensated Rendering

Visuals values remain authored against a 1 mm point-spacing baseline, regardless of the selected display density. The renderer derives a transient compensation for each displayed role; this state is not written back into the authored point style.

For a displayed role:

```text
g = displaySpacing / 0.001
C = (displayPointCount / referencePointCount)
    x (displaySpacing / referenceSpacing)^2
k = clamp(1 / C, 1/16, 16)
```

The appearance reference is the role's 1 mm variant when it exists; otherwise it is that role's canonical analysis source or densest known variant. Invalid or zero point-count data uses `k = 1`.

`g` scales the complete authored footprint, including field-mapped sizes and water, Ripple, and Shoreline size additions. Depth-of-field and antialias additions are applied afterward. For example, an authored 2 mm point size renders as 10 mm on a 5 mm bundle; a mapped 1.5–2.2 mm range renders as 7.5–11 mm. The values displayed in the Visuals tab do not change.

Beauty rendering applies `k` after falloff, stylisation, depth fade, water effects, and coverage:

```text
alphaFinal = clamp(alphaRaw x k)
emission   = alphaRaw x k x emissive x exposure
```

This avoids applying density correction twice when authored opacity changes. A non-identity coverage correction uses the unified transparent material path. Viewport, still, animation, EXR, and CPU offline rendering use the same compensation. Fast Basic remains an opaque approximation: it applies footprint scale `g`, but ignores authored opacity, emission, and `k`.

Scalar-field bindings follow field names across variants because numeric field slots may be reordered. Resolution uses an exact name first, then one unique case-insensitive match. A slot is used only for a legacy binding with no field name. If a named field is absent, the authored binding is retained, its constant fallback is rendered, and a warning is shown so a field such as `Interest` cannot silently bind to `Roughness`.

## Water And Field Routing

Display switching changes presentation only. Explicit Flow/Field analysis uses CPU-ready canonical sources, while placement and responsive runtime editing can use committed support. Rain, surface-guided Flow, and connected-cell Seepage use the scene's static 10 mm shared surface cache, whose source signature and GPU upload revision remain stable while display density changes.

Display-dependent payloads are handled separately:

- Ripple memberships and Field presentation are display-source-specific because point indices differ between density variants; indices are never copied across variants.
- A display commit performs no Ripple, Field, Flow, or Seepage topology scan. Missing display-specific Ripple/Field state remains dirty until an explicit recalculation; settled Flow stays active, while compact connected cache-cell Seepage support is shared by scene role and surface-cache identity and only attached to each newly uploaded density layer.
- Seepage node **Visible** state lives in the compact parameter ring. Disabled viewport nodes remain in the semantic spatial topology with a zero enabled factor, so off/on changes neither connected support, hash cells, nor descriptors. Export enablement retains its independent snapshot filtering.
- Generated Flow and Field Streamline sessions are unchanged by the selector; dedicated Rain particles reuse the same shared surface cache and world-space impacts.
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
