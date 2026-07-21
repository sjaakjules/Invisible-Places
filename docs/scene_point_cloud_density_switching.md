# Scene-Wide Point-Cloud Density Switching

## Scene Contract

A grouped LiDAR scene is a folder containing role-named point clouds. The switching workflow currently recognises the `ROCK`, `SAND`, and `VEG` roles and infers physical point spacing from filename tokens such as `1mm`, `2mm`, `3mm`, and `5mm`.

Discovery quantizes spacing to integer micrometres before comparing variants. A spacing is offered in **Visuals > Visible Point Cloud** only when the scene folder contains exactly one file at that spacing for every required role. A missing role or duplicate `(role, spacing)` file makes that spacing unavailable. Scene1 therefore exposes exactly 1, 2, 3, and 5 mm. Standalone point clouds and generated water overlays remain independent layers and are not included in the selector.

The selector sits between **Cloud** and **Saved Visuals** and is scene-wide. Changing it replaces ROCK, SAND, and VEG as one display bundle; it is not a per-role variant control. The former role-level Variant controls are read-only analysis-source status. The UI reports the spacing, total point count, loading progress, and any error. The old committed bundle remains visible while a target bundle is loaded, uploaded, and prepared. All roles commit together. A failed or superseded switch discards staged resources and leaves the previous bundle unchanged. Switching is disabled during an active export, and exports include only the committed display bundle.

New scenes select the finest complete bundle. A scene with no complete bundle retains its existing role selection as a non-switchable `Mixed` display.

## Analysis And Display Residency

Each project-loaded scene has two independent source classifications:

- **Analysis sources:** ROCK 1 mm, SAND 2 mm, and VEG 1 mm. These canonical files are loaded CPU-only at startup and remain resident while the scene is active.
- **Committed display sources:** the complete density bundle selected in the Visuals tab. Only these scene sources are renderable and GPU-resident after a switch completes.

When one file belongs to both sets, its CPU data is reused. Obsolete display CPU/GPU resources are released after commit unless they are canonical analysis sources. A staged target is neither a committed display source nor renderable.

Pivot samples and combined Flow/Field support caches are prewarmed from the analysis set. Picking, region selection, source placement, flow solving, field solving, water support signatures, and support-cache generation never fall back to a sparse display source. Rain collision is display-independent too: it streams exact 5 mm role files, or each role's coarsest fallback, into a separate persisted 20 mm cache. If a required source is still loading or failed, the dependent action is disabled and reports the loading/error state.

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

Display switching does not change simulation input. Combined water support, source placement, path/field caches, Flow routing, and region analysis use CPU-ready canonical analysis sources. Rain collision queries use the scene's static 20 mm role-aware cache, whose source signature and GPU upload revision also remain stable while display density changes.

Display-dependent payloads are handled separately:

- Ripple memberships are rebuilt or restored for each exact display cloud because point indices differ between density variants.
- Field simulation remains analysis-based, but presentation fields are remapped spatially to the target display cloud before upload. An analysis-cloud point index is never reused as a display-cloud index.
- Target Ripple and Field payloads are prepared before a display switch commits.
- Generated Flow and Field Streamline sessions are unchanged by the selector; dedicated Rain particles reuse the same shared collision cache and world-space impacts.
- SAND Shoreline settings remain authored once and are evaluated on the committed SAND display source.

Viewport rendering, framing, frustum masks, still/animation snapshots, and offline export all use the committed-display predicate. CPU-only analysis sources and staged switch targets are excluded.

## Project Schema 33

Schema 33 adds an authoritative `scene_point_cloud_groups` array. Each group records the committed display state and the per-role analysis/display paths:

```json
{
  "scene_point_cloud_groups": [
    {
      "scene_group": "Scene1",
      "display_spacing_meters": 0.005,
      "display_loaded": true,
      "display_visible": true,
      "roles": [
        {
          "scene_role": "ROCK",
          "analysis_source_path": "Data/Scene1/Site3-ROCK-1mm.ply",
          "display_source_path": "Data/Scene1/Site3-ROCK-5mm.ply"
        }
      ]
    }
  ]
}
```

The example abbreviates the `roles` array; a normal complete scene stores ROCK, SAND, and VEG records. Legacy per-layer grouping, selected-variant, loaded, and visible fields remain compatibility mirrors, but the group record is authoritative when present.

When loading a schema-32-or-earlier project, legacy selected paths are preserved as analysis-source candidates. The loader derives the display spacing from the visible primary/ROCK selection when that spacing forms a complete bundle. Otherwise it chooses the nearest complete bundle, preferring the denser bundle on a tie. If no complete bundle exists, it retains a non-switchable `Mixed` selection. Missing saved paths fall back through the same catalog validation instead of substituting a sparse display source for analysis.

Only the last successfully committed display spacing is saved. A pending or failed switch never becomes project state.
