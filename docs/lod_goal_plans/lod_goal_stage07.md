# lod_goal_stage07.md — Progressive `.ipcloud` Cache Bundle

## Codex goal

```text
/goal Implement the persistent `.ipcloud` cache bundle and progressive coarse rendering path so first-load and subsequent-load point clouds can show upper-hierarchy representatives before full parse/build/upload completes, verified by clean-cache, interrupted-build/resume, and warm-cache runs with explicit cache status and time-to-first-coarse-frame diagnostics. Preserve the existing working adaptive renderer while migrating safely. Between iterations, implement one cache component or load phase, verify atomicity/validation, and rerun cold/warm load checks. If compatibility decisions are needed, version/invalidate safely and report the tradeoff.
```

## Required prior stage

Stage 06 should be complete. The adaptive renderer should already be usable and measured before changing cache architecture.

## Why this stage exists

The current cache stores hierarchy nodes and representatives only. The target system needs a cache bundle with manifest, schema, hierarchy pages, raw chunks, LOD representatives, scalar stats, build status, and build log. The first load should not be a long black-screen preprocessing stage.

## Scope for this stage

In scope:

- `.ipcloud` directory bundle format.
- Manifest and fingerprint validation.
- Attribute schema file.
- Hierarchy/nodes/pages/stats files.
- LOD representative data.
- Scalar stats.
- Build status and build log.
- Temporary build directory and atomic publish by rename.
- Resumable partial build metadata.
- Coarse-first progressive display while building/loading.
- Diagnostics for cache state and time to first coarse frame.

Out of scope:

- Full optimized raw chunk residency/LRU. Stage 08 handles streaming/memory architecture.
- GPU compute traversal.
- Tile overdraw/occlusion.
- Export-specific finalization beyond preserving existing exports.

## Target cache layout

Use this layout unless the repo already has a better convention:

```text
PointCloudCache/
    ExampleSite.<fingerprint>.ipcloud/
        manifest.json
        attribute_schema.bin
        hierarchy.bin
        node_pages.bin
        node_stats.bin
        raw_chunks/
            chunk_000000.bin
            chunk_000001.bin
            ...
        lod_representatives.bin
        scalar_stats.bin
        build_status.json
        build_log.txt
```

Build into:

```text
ExampleSite.<fingerprint>.ipcloud.tmp/
```

and publish by rename when complete.

## Implementation tasks

1. Inspect current `Saved/lod/<source-stem>-<source-path-hash>-PointCloudLodCache-v3.bin` cache implementation.
2. Design a versioned manifest that includes at minimum:
   - cache format version
   - source path hash/fingerprint fields
   - source file size
   - source modified time
   - source header hash
   - source content hash or sampled block hashes
   - point count
   - scalar field count
   - RGB/normals presence
   - bounds
   - estimated raw spacing
   - hierarchy node count
   - leaf chunk count
   - max tree depth
   - attribute schema version
   - build settings version
   - coordinate/transform settings if they affect stored positions
3. Implement validation with clear stale/miss reasons.
4. Write cache files into a temporary bundle and publish atomically only after required files are complete.
5. Record resumable progress in `build_status.json`:
   - parse/header complete
   - upper hierarchy complete
   - node pages complete
   - representative ranges complete
   - raw chunk ranges complete, even if Stage 08 will optimize their residency
   - scalar stats complete
   - complete/failed/interrupted state
6. Implement progressive first-load behaviour:
   - parse header quickly and show bounds/placeholder
   - build root/upper hierarchy first
   - render coarse representatives while deeper data continues building
   - refine visible view as more hierarchy data becomes available
   - show explicit statuses such as `Loading point cloud hierarchy...`, `Rendering coarse preview...`, and `Refining visible detail...`
7. Implement subsequent-load behaviour:
   - load manifest and upper hierarchy first
   - start rendering coarse cloud quickly
   - load remaining hierarchy/LOD data asynchronously
   - keep old single-file cache compatibility only if it is cheap and safe; otherwise invalidate/migrate with clear versioning
8. Add diagnostics:
   - cache hit/miss/stale reason
   - build phase
   - load phase
   - time to bounds/placeholder
   - time to first coarse frame
   - time to first refined frame
   - build progress percentage if available
   - cache publish/resume status

## Automated verification

Run:

```text
cmake --build --preset build-macos-debug
./build/macos-debug/invisible_places_tests "[pointcloud][lod]"
ctest --test-dir build/macos-debug --output-on-failure
```

Add tests for:

- manifest validation and stale reasons
- temporary bundle not treated as complete cache
- atomic publish behaviour
- interrupted/resumed build status parsing
- corrupt/missing file handling
- upper-hierarchy load before full lower hierarchy

## Manual runtime checkpoint for the user

Test with a representative large cloud:

1. Delete/rename existing cache for the cloud.
2. Launch the app.
3. Observe first-load behaviour.
4. Interrupt/quit during build, relaunch, and observe resume/rebuild behaviour.
5. Relaunch after full cache completion.

Expected behaviour:

- The user sees bounds/placeholder quickly instead of a long black screen.
- Coarse representatives appear before full processing completes.
- The cache is not marked complete until all required files are written.
- An interrupted build either resumes safely or restarts with a clear reason.
- Warm-cache load reaches first coarse frame significantly faster than cold source parse.
- Existing Stage 02–06 adaptive runtime behaviour remains intact after cache load.

## Completion condition

This stage is complete only when:

- `.ipcloud` bundle exists with versioned validation and atomic publish.
- Progressive coarse rendering works on cold and warm loads.
- Cache status and time-to-first-coarse-frame diagnostics are visible.
- Corrupt/partial/stale caches do not crash or silently produce wrong data.

## Stop conditions

Stop and report if:

- Source loading architecture requires a larger async asset-system refactor.
- The project cannot render any data until full source upload due to deep renderer assumptions.
- Cache migration would destroy user data or require a policy decision.

## Handoff to Stage 08

Pass along:

- Raw chunk metadata and any temporary raw chunk format.
- Time-to-first-coarse and warm-cache measurements.
- Memory duplication observed during progressive load.
