# AGENT.md — Codex Execution Contract

This file is the operating contract for Codex agents working on this project.

Keep this file short in spirit: use it as the control plane, not as the encyclopedia. The source of truth is split across:
- `project_description.md` — creative intent, workflows, scene behaviour, user-facing goals
- `PDR.md` — product requirements and acceptance criteria
- `code_plan.md` — target architecture, modules, milestones, and implementation details

When this file conflicts with a more detailed technical note that was intentionally updated later, prefer the newer source and update this file in the same change.

---

## 1. Project intent
Build a C++ desktop renderer for:
- large point clouds from CloudCompare-exported binary PLY,
- Gaussian splat layers,
- scalar-field-driven visual styling,
- cinematic camera shots,
- interactive look-dev on Apple Silicon M1,
- optional higher-end offline rendering on Windows GPUs.

Primary implementation direction:
- C++
- Vulkan
- MoltenVK on macOS
- wrapped `3dgs-vulkan-cpp`-style GS subsystem
- custom point-cloud renderer
- immediate-mode UI side panel for render/style editing

Do not flatten Gaussian splats into plain points just because a task is easier that way.

---

## 2. Non-negotiables
1. Preserve a clean separation between:
   - renderer core,
   - point-cloud rendering,
   - Gaussian splat integration,
   - scene/camera logic,
   - UI,
   - IO/serialization,
   - output/AOV.
2. Point-cloud style parameters must support both:
   - constant values,
   - field-mapped values.
3. Camera orientation storage and interpolation must use quaternions.
4. Procedural motion must be non-destructive and evaluated from rest position.
5. Preview behaviour on M1 matters more than maximum feature count in early milestones.
6. Project files must remain portable across macOS and Windows.
7. If a task changes a public data contract, update the owning docs in the same change.
8. Prefer small vertical slices that compile over broad speculative rewrites.

---

## 3. Working rules for every agent
Before making edits:
1. Read this file.
2. Read the relevant sections of `project_description.md`, `PDR.md`, and `code_plan.md`.
3. Restate the task as a narrow diff.
4. Identify owned files and do not edit outside them unless explicitly allowed.
5. List assumptions in code comments or task notes when the docs are silent.

While editing:
1. Keep interfaces explicit.
2. Keep shader parameter evaluation shared and centralised.
3. Add tests or validation scaffolding for every new subsystem.
4. Do not silently rename data fields, JSON keys, shader bindings, or enum values.
5. Do not add platform-specific code without a fallback or compile guard.

Before finishing:
1. Verify the change against the task acceptance tests.
2. Note remaining risks or follow-up work.
3. Update docs when architecture, UX, or file formats changed.

---

## 4. Source-of-truth map
Use this map instead of guessing.

### Product / UX
- `project_description.md`
- `PDR.md`

### Architecture / ownership / implementation
- `code_plan.md`

### Primary acceptance anchors
- `PDR.md`, sections `5`, `6`, `7`, `8`, `9`

### Important design anchors
- side panel behaviour and parameter binding: `code_plan.md`, sections `6` and `7`
- camera and shot system: `code_plan.md`, section `8`
- point rendering and motion: `code_plan.md`, sections `9` and `10`
- AOV/output behaviour: `code_plan.md`, sections `11` and `12`

---

## 5. Repository shape expected by Codex
Unless the repository later changes intentionally, assume this layout:

```text
/src
  /app
  /platform
  /renderer
    /core
    /pointcloud
    /gsplat
    /passes
    /shaders
  /scene
  /camera
  /ui
  /io
  /style
  /motion
  /rendergraph
  /jobs
  /output
  /serialization
/tests
/assets
/shaders
/docs
```

If the repository is still empty, create files consistent with this structure.

---

## 6. Ownership model
Each Codex task must have a single primary owner. Cross-cutting changes are allowed only when the contract says so.

### A. Bootstrap / Build Agent
Owns:
- `CMakeLists.txt`
- `/cmake/**`
- `/src/app/**`
- `/src/platform/**`
- minimal wiring in `/src/main*`

May touch:
- `/src/renderer/core/**` for bootstrapping only

Must not own:
- point styling logic
- UI behaviour beyond app shell hooks
- GS internals

### B. Renderer Core Agent
Owns:
- `/src/renderer/core/**`
- `/src/rendergraph/**`
- shared GPU resource abstractions
- frame orchestration interfaces

May touch:
- `/src/output/**`
- `/src/renderer/passes/**`

Must not own:
- layer-specific styling semantics
- camera UX
- project serialization schemas unless required for render graph config

### C. Point Cloud Agent
Owns:
- `/src/renderer/pointcloud/**`
- `/src/style/**`
- `/src/motion/**`
- point-cloud shader code under `/src/renderer/shaders/**` or `/shaders/**`

May touch:
- `/src/scene/**` for layer metadata
- `/src/io/**` for point attribute layout

Must not own:
- GS renderer internals
- side panel framework
- shot sequencing

### D. GSplat Agent
Owns:
- `/src/renderer/gsplat/**`
- GS adapter code
- GS asset import adapters

May touch:
- `/src/scene/**`
- `/src/output/**` for pass compatibility hooks

Must not own:
- generic point-cloud code
- UI parameter-binding widgets

### E. Camera + Scene Agent
Owns:
- `/src/camera/**`
- `/src/scene/**`
- picking / pivot inference plumbing

May touch:
- `/src/renderer/passes/**` for point ID / depth helpers
- `/src/ui/**` for shot editor data binding

Must not own:
- render output writing
- PLY parsing

### F. UI Tools Agent
Owns:
- `/src/ui/**`
- side panel state machine
- parameter binding widgets
- preset UI plumbing

May touch:
- `/src/style/**` view models
- `/src/camera/**` UI-facing APIs
- `/src/output/**` UI-facing APIs

Must not own:
- shader semantics
- core renderer memory management

### G. IO + Serialization Agent
Owns:
- `/src/io/**`
- `/src/serialization/**`
- project JSON schemas
- PLY import and field metadata extraction

May touch:
- `/src/scene/**`
- `/src/style/**`
- `/src/output/**` metadata sidecars

Must not own:
- runtime render graph logic
- camera control behaviour

### H. Output + Batch Agent
Owns:
- `/src/output/**`
- `/src/jobs/**`
- offline/tiled render orchestration
- image sequence writing
- AOV selection plumbing

May touch:
- `/src/renderer/core/**`
- `/src/renderer/passes/**`

Must not own:
- side panel framework
- point styling semantics beyond output selection

### I. QA / Validation Agent
Owns:
- `/tests/**`
- validation scenes, golden references, smoke tests
- test harness docs under `/docs/**`

May touch:
- any module only to add test hooks, debug names, deterministic flags, or missing seams required for testing

Must not own:
- feature logic unless the owning agent explicitly delegated a seam

---

## 7. Task contract format
Every Codex task should be written and executed in this format.

```text
Task: <short name>
Owner: <one owner from section 6>
Goal: <single vertical slice>
Inputs:
- docs sections
- files/directories already relevant
Files allowed:
- exact files or globs
Files forbidden:
- exact files or globs
Deliverables:
- code
- tests
- docs updates
Acceptance tests:
- explicit pass/fail checks
Out of scope:
- list
```

Do not start implementation until the contract is narrowed enough that a reviewer can tell which directories should change.

---

## 8. Prompt templates for Codex tasks
Use these prompts directly or with minimal edits.

### 8.1 Bootstrap prompt
```text
Read AGENT.md, project_description.md, PDR.md, and code_plan.md.
Implement the smallest compilable project skeleton for the app shell and build system.
Own only the Bootstrap / Build Agent files.
Do not implement renderer features beyond the minimum wiring needed to launch a window and initialise the Vulkan app shell.
Add smoke-test scaffolding where possible.
Return: files changed, assumptions, compile/run steps, and acceptance-test status.
```

### 8.2 Point-cloud styling prompt
```text
Read AGENT.md and the point-cloud, style, motion, and parameter-binding sections of the docs.
Implement one vertical slice of field-driven styling for point clouds.
Use the shared ParameterSourceMode and FieldMapConfig model.
Support Constant and Field-Mapped modes for the selected parameter.
Keep GPU evaluation logic centralised.
Own only Point Cloud Agent files unless a documented seam requires a minimal shared-interface change.
Return: files changed, data structures, shader changes, tests added, and acceptance-test status.
```

### 8.3 Side-panel prompt
```text
Read AGENT.md and the UI sections of the docs.
Implement the slide-out / pinnable side panel and the reusable parameter-binding widget.
Support hover reveal, pin on double-click, project-state serialization hooks, and global/per-layer editing.
Do not invent new render semantics; use existing style and output interfaces.
Own only UI Tools Agent files unless a minimal interface seam is required.
Return: files changed, user interactions supported, screenshots or notes if available, and acceptance-test status.
```

### 8.4 Camera / shot prompt
```text
Read AGENT.md and the camera/scene sections of the docs.
Implement the camera state, orbit/free-fly/look-at modes, surface-inferred pivot plumbing, and quaternion shot interpolation.
Do not implement unrelated UI polish.
Own only Camera + Scene Agent files unless a render pass seam is required for picking.
Return: files changed, camera math summary, any assumptions, and acceptance-test status.
```

### 8.5 Output / AOV prompt
```text
Read AGENT.md and the output/AOV sections of the docs.
Implement one offline rendering slice that writes beauty, depth, alpha, and at least one selected scalar-field pass.
Support deterministic tiled rendering hooks.
Own only Output + Batch Agent files unless a render-core seam is required.
Return: files changed, passes written, file naming scheme, and acceptance-test status.
```

### 8.6 GS integration prompt
```text
Read AGENT.md and the GS integration sections of the docs.
Wrap the GS subsystem so that it shares scene transforms, camera state, visibility, and output hooks with the rest of the renderer.
Do not convert splats into plain points.
Own only GSplat Agent files unless a minimal scene/output seam is required.
Return: files changed, adapter boundaries, assumptions, and acceptance-test status.
```

### 8.7 QA prompt
```text
Read AGENT.md and the acceptance criteria in PDR.md.
Add or update automated and manual validation coverage for the targeted feature slice.
Prefer deterministic tests, golden images, schema validation, and smoke-test CLI coverage.
Do not rewrite feature logic unless a missing seam blocks testing.
Return: tests added, gaps remaining, and pass/fail status.
```

---

## 9. Milestone decomposition
These are the default vertical slices. Do not skip ahead unless dependencies are already landed.

### Milestone 1 — Foundation
Owner sequence:
1. Bootstrap / Build Agent
2. Renderer Core Agent
3. QA / Validation Agent

Must deliver:
- project structure
- build system
- app shell
- Vulkan context bootstrap
- basic render loop
- smoke tests

### Milestone 2 — Point-cloud ingest + scene core
Owner sequence:
1. IO + Serialization Agent
2. Camera + Scene Agent
3. Point Cloud Agent
4. QA / Validation Agent

Must deliver:
- binary PLY import
- scalar-field discovery
- scene/layer model
- basic point display
- scene bounds
- simple navigation

### Milestone 3 — Side panel + parameter binding
Owner sequence:
1. UI Tools Agent
2. Point Cloud Agent
3. IO + Serialization Agent
4. QA / Validation Agent

Must deliver:
- reveal/pin side panel
- reusable parameter-binding widget
- constant vs field-mapped values
- project serialization of UI state and bindings

### Milestone 4 — Styling + motion
Owner sequence:
1. Point Cloud Agent
2. UI Tools Agent
3. QA / Validation Agent

Must deliver:
- point size, opacity, colour, X-ray controls
- field-driven mapping
- subtle procedural motion from rest position
- preview-safe toggles

### Milestone 5 — Camera shots
Owner sequence:
1. Camera + Scene Agent
2. UI Tools Agent
3. IO + Serialization Agent
4. QA / Validation Agent

Must deliver:
- shot save/load
- quaternion interpolation
- target camera mode
- inferred pivot workflow
- 30 fps timing assumptions

### Milestone 6 — GS integration
Owner sequence:
1. GSplat Agent
2. Camera + Scene Agent
3. Output + Batch Agent
4. QA / Validation Agent

Must deliver:
- GS layer adapter
- shared camera/transform integration
- preview coexistence with point clouds

### Milestone 7 — Offline rendering + AOVs
Owner sequence:
1. Output + Batch Agent
2. Renderer Core Agent
3. UI Tools Agent
4. QA / Validation Agent

Must deliver:
- beauty/depth/alpha
- selected scalar-field pass
- tiled 8K render path
- batch/headless entry point

---

## 10. Acceptance tests by subsystem
These are minimum checks. Add finer tests per task.

### Build / bootstrap
- Project configures on macOS Apple Silicon.
- App launches a window and cleanly exits.
- No hard dependency on Windows-only APIs.

### PLY ingest
- A CloudCompare-exported binary PLY loads.
- Field names are discoverable in the UI/data model.
- Import errors identify the bad file and missing attribute.

### Point-cloud rendering
- A test scene renders points with source colour.
- At least one parameter can switch between Constant and Field-Mapped.
- Mapping a scalar field changes the rendered result predictably.

### Side panel
- Panel reveals when the pointer enters the activation strip.
- Panel stays open while hovered.
- Double-click pins and unpins.
- Panel state round-trips through project save/load.

### Camera / shots
- Orbit, pan, dolly, and free-fly all function.
- Look-at / target mode functions.
- Shot interpolation uses quaternion slerp.
- Saving and reloading shots preserves pose and timing.

### Motion
- Motion is evaluated from rest position.
- Disabling motion returns points to rest.
- Field-driven amplitude/frequency modify visible motion without mutating source data.

### GS integration
- A GS layer loads without conversion to plain points.
- GS and point-cloud layers share the same camera path.
- Layer visibility and transforms work consistently.

### Output / AOV
- Beauty, depth, and alpha write correctly.
- At least one selected scalar-field pass writes correctly.
- Tiled renders stitch without visible seams for a deterministic test scene.

---

## 11. Definition of done
A task is done only if all are true:
1. The task contract is satisfied.
2. Files changed are within ownership rules.
3. Acceptance tests pass or failures are explicitly documented.
4. New public data structures and enums are documented where needed.
5. New UI behaviour is serializable if it affects project state.
6. No unnecessary refactors were mixed into the feature change.

---

## 12. Change control
Update `AGENT.md` when:
- ownership changes,
- the module layout changes,
- milestone ordering changes,
- prompt templates prove repeatedly insufficient,
- acceptance tests become outdated.

Do not bloat this file with feature details that belong in `PDR.md` or `code_plan.md`.

---

## 13. Default review checklist for Codex
Use this checklist before handing work back.
- Did I read the owning docs?
- Did I keep the change inside the allowed files?
- Did I preserve point-cloud vs GS separation?
- Did I preserve Constant + Field-Mapped support where relevant?
- Did I add or update tests?
- Did I avoid platform lock-in?
- Did I document assumptions and follow-ups?
