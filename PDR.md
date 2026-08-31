# PDR.md — Product Design Requirements

## 1. Purpose
Define the functional, UX, and technical requirements for a desktop application that renders point clouds and Gaussian splats in the same scene, supports cinematic shot authoring, and exposes artist-facing field-driven styling through a slide-out side panel.

## 2. Product Goals
### 2.1 Primary goals
- Render large point clouds and Gaussian splats together.
- Support stylised point-cloud rendering driven by scalar fields authored in CloudCompare.
- Provide a side panel for live editing of render and style parameters.
- Allow each eligible parameter to be edited as either a constant or a field-driven mapping.
- Support smooth camera shot interpolation for exhibition rendering.
- Prioritise responsive iteration on Apple Silicon M1.
- Support higher-end offline rendering on Windows machines if desired.

### 2.2 Secondary goals
- Preserve project portability between macOS and Windows.
- Export standard AOVs and selected point-cloud data passes.
- Support reusable style presets and shot-based workflows.

## 3. User Profile
Primary user:
- technically proficient visual researcher / designer,
- comfortable using CloudCompare for preprocessing,
- wants to artistically interpret point clouds using scalar fields,
- needs both exploratory navigation and controlled shot authoring,
- wants M1-friendly look-development and optional stronger offline rendering elsewhere.

## 4. Version 1 Scope
### In scope
- C++ desktop application.
- Vulkan rendering, using MoltenVK on macOS.
- Integration of a GS subsystem based on `3dgs-vulkan-cpp` or equivalent.
- PLY point-cloud import.
- Scalar-field discovery and statistics.
- Point-cloud styling driven by constants or scalar-field mappings.
- Slide-out / pinnable side panel.
- Saved style presets.
- Saved camera shots and interpolated shot playback.
- Interactive preview at approximately 1080p.
- Offline render at 8K, including tiled render path.
- Standard AOV output plus selected field-based passes.

### Out of scope
- Native E57 import in v1.
- Native LAS/LAZ import in v1.
- Full node-based material graph.
- Physics simulation.
- Distributed farm management beyond command-line batch rendering.
- Full nonlinear sequence editor.

## 5. Functional Requirements
### 5.1 File IO
#### FR-IO-1
The application shall import point clouds from binary PLY.

#### FR-IO-2
The application shall read point positions and colours when present.

#### FR-IO-3
The application shall discover and expose scalar fields from the imported point cloud.

#### FR-IO-4
The application shall import one or more Gaussian splat assets via the GS subsystem.

#### FR-IO-5
The application shall save scene, shot, style, side-panel, and render configuration to disk.

### 5.2 Scene and Layer Model
#### FR-SCENE-1
The application shall support multiple point-cloud layers.

#### FR-SCENE-2
The application shall support multiple Gaussian splat layers.

#### FR-SCENE-3
Each layer shall support visibility, transform, naming, and selection.

#### FR-SCENE-4
Point-cloud layers shall support static and dynamic motion modes.

#### FR-SCENE-5
Point-cloud layers shall preserve rest position for non-destructive procedural motion.

#### FR-SCENE-6
Only one available grouped LiDAR scene shall own interactive point-cloud
residency. Making another scene authoritative shall release every CPU/GPU
point-cloud session and queued load stored directly in the outgoing scene
folder; an in-flight whole-file result that finishes after ownership changes
shall be discarded before publication. Project restoration shall replay loaded
state for at most one resolved active scene.

#### FR-SCENE-7
Delimiter-bounded `WATER` point clouds in a grouped scene folder shall form one
density-aware auxiliary family. Live display shall load only an exact spacing
match to the committed base bundle, or the nearest available spacing when no
exact source exists. Recovery copies identified by `old`, `backup`, `archive`,
or equivalent recovery tokens shall remain undiscovered.

#### FR-SCENE-8
Final output shall resolve the finest complete ROCK/SAND/VEG bundle plus the
WATER variant nearest that density. WATER footprint shall follow nominal point
spacing and its opacity and emission shall use measured point-count coverage
relative to the finest installed WATER source, so live and output densities
retain equivalent visual weight.

### 5.3 Navigation and Camera
#### FR-CAM-1
The application shall support orbit, pan, dolly, and free-fly navigation.

#### FR-CAM-2
The application shall support explicit look-at / target cameras.

#### FR-CAM-3
The application shall support surface-inferred pivots from screen centre or cursor-based picking.

#### FR-CAM-4
Surface-inferred pivot selection shall estimate a stable target from the closest valid visible points when available.

#### FR-CAM-5
If no stable pivot is found, the system shall fall back to current pivot, object centre, or scene centre.

#### FR-CAM-6
The application shall allow the user to save named shots.

#### FR-CAM-7
Shot rotation shall be stored and interpolated using quaternions.

#### FR-CAM-8
The default playback and render timebase shall be 30 fps.

#### FR-CAM-9
The shot system should support both free-orientation shots and target-driven shots.

#### FR-CAM-10
The Animation Keys panel shall allow the selected key to adopt the live camera
pose. Focus shall use the first stable point-cloud hit along the live view ray,
falling back to the selected key's previous camera-to-focus distance when the
ray misses.

Selecting a viewport focus key shall expose a world-Z quarter-arc rotation
handle from +Y to +X on its XY plane. Dragging it shall orbit the key camera
around that focus while preserving Z offset and focus distance. Selecting
either the camera or focus control shall draw a thin camera-to-focus line and a
cube one-third of the way from focus to camera. The line shall be clipped to
the live viewport rather than suppressed when either endpoint is off screen;
it shall also remain visible when both endpoints are outside but their segment
crosses the view. The cube shall carry the
ordinary world-axis, world-plane, and view-plane translation controls; dragging
any of them shall apply one equal delta to camera and focus, preserving their
relative geometry, authored orientation, and lens values. The ordinary gizmo
at the selected node shall keep moving only that selected control. The former
detached secondary plane handles shall not be drawn. Double-clicking a focus
key shall enter a transactional Live Edit Mode at the complete evaluated
key camera and lens state. Live camera navigation shall update the key preview;
Enter or a tick commits, while Escape or a cross restores the prior animation
and view. The live view shall show a faded red border and top-right mode card
for the duration of the edit.

#### FR-CAM-11
Matching-Frame Key Alignment shall provide an **Extend Both Seams** assistant
for two target-driven, fixed-lens animations and one unambiguous working
point-cloud scene with either CPU analysis samples or a visible committed
display source. A shared association is preferred; the selected or sole
pickable scene is accepted for older unassociated paths. The animations need not already have velocity-blend metadata;
their crossed endpoints are treated as the two existing 50%-blend seam poses.
The adjacent **Fix A+B Lens** action shall normalize each animation to one
median authored FOV/focus-distance/aperture profile, normalize both to the
least restrictive combined clip range, and remove lens-only spline tangents
without changing camera/focus positions or timing. If that repair alone makes
the pair eligible, Extend shall apply it to private wizard baselines; Cancel
shall discard it and Apply Both shall commit it with the extension.

The Animation tab shall persist each animation's preferred Blend Partner even
without reciprocal velocity-link metadata and restore it when that animation
is loaded. A live reciprocal link shall override the preference while linked,
and unlinking shall retain the former link partner as the preference. Flipping
the displayed A/B roles shall resolve each path by stable file identity rather
than silently substituting another registered animation.

On the ordinary paired matching-frame timelines, right-clicking a key shall
select it by stable animation-file and key identity and show a red outline.
The alignment area shall show live values for the active scrub frame, the
selected node, and the copied snapshot: geometric camera-to-focus distance,
polar angle from world +Z, signed horizontal angle from the evaluated local
path tangent, camera/focus height above the authored ground reference, and
ground sample Z.
Both heights shall use the same ground value sampled at the horizontal
projection of the point one-third of the way from focus to camera, with only a
bounded nearest-cell tolerance for a small sampling hole. The authored SAND
channel in the shared 10 mm cache shall be authoritative. A MESH/MESHSampled
Ground cell may substitute only when that exact retained cell is marked as
vegetation-supported; unclassified MESH and MESH over ROCK/SAND shall never
drive camera clearance. If the reference has no usable SAND or qualified MESH,
sampling shall advance toward the camera until it finds one or reaches the
camera. The selected-node readout shall name the source, state whether it
advanced, and explain cache, geometry, or coverage failures rather than showing
an unexplained blank. **Copy Camera Alignment** shall freeze all measurements
and evaluated orientation by value; subsequent edits to the source node shall
not alter the clipboard. **Paste Camera Alignment** shall offer independent
checkboxes for distance, polar angle, camera ground height, focus ground
height, tangent-relative angle, and horizon/roll. It shall preserve unchecked
focus-relative destination components before ground constraints are applied:
focus height translates the rig vertically and camera height then sets camera
clearance exactly. It shall preserve target-driven orientation policy and copy
horizon/roll only when the destination path already authors orientation.

#### FR-CAM-12
The assistant shall collect ordered anchor/front/side source triangles at
A-start and B-start. One corresponding feature-centre click at each destination
end shall rotate those physical offsets into the destination camera frame;
generated nodes remain individually editable. Each seam shall receive a fitted,
synchronized source/destination scrub review before proceeding. That review
shall expose generated and nearby authored keyframes on two selectable
timelines, keep the paired 50% controls coordinated by the three-node triangle,
and permit a spatial-only one-seam smoothing preview without changing segment
frames. The review shall report each side's endpoint alignment move, eased-key
count, and peak incoming drift speed, provide a ping-pong seam transport with
single-frame steps beside the synchronized offset, and allow direct viewport
editing of enabled keys (camera or focus) with paired midpoints carried
through the captured triangle transform. A single action shall flip the full
A/B frame at the matched seam pose; during orbit inspection that flip shall
instead move the free camera by the rigid matched-camera transform so the
scene appears to swap in place beneath an unchanged vantage, with both matched
seam camera frusta drawn as wireframes. The selected authored controls shall
carry into the final joint fit. The final fit
shall align each existing destination end to the opposite source start and
constrain signed screen X/Y velocity, in-plane patch rotation, overlay position,
and perspective scale on the 30 fps animation timebase.

#### FR-CAM-13
Each successful reciprocal seam shall add two unlinked keys at each generated
end for a simple source span, or three at that end when the span crosses an
authored key or screen-motion inflection. The same ordered triangle pair shall
generate the source animation's pre-roll and the destination animation's tail,
with the original crossed endpoints remaining the temporal midpoint. It may
align the former first/last poses. Because the two 50% poses are never
perfectly aligned, each seam shall expose an **alignment ramp** (defaulting to
the seam span) that eases the endpoint alignment move across the authored keys
inside its window as fractional localized corrections with matched tangents;
the base spline, every segment frame count, and the path outside the window
stay exact, and a zero ramp reproduces the legacy single-segment behaviour.
Each seam shall also expose an **alignment share** that solves both 50%
endpoint cameras toward a blended screen-space triangle target:
destination-only (legacy), meet-halfway (default, halving each side's
transient), or source-only. Interior authored key frames and bulk motion
shall keep their original spacing and 30 fps speed until the optional final
whole-cycle duration fit described in FR-CAM-20.

#### FR-CAM-14
Extension candidates shall remain immutable and outside the animation registry
until Preview. Apply shall create both paired `_Edited` paths atomically; Back,
Cancel, or Escape shall restore the launch camera, playhead, pivot, and ghost
state without mutation.

#### FR-CAM-15
During the assistant, ordinary inspection scrubbing shall snap back on release,
seam-span frame pickers shall commit integer frames, and generic playback,
key editing, file switching, saving, and alignment controls shall remain locked.

#### FR-CAM-16
Extending a camera path shall preserve every pre-existing authored timing,
water, and effect value at the frame of the same old camera pose. Their
normalized coordinates shall shift by the prepended frame count and rescale to
the new duration; start/end sentinels continue through the generated ends.
Explicit export bounds shall shift by the prepended count so they select the
same old camera content. Camera position shall select keyed values only: procedural waves,
Rain, trails, and related motion shall continue from the steady live clock even
while the camera is paused, and shall use the full deterministic output sample
time during offline export.

#### FR-CAM-17
Apply shall create reciprocal blend metadata when A and B were unlinked, or
update their existing reciprocal pair. The overlap metadata, eligible terminal
key IDs, saved-animation link mirror, and pair dependency shall update together
so A, B, and the project save or discard as one transaction. Save As on this
pending dependency shall create two newly named, reciprocally linked animation
files, retarget their partner filenames, preserve the calculated start/end
blend durations, and save both files with the project atomically.

#### FR-CAM-18
Before fitting, the assistant shall let the user flip between A and B, select
any of the three correspondence nodes, re-raycast it from the viewport, and
move it with a world-axis gizmo. Triangle stages shall also provide a temporary
orbit-inspection view: drag navigation may leave the authored perspective, a
short click shall still capture the armed world-space node, and an explicit
Return action shall restore the exact animation camera for that stage. The
temporary view shall never alter either animation pose. Candidate fitting shall
not begin until both ordered endpoint triangles and both seam spans are defined.

#### FR-CAM-19
Each fitted-seam review shall offer a two-camera A/B split. Its horizontal
feature-relative offset shall be editable, while its hard boundary continues
to follow the paired anchor projections during synchronized scrubbing,
including while the anchors are outside the image. Both correspondence triangles
shall remain visible and the active A/B side shall remain editable. Procedural
water rendering and simulation work shall be suppressed while the assistant is
open.

#### FR-CAM-20
Final Preview shall offer an optional selected-key transition-smoothing pass.
The two keys at each exact triangle-aligned midpoint shall be a coordinated
movable pair rather than fixed poses. Generated keys, both midpoint pairs, and
up to two neighbouring authored keys on each side shall be recommended as
movable. Toggling either midpoint key shall toggle its partner. The user may
toggle other eligible keys per animation and set a bounded movement allowance.
Every enabled key shall also be manually movable; moving either member of a
midpoint pair shall move its counterpart through the captured triangle
transform. The final view shall redraw every affected spline segment from the
current evaluated candidate, including exact key knots. A force-alignment
action shall keep the selected A or B reference path fixed and solve the paired
midpoint camera/focus controls so all three projected triangle nodes overlap
again, without changing timing or locked keys. Spatial-only actions shall equalize X velocity, Y velocity, X plus
rotation, image rotation, or perceived speed under each supported speed method.
They shall move only green controls and shall never modify segment-frame
weights. A/B motion graphs shall mark both 50% poses. A signed -1..1 transport
shall run the complete A/B cycle, activate the appropriate feature-following
hard split at either seam, retain independent per-seam boundary offsets, wrap
repeated drags on release, and loop with Space. Reset shall restore the
immutable triangle fit. Applying without a successful pass shall preserve the
unsmoothed fit, and applying either result shall create/update reciprocal blend
durations from the generated spans. Final Preview shall default to an optional
4:00 whole-cycle fit. It shall scale both authored bulk timelines and every
generated seam half by one common factor, apportion integer segment frames so
the unique A/B cycle is exactly 7,200 frames, and scale reciprocal overlaps and
finite export bounds with them. Camera/focus/lens geometry, key order, triangle
alignment, and the independent procedural-effects clock shall not change.

After Apply, every valid reciprocal pair shall expose a persistent **Seamed
View**. While either linked animation is scrubbed or played inside a saved
start/end overlap, the viewport shall evaluate the exact reciprocal partner
frame and composite a moving hard split. Newly applied pairs shall carry the
captured anchor positions into the linked view for the active session; links
without those transient observations shall use saved overlap progress as a
deterministic fallback. The ending animation shall remain on
the left and the starting animation on the right at both seams. The boundary
shall pass through the aligned midpoint at 50% overlap progress, retain one
artistic pixel offset per physical seam when the loaded A/B role reverses, and
fall back to the ordinary single-animation view outside the overlap.

The linked-view controls shall appear above Global Animation Position. A
reciprocal pair shall use one shared Timing Take over its unique cyclic
duration while each member retains its own local camera timeline. Local live
and export evaluation shall map through the member's signed loop window and
shall use virtual adjacent-cycle keys so boundary values and derivatives retain
the authored Hold, Linear, Smooth Step, Smooth Velocity, Catmull-Rom, or manual
spline behavior. Camera position selects keyed settings only; procedural
Water, Rain, and trail motion shall continue on their independent live or
deterministic export clocks.

Selecting a Timing Take from either member of a reciprocal pair shall assign
that same reusable take to both members as one atomic animation edit. A take
already authored for a linked loop shall be reused unchanged; any other take's
normalized `0..1` domain shall become one complete target loop and repeat
cyclically. Assignment shall not clone, rotate, retime, or otherwise rewrite
the take's keys, handles, setting clips, scene states, or Rain profile. The
full-loop editor shall remain available immediately after assignment, and Save
or Discard shall keep the two animation selections together.

#### FR-CAM-21
Every valid reciprocal pair shall expose a session-only **Seam / A / B** view
selector above Global Animation Position. A and B identity shall be established
by lexical order of the normalized full animation file paths and shall remain
stable whichever member is loaded; this display identity is independent of
which member owns shared cycle frame zero. Seam mode shall display the existing
hard-split compositor wherever both exact reciprocal occurrences are available
and the available single member elsewhere. A and B mode shall isolate only that
finite member. Switching mode shall preserve the canonical cycle frame and
matched visual pose while changing the displayed position to the selected
member's local coordinate. An unavailable member occurrence shall not be
manufactured by clamping to an endpoint. This selector shall remain independent
of the compiled linked-animation **Live Camera** Overlay/A/B facility and shall
not change that facility's saved behavior.

The canonical Seam position shall use the signed `-1..1` cycle. Signed zero
shall be the midpoint of lexical member A's start / member B's end overlap.
Signed `-1` and `+1` shall be equivalent copies of the midpoint of A's end /
B's start overlap. Conversion to either local member shall use its exact saved
timing window; where a synthetic timing window admits repeated local
occurrences, resolution shall choose the occurrence nearest the previous local
playhead. The selected view and local-display projection shall not be written
to project or animation documents.

#### FR-CAM-22
The linked Global Animation Position shall provide one pair-keyed cyclic focus
lens over the canonical signed `-1..1` domain. A narrowed lens shall be able to
span zero or wrap through the equivalent `-1` / `+1` boundary, and all Water,
Timing, and visual-setting key timelines shall use that same focused
canonical interval. Selecting A or B shall not rebase the lens to that member's
local `0..1` duration. The lens shall initialize to the full range for a
different canonical pair, remain session-only, and never be serialized.

#### FR-CAM-23
Seam-mode playback shall wrap and loop over the complete unique reciprocal
cycle. A-only and B-only playback shall remain finite, preserve that member's
saved local timing window, and stop at its local end instead of wrapping into
the partner. Live preview and immutable/frozen export evaluation shall map the
same canonical cycle phase into the shared Timing Take and visual state.
Virtual previous- and next-cycle keys shall preserve values and derivatives at
phase zero and at both local path boundaries. A member export shall continue to
use its saved local start/end frames; the view mode and canonical transport
shall not extend or replace its output bounds.

The production acceptance pair `Proj_A_09S01` / `Proj_B_09S01` shall be treated
as an existing exact four-minute loop at 30 fps: member durations 6,868 and
6,230 frames, less reciprocal overlaps 3,080 and 2,818 frames, yield
`6,868 + 6,230 - 3,080 - 2,818 = 7,200` unique frames. No `09S02` retimed copies
are required for that pair. The optional wizard 4:00 fit in FR-CAM-20 remains
available for other pairs whose unique duration is not already 7,200 frames.

#### FR-CAM-24
Every animation with a grouped scene containing complete 1 mm/5 mm bundles
shall expose an independent session-only **HQ** control. A reciprocal pair
shall show it beside Seam/A/B; an unlinked animation shall show it in its local
animation view. Preparation shall begin automatically after the complete 5 mm
baseline and higher-priority display/shared-cache work are ready. Until every
requested role patch is ready, the disabled control shall show monotonic
progress and its Waiting, Scanning, Organising, or Uploading stage; a failure
shall retain the pure 5 mm view and offer retry details.

HQ off shall render the complete 5 mm ROCK/SAND/VEG bundle. For a reciprocal
pair, HQ on shall render 1 mm ROCK and VEG point centres inside the union of
the two member cameras evaluated at normalized
`0.5`, and render 5 mm ROCK and VEG outside that union. For an unlinked
animation, the union shall instead contain nine evenly spaced camera views
including normalized `0`, `0.5`, and `1`, plus every authored camera-key time,
so the higher-density patch spans the complete finite path. A project-saved
patch spacing of 1, 2 or 3 mm
selects the inside density: 1 mm keeps every point, while 2 mm and 3 mm thin
the scanned 1 mm points with the density-preserving cell stratification of the
display-density cache (cells of the chosen spacing keep a hash-dithered quota
of one in four or one in nine of their parents, a single output being the real
parent nearest the cell's parent centroid) and declare that nominal spacing
with the exact kept/scanned counts so the existing density compensation
restores the 1 mm reference coverage. Changing the spacing shall re-prepare the
patches and restore an enabled HQ view once they publish. Each camera shall use
its authored live-view aspect ratio, falling back to the current live aspect
only when absent. The
viewport shall extend by 5% of its full width/height beyond every X/Y side;
authored near/far clipping shall not be extended. The same exact centre test
shall construct the 1 mm selection and complementary 5 mm mask, leaving no
overlap or gap. A project-saved **Sand** toggle shall be off by default. While
off, HQ shall retain complete 5 mm SAND and never fingerprint, open, or allocate
1 mm SAND. While on, it shall apply the same view union, spacing, 1 mm selection,
and complementary 5 mm mask to SAND.

All requested ROCK/VEG or ROCK/VEG/SAND patches and masks shall publish as one
live-resource transaction. Enabling or disabling HQ shall preserve Seam/A/B mode, canonical
and local playheads, camera, and playback, while resetting only temporal and
compositor history. Internal layers shall reuse the corresponding role's
resolved Visual, field names and full-role mapping ranges, Timing Colourise,
Rain, Seepage, Flow/roughness motion, renderer mode, depth of field, and EDL.
An included SAND patch shall also retain ordinary Shoreline eligibility and
resolved wave settings.
The 5 mm remainder shall retain normal density compensation and the 1 mm patch
shall use identity compensation. Retained source indices shall support later
indexed field gathering without retaining or rescanning a complete in-memory
1 mm cloud.

The patch cache shall remain memory/GPU-only for the current active animation
or pair and shall be cancelled/rebuilt when that selection, any contributing
sampled camera/aspect, project identity, saved Sand mode, or a relevant source
fingerprint changes. A hidden 5 mm override may be loaded for this purpose
without changing the saved Visible Point Cloud selection.
Internal layer ids, masks, and that override shall never enter serialization or
export snapshots. Every still, frame preview, video, and EXR export shall keep
the existing complete canonical 1 mm ROCK/SAND/VEG path. The spacing and Sand
preferences may be serialized without a schema-version increment.

#### FR-CAM-25
The application shall expose a mutually exclusive experimental **aHQ** live
mode while retaining fixed HQ. aHQ shall work without a loaded animation and
shall resolve the selected compatible grouped scene, or the only visible
compatible grouped scene when unambiguous. Its fine-data union shall always
include a guarded current-camera view. Any loaded animation associated with
that scene may add path prefetch coverage but shall not constrain where aHQ
can operate.

aHQ shall retain the complete 5 mm baseline and select fine/coarse points with
a camera-stable GPU transition derived from projected 5 mm spacing. Fine source
preparation shall be depth-bounded beyond that transition. Leaving a published
camera guard shall hide the stale fine patch, preserve the complete 5 mm view,
and request a background replacement without repeatedly cancelling one active
block request. On first use, each fine source shall produce one machine-local,
Morton-ordered cache PLY and a sidecar block index under
`Saved/.invisible_places/cache/adaptive_hq/`. Blocks shall be approximately
1--8 MiB and expose bounds, file offset, and point count so guarded camera
requests can seek only intersecting ranges. Recently used blocks shall be
retained so camera movement normally loads only its new fringe.

The spatial cache shall never be written to OneDrive, beside a source, or over
an export-quality PLY. It shall be rebuilt automatically whenever the source
path, size, timestamp, property schema, point layout, or content fingerprint no
longer matches its sidecar. aHQ shall share HQ's 1/2/3 mm and Sand preferences
and remain serialization-free and excluded from every frozen export path; only
its validated derived disk cache may persist between sessions.

### 5.4 Side Panel Behaviour
#### FR-UI-1
The application shall include a side panel for render and style controls.

#### FR-UI-2
The side panel shall reveal when the mouse enters an activation strip near its edge.

#### FR-UI-3
The side panel shall remain visible while hovered.

#### FR-UI-4
The side panel shall pin open when double-clicked on its tab, title area, or equivalent interaction target.

#### FR-UI-5
The side panel shall unpin when double-clicked again or explicitly closed.

#### FR-UI-6
The side panel state shall be serializable in the project file.

#### FR-UI-7
The side panel shall remain usable during navigation and playback when pinned.

### 5.5 Side Panel Sections
#### FR-UI-8
The side panel shall expose sections for:
- Scene
- Layers
- Camera
- Shot
- Style
- Field Mapping
- Motion
- Preview
- Render
- Output / AOV
- Presets

#### FR-UI-9
The side panel shall expose both global render controls and per-layer overrides.

#### FR-UI-10
The side panel shall provide a reusable parameter editor for all field-drivable values.

#### FR-UI-11
Each saved animation shall store a default live-view window size and request
that size when loaded.

#### FR-UI-12
The Project section shall store a preferred live-view window size and expose a
lock that overrides animation defaults and restores the project size after a
manual resize.

#### FR-UI-13
The camera-view toolbar at the top of the controls panel shall expose an
accessible **Grid** toggle for a live-view composition guide. The guide shall
draw vertical and horizontal lines at one third, halfway, and two thirds, with
the halfway lines visually stronger than the thirds. Adjacent settings shall
allow halfway and thirds to be shown independently and shall expose colour,
opacity, and line-weight controls. The guide shall be preview-only and shall
not appear in frame previews or exported output.

#### FR-UI-14
Every numeric slider in the Water features UI shall enter native numeric text
editing when its slider bar is double-clicked. Controls with an exact normalized
`0..1` range shall display three decimal places without rounding the stored
single-precision value; Water keys and their interpolation shall use that
underlying value rather than the display text.

### 5.6 Parameter Source Modes
#### FR-PARAM-1
Every eligible numeric or vector render parameter shall support `Constant` mode.

#### FR-PARAM-2
Every eligible parameter shall support `Field-Mapped` mode when the active layer exposes scalar fields.

#### FR-PARAM-3
Field-Mapped mode shall allow selection of a scalar field source.

#### FR-PARAM-4
Field-Mapped mode shall allow independent input range and output range editing.

#### FR-PARAM-5
Field-Mapped mode shall support clamp and invert toggles.

#### FR-PARAM-6
Field-Mapped mode should support gamma or curve shaping.

#### FR-PARAM-7
The renderer shall evaluate field mappings on GPU when feasible.

#### FR-PARAM-8
The UI shall show field statistics such as discovered min and max values.

#### FR-PARAM-9
The UI should provide reset-to-constant and quick-normalise actions.

#### FR-PARAM-10
Field-Mapped mode shall show the selected field's value distribution as a histogram with directly draggable lower and upper input bounds, defaulting to the field's discovered minimum and maximum.

#### FR-PARAM-11
Manually edited input bounds shall be remembered per parameter and field, so switching a parameter to another field and back restores the bounds it was last edited with; fields left in layer-stats mode return to their discovered range.

### 5.7 Point-Cloud Styling
#### FR-STYLE-1
The renderer shall support at least the following point style parameters:
- point size,
- opacity,
- colour control,
- emissive intensity,
- X-ray strength,
- depth fade,
- additive weight,
- jitter amplitude,
- vibration amplitude,
- vibration frequency,
- phase offset.

#### FR-STYLE-2
Point size shall support screen-space and world-space modes.

#### FR-STYLE-3
Colour shall support source colour, constant colour, field ramp, and source-colour modulation.

#### FR-STYLE-4
Opacity shall support constant and field-driven control.

#### FR-STYLE-5
The renderer should support multiple visual modes including solid dots, discs, X-ray, additive accents, and depth-cued looks.

#### FR-STYLE-6
Styles shall be saveable and reusable as presets.

#### FR-STYLE-7
The Timings tab shall support one ordered list containing both Colourise and
Emissive scalar effects. Each effect shall have an enable toggle and an
inclusive animation activation range.

#### FR-STYLE-8
An Emissive timing effect shall expose a non-negative, smoothly keyable level
and smoothly keyable scalar-field bounds/fade controls, without exposing a
palette. It shall add emission without changing point opacity.

#### FR-STYLE-9
Keys outside a timing effect's activation range shall remain editable and
shall determine the smoothly evaluated value at each activation boundary.

#### FR-STYLE-10
Colourise and Emissive effects shall share a recommended maximum of five
simultaneous effects. The UI shall identify ranges above that recommendation,
and preview and export shall evaluate both kinds on GPU where feasible.

#### FR-STYLE-11
Scrubbing a keyed Water feature or Timings visual-feature timeline shall always
advance the one shared animation position and evaluate every active keyed
feature. It shall move the live camera only while that camera still matches the
animation frame active before the scrub. After orbit, pan, or dolly navigation,
feature scrubbing shall preserve the inspection view. Global Animation Position
and camera-key editing shall always move the camera. A shared **Always Follow
Camera** override shall appear beside **Subtle Selected Cues** in Water and
beside **Split Graphs** in Timings. Ordinary Play/Space transport shall follow
the live camera only when it matches the active animation frame at playback
start, shall detach without snapping back if the user navigates away during
playback, and shall continue advancing the shared playhead and keyed features
while detached.

#### FR-STYLE-12
Timings feature runs shall support settings clips: named normalized spans that
group one feature's keyed-setting keys by explicit clip id for whole-clip
manipulation. Clip windows may overlap and pass across one another without
capturing or deleting each other's keys. A Clips
lane in the Timings tab and the embedded per-feature timelines shall offset,
stretch, duplicate, and retarget clips by direct drag, including marquee and
modifier-click multi-selection, group offset and anchored group scaling with
normalized-domain clamping and exact same-track key collision protection,
Alt-drag duplication, and drag-between-rows transfer restricted to features
of one kind. Selecting a clip shall also select all of its active owned key
nodes for bulk interpolation or keyboard deletion; modifier-clicking graph
nodes shall toggle them individually. A focused settings graph or Clips lane
shall support platform Select All, Delete/Backspace, Escape, plain-click
replacement, modifier-click toggling, and Shift-click range selection without
capturing shortcuts from Global Animation Position. Selecting exactly one
stored clip shall attach newly authored keys for that feature to it; otherwise
new keys shall remain loose and visible in a dashed loose-key block alongside
clips. Clip keys shall use distinct square and diamond graph markers, and their
first and last key shall derive the clip bounds. Blocks
shall indicate the primary keyed value by fill brightness, not curves. A clip
shall save as a reusable package that records its native length; package
identity shall be feature kind plus name, so same-named packages for different
feature kinds coexist. Packages shall apply to any same-kind feature at any
position and stretch to any length. Clip operations shall rewrite only the
grouped keys — evaluation,
per-key editing, and compiled export snapshots are unchanged — and Timing
Take retiming shall carry clip bounds with their keys. Project schema 74 and
water-sources schema 28 introduced clips and package lengths; project schema
76 and water-sources schema 30 store explicit per-key `clip_id` ownership.
Deleting a scene Water feature shall remove that exact kind-and-object
timeline from every legacy scenario and Timing Take scene state while
retaining authored run shells and reusable saved packages. Owner-shadow
cleanup shall never remove a same-id package belonging to another feature
kind.

#### FR-STYLE-13
Each keyed Water setting shall offer **Spline Handles** alongside Smooth Step,
Linear, Hold, Monotone Spline, and Centripetal Catmull-Rom. The mode shall use
persisted incoming and outgoing cubic-Bezier controls; selecting a participating
key shall expose its circular controls on the value graph, and dragging a
control shall change curve time/value shape without moving the key. The graph's
time-only key markers shall occupy a separate lower rail with non-overlapping
hit targets even when the keyed value is at its minimum. Project schema 75 and
water-sources schema 29 store the handle time fractions and value offsets. New
setting tracks and their inherited keys shall default to Monotone Spline;
serialized and migrated older tracks shall retain their authored curve style.

#### FR-STYLE-14
Water key authoring shall retain one atomic edit of session history. Ctrl+Z on
Windows/Linux and Cmd+Z on macOS shall restore the state before the latest key,
keyed-setting, or settings-clip edit, and pressing the same shortcut again shall
reapply it. Continuous slider, key, spline-handle, and clip drags shall each form
one edit. Focused text and numeric inputs shall keep their native field-level
undo instead of triggering the timeline history.

#### FR-STYLE-15
Rain shall remain one active simulation rather than a collection of concurrent
objects. A project-owned Rain profile shall store the complete authored runtime
and visual snapshot, with a stable identity independent of its display name.
Each Timing Take shall reference one shared base profile; its first Rain edit
shall create or update one owner copy named `<base>_<Timing Take>`, leaving other
takes on the shared base. Renaming, duplicating, and deleting a Timing Take shall
rename, duplicate, or remove only its owner copy and preserve surviving
assignments. Saving an edited copy under a shared name shall promote its exact
snapshot and return the take to that shared profile; Discard shall return the
take to its base and remove the temporary copy. Timing Take scene states shall
continue to own Rain scalar keys,
which overlay the take's resolved profile at evaluation time. Loading a project
without the profile library shall migrate the legacy singleton Rain settings and
its embedded visual values into one shared base without manufacturing one copy
per take. Project schema 78 introduces the Rain profile library and Timing Take
profile assignments; the legacy singleton shall remain a compatibility
projection. Standalone Water Sources schema 31 shall carry the same profile
library plus assignment-only Timing Take records; loading a schema-30 singleton
shall create one shared profile without creating any Timing Take record. Live
preview and every frozen export shall resolve exactly one profile into the
existing Rain simulation.

#### FR-STYLE-16
Each Timing Take scene state may own short named timeline markers in its
normalized animation domain. A marker applies to the whole take — it draws on
the global animation bar and on every Timings and embedded feature timeline
regardless of run or feature focus — persists with the Timing Take scene
state, follows reciprocal-loop mapping and retiming, and survives
linked-state merging with stable take-local identity. While a take is active,
M inserts a marker at the current frame with a unique default `Marker NN`
name; single-click selects, dragging retimes, double-click renames, and
Delete removes it. Project schema 90 adds the optional take-level `marks`
array; the schema-79 run-scoped `marks` arrays of older projects hoist into
the owning take on load, and projects without either load with no markers.

#### FR-STYLE-17
A Timing Take scene may restrict Water visibility to features assigned to its
Feature Runs. Disabled runs retain their visibility membership while producing
no keyed samples. Global Shoreline, Seepage, and Flow assignments act as
category umbrellas; an individual assignment exposes only that object while
retaining its category-level master. Rain and Mesh Flow require their own
assignments. Project schema 80 adds the optional
`only_show_water_features_in_runs` scene-state flag; older projects default to
the unrestricted behaviour.

#### FR-STYLE-18
Visual Feature palettes shall support per-source private `_edited` variants
for built-in presets and saved project palettes alike, keyed by source kind
plus id. While a variant exists its original shall be view-only until the
variant is deleted; switching between any original or variant shall never
discard work. Save shall overwrite a saved original from its variant and
retire the variant; deleting a saved original beside a variant shall ask
whether the variant goes too. The palette shall additionally offer, without
moving any colour marker: a Loop toggle that mirrors the sampled output so
the left colour sits at the centre and the ends match; and a Palette Skew
warp edited in a band attached to the histogram bottom, where evenly spaced
palette-coordinate notches bunch and spread over the sampled palette strip
(whose opacity follows the colourise amount). The warp is a monotone curve
through draggable nodes: each node skews left/right and spreads or pinches
up/down its surrounding area, snapping onto neutral spread and its unskewed
home; double-click adds a node anchoring the coordinate that lands there,
or removes one. The centre node's coordinates stay keyable as Skew Centre
and Skew Spread. Bounds edge fades shall be independent per edge and keyable.
An inward fade shall stop at 100% of the selected spacing, while an outward
fade may exceed 100% so a narrow selected interval can feather through the
rest of a scalar field; they stay linked through one recoloured "Ends Fade"
control until a fade handle is
double-clicked apart, splitting into per-edge tinted columns. Every
geometric bounds coordinate shall remain text-editable in any Bounds Keying
mode, writing through the keyed pair. Palette marker Positions shall key as
one complete group through a vertical `+ / < / > / X` rail beside the palette;
Colour and Colourise Amount shall remain independent stop tracks. Every marker
Position curve shall share the min/max of only the marker tracks whose authored
values vary, excluding invariant group-keyed markers from that axis. Other
keyed palette-stop properties shall join the shared value graph as one
auto-ranged curve per track with the full curve-style set (Linear and Hold are
honoured stored styles, keys are authoritative at their exact instant, and new
keys default to Monotone Spline); keyed stop colours shall interpolate in an authored sRGB,
linear-light, or OkLab space and tint their curve. Emissive output shall
support a falloff profile over the bounds: level-multiplier nodes joined by
a Monotone Spline and shaped by the falloff's own skew warp. The curve shall
support click and window selection of nodes plus Delete/Backspace removal.
Authoring any falloff key shall store Position and Level for every node as one
complete snapshot; all coordinates shall interpolate between those shared
instants, while the unified timeline shall draw only node-coordinate tracks
whose stored values change. The result shall be previewed as a mid-grey
response strip atop the histogram.
Value-graph time markers shall hang below the axis, select every key at
their time (or a pre-selected subset), and drags shall snap onto other
keys' times unless a dragged key would stack on its own setting, with a
mouse shake toggling snapping. The selected feature's interactive controls
shall be ordered as Bounds/profile controls, histogram, enabled Colourise
palette, enabled Emissive falloff curve, and one unified keyframe timeline;
scalar numeric settings shall occupy a separate block below. When Colourise is
enabled, its preset and saved-palette selectors shall sit directly below
Bounds Profile and above Histogram Axis. Position, Fade, Skew, Palette, and
Intensity key groups shall be independently showable for the session without
deleting keys; Palette and Intensity visibility controls shall appear only
for their enabled output aspects. Each group shall snap only to its own kind
on the shared graph. Disabling Colourise or Emissive shall hide that aspect's
interactive editor and timeline group while retaining its dormant authored
data. Double-clicking open
value-graph space shall add a key to the only or nearest visible curve, and
double-clicking an existing key shall open exact normalized-position editing.
Colourise Amount, the
palette/emissive centre-skew pair, and colour-stop Position/Amount shall show
their value on a draggable ranged bar whose double-click mode accepts an exact
numeric value. Colourise Phase may retain its specialised relative-turn rail
because its stored keys are deltas and its graph is accumulated. Colourise and
Emissive settings shall follow
the selected scalar field through a per-field visual memory unless the user
opts a feature into one global set. Each feature's colourise output shall
carry an authored Blend Mode against the colour beneath it - Normal (the
historical mix), Multiply, Screen, Add, Divide, and Vivid Light, the
vocabulary of the 2024 After Effects exhibition grade - applied identically
in the live viewport and offline renders with Colourise Amount acting as
the layer opacity, and slot order remaining the compositing order. Project
schema 85 introduces the per-edge fades, and schema 87 widens their existing
negative span-relative values without changing any schema-86 base value,
bounds snapshot, parameter key, or per-field memory; the remaining state
serializes additively with omitted defaults.

#### FR-STYLE-19
Each Water Feature Run shall support project-owned named variants that share
the run's feature membership, keys, and clips while storing sparse,
typed overrides only for detached non-keyable settings. Variants shall inherit
live Base Run values for every attached setting, retain the last detached
value across reattachment, and expose divergent state directly on the fixed
controls. Each animation shall independently persist whether each run is Off,
uses Base Run, or uses one variant; a linked pair shall expose independent A/B
choices with one active version per run per member. Preview and export shall
resolve the animation actually presented or rendered, apply fixed overrides
before shared keyed tracks, and freeze the effective state in render setups.
Project schema 86 and animation schema 26 introduce these records; missing or
stale selections safely preserve legacy enabled state or fall back to Base.

### 5.8 Procedural Motion
#### FR-MOTION-1
Dynamic point motion shall be procedural and non-destructive.

#### FR-MOTION-2
Motion shall be evaluated from rest position, time, and selected scalar-field mappings.

#### FR-MOTION-3
Motion controls shall include amplitude and frequency at minimum.

#### FR-MOTION-4
Amplitude and frequency shall each support constant and field-mapped control.

#### FR-MOTION-5
The motion system should support phase variation derived from point index, position hash, or scalar fields.

#### FR-MOTION-6
The motion system should support subtle vibration or wobble without requiring full geometry rebuild each frame.

### 5.9 AOV and Data Output
#### FR-AOV-1
The renderer shall output a beauty pass.

#### FR-AOV-2
The renderer shall output depth and alpha.

#### FR-AOV-3
The renderer should support layer ID and world position passes.

#### FR-AOV-4
The renderer should support motion vectors when feasible.

#### FR-AOV-5
The renderer shall allow selected scalar fields to be exported as additional passes.

#### FR-AOV-6
The renderer shall allow export of mapped style results such as final point size factor or final opacity factor when feasible.

### 5.10 Render Output
#### FR-RENDER-1
The renderer shall support interactive preview at approximately 1080p.

#### FR-RENDER-2
The renderer shall support offline rendering at 8K.

#### FR-RENDER-3
The renderer shall support tiled rendering for large outputs or constrained VRAM.

#### FR-RENDER-4
The renderer shall support image sequence output.

#### FR-RENDER-5
The renderer should support headless or command-line render invocation.

## 6. Non-Functional Requirements
### 6.1 Performance
#### NFR-PERF-1
The application shall prioritise interactive response on Apple Silicon M1 during look-development.

#### NFR-PERF-2
The application should maintain comfortable navigation on representative scene subsets at 1080p.

#### NFR-PERF-3
Parameter edits in the side panel should propagate quickly enough to feel interactive.

#### NFR-PERF-4
Offline rendering may be slower but shall preserve visual consistency with preview where practical.

### 6.2 Portability
#### NFR-PORT-1
The project shall build on macOS Apple Silicon.

#### NFR-PORT-2
The project should build on Windows for stronger render hardware.

#### NFR-PORT-3
Project files shall remain portable across supported platforms.

#### NFR-PORT-4
Project and animation documents shall store relocatable source-data,
authored-workspace, and machine-local render locations. A configured cloud
source root may hold the production 1 mm/5 mm and mesh PLY subset, and a
separate cloud folder may be the live authored workspace. Generated caches,
renders, render history, validation, and build output remain local. An exact
complete 2 mm ROCK/SAND/VEG bundle shall be used for a water-surface cache when
available; if it is absent, the complete 1 mm bundle shall be the explicit
fallback. A new computer with only the shared 1 mm/5 mm source subset shall
build and persist its 10 mm water-surface cache below its machine-local Saved
root. The configured authored workspace shall be authoritative while active.
Before replacing a project, Save shall compare its loaded ancestor, current
runtime serialization and current OneDrive JSON. Identifiable array entries and
object fields shall merge recursively; independent package/profile/Timing Take,
feature, setting and key edits shall combine automatically, while overlapping
field edits shall require an explicit local-or-OneDrive choice. Animation files
shall retain strict whole-file revision validation. Every target shall be
rechecked after staging, and a mid-save conflict shall preserve local recovery
JSON. **Save Project** and orderly close shall write the project together with
every changed animation as one non-deselectable save-all transaction. The save
dialog shall explain that Water packages, Feature Run clips/keys, and Timing
Takes are project-owned while camera paths and animation metadata live in the
listed animation files. Because OneDrive cannot enforce an offline distributed lock, both
machines shall allow cloud sync to settle before opening and after saving.

### 6.3 Reliability
#### NFR-REL-1
Missing scalar fields shall not crash the renderer.

#### NFR-REL-2
Corrupt or unsupported point attributes shall produce actionable errors.

#### NFR-REL-3
Render queue failures shall identify frame, pass, and shot context.

## 7. UI Requirements
### 7.1 General UI stack
- immediate-mode UI is acceptable and preferred for fast tooling development,
- the UI must not block camera navigation,
- panel state must be serializable per project.

### 7.2 Side panel sections
Recommended sections:
- Scene
- Layer
- Camera
- Shot
- Style
- Field Mapping
- Motion
- Preview Quality
- Render Output
- Output / AOV
- Presets

### 7.3 Parameter editor requirements
Each reusable parameter editor shall expose:
- source mode,
- constant value control,
- field selection,
- discovered field min/max display,
- editable input min/max,
- editable output min/max,
- clamp,
- invert,
- gamma or remap curve,
- reset to constant.

## 8. Data Model Requirements
### 8.1 Point-cloud attribute model
Each point should be able to expose:
- position,
- colour,
- point index,
- rest position,
- any imported scalar field,
- derived normalized fields when generated.

### 8.2 Render parameter binding model
A render parameter binding shall store:
- parameter name,
- source mode,
- constant value if used,
- field name or field index if used,
- input range,
- output range,
- remap options,
- enabled flag.

### 8.3 Style preset model
A style preset shall store:
- visual mode,
- parameter bindings,
- colour ramp references,
- X-ray and depth settings,
- motion settings,
- preview flags.

## 9. Acceptance Criteria
### AC-1
A CloudCompare-exported PLY with scalar fields can be loaded and the fields are visible in the UI.

### AC-2
A point-cloud parameter such as point size can be switched between Constant and Field-Mapped mode.

### AC-3
A user can map a field such as roughness from a source range to an output size range.

### AC-4
The side panel reveals on hover-near-edge and can be pinned or unpinned by double-click.

### AC-5
A user can create at least two shots and interpolate smoothly between them.

### AC-6
A user can render a shot sequence with beauty, depth, and at least one selected scalar-field pass.

### AC-7
A dynamic layer can apply subtle field-driven vibration without changing source data.

## 10. Risks
- 8K rendering may exceed practical M1 memory/performance without tiling.
- GS subsystem integration may constrain some custom shading paths.
- Very large point clouds may require careful paging, chunking, and point-budget design.
- Motion vectors for stylised point and splat rendering may be non-trivial.
- UI complexity may grow if the parameter-binding system becomes inconsistent.

## 11. Future Extensions
- native E57/LAS/LAZ import,
- node or graph editor for style logic,
- curve editors and histograms for field remapping,
- per-shot style overrides,
- render-farm orchestration,
- additional camera rigs,
- volumetric passes and fog.
