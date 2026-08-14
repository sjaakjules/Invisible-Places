# Project Description — Field-Driven Point Cloud + Gaussian Splat Renderer

## Working Title
Field-Driven Point + Splat Renderer

## Summary
This project is a desktop renderer and shot-authoring tool for combining very large point clouds and Gaussian splats in one 3D scene. It is designed for exhibition image-sequence production, with fast look-development on Apple Silicon M1 and higher-end offline rendering on stronger Windows GPUs when required.

The renderer is not just a viewer. It is an authoring environment for stylised interpretation of point-cloud data.

It must let the user:
- load CloudCompare-exported point clouds as binary PLY,
- load Gaussian splat assets in the same world space,
- navigate the scene like CloudCompare,
- define camera shots and interpolate between them smoothly,
- render preview shots interactively at 1080p on M1,
- render final shots at 8K for downsampled 4K exhibition output,
- drive visual styling and subtle motion from static scalar fields authored in CloudCompare.

## Core Creative Goal
Treat point clouds as expressive visual media rather than neutral survey data.

Static scalar fields such as roughness, height, curvature, density proxies, classification, or custom derived metrics should be able to control how points look and move. The renderer should make it easy to turn these data fields into visual decisions.

Examples:
- map `roughness` to point size,
- map `height` to colour ramp,
- map `classification` to opacity bands,
- map `density` to emissive strength,
- map `curvature` to vibration amplitude,
- map `roughness` to vibration frequency.

## Core Technical Direction
- **Language:** C++
- **Graphics API:** Vulkan
- **macOS path:** MoltenVK on Apple Silicon M1
- **GS subsystem:** integrate `3dgs-vulkan-cpp` or a close derivative rather than flattening splats into plain points
- **Point cloud renderer:** custom Vulkan renderer with explicit support for scalar-field-driven styling
- **UI:** immediate-mode side panel tooling suitable for rapid iteration

## Primary Use Case
A user loads:
- a large point cloud exported from CloudCompare as binary PLY,
- one or more scalar fields already computed in CloudCompare,
- a Gaussian splat aligned into the same coordinate system,
- one or more saved camera shots,
- one or more style presets.

The user then:
- explores the scene interactively,
- edits visual styling through a side panel,
- switches any editable parameter between constant and field-driven control,
- saves camera poses and interpolated shot segments,
- previews results at 1080p,
- renders final 8K image sequences or tiled 8K output.

## Rendering Strategy
### Development / Look-Development
During design on M1:
- interactive preview at 1080p is sufficient,
- preview quality can be adaptive,
- motion and styling changes should feel immediate,
- decimation, point budgets, and lower-cost shader paths are acceptable.

### Final Output
For final exhibition output:
- render high-resolution stills or sequences at 8K,
- allow tiled rendering to manage VRAM limits,
- keep output deterministic,
- preserve project portability so the same scene can be batch rendered on stronger Windows GPUs.

The intended workflow is to render larger than delivery size, then downsample and finish in Houdini or Adobe postproduction tools.

## Input Constraints
### Version 1 input
- Binary PLY point clouds
- Scalar fields stored in or associated with the PLY
- Gaussian splat assets supported by the GS subsystem

### Not required in version 1
- native E57 import
- native LAS/LAZ import

CloudCompare remains the main preprocessing and scalar-field authoring tool for v1.

## Output Requirements
- Interactive preview viewport
- Offline still and sequence rendering
- Beauty pass
- Standard AOVs
- Optional scalar-field AOVs
- Optional mapped-style AOVs
- Project file containing shots, layers, styles, and render settings

For multi-computer authoring, that project file and its animation paths, named
render setups, style preset, and optional standalone water state can live in a
single cloud-synchronized authored workspace. Durable path roles are stored
independently of either computer's absolute folders: scene assets resolve below
the configured source-data root, authored files below the shared workspace,
and renders below the local output root. The production Scene3 1 mm/5 mm and
mesh PLY subset may use a separate OneDrive source folder; 2 mm/3 mm clouds,
gSplats, validation fixtures, generated caches, render history, renders, and
builds remain local. Water-surface preprocessing prefers a complete 2 mm
ROCK/SAND/VEG bundle when available, explicitly falls back to the synchronized
complete 1 mm bundle when 2 mm is absent, and reduces either source into the
same 10 mm cache. Consequently, a new computer can cold-build its own cache in
its local `Saved/` root from OneDrive's 1 mm/5 mm subset. The configured cloud
authored workspace is authoritative rather than the repository's fallback
`Saved/` copy. Existing project files use their loaded JSON as a semantic
three-way merge ancestor, automatically combining independent edits to named
libraries, Timing Takes, features, settings, keys and fields and asking for a
choice only where both computers changed the same value. Animation camera files
retain strict optimistic content revisions. All saves preserve a local recovery
copy when a peer update arrives during staging. Parallel offline work still
requires care because OneDrive is not a distributed locking service.

## Scene Layer Model
The scene supports multiple layers.

### 1. Static Point Cloud Layers
These contain millions to hundreds of millions of points and remain fixed in world space.

They support:
- scalar-field-driven appearance,
- point budgets,
- chunking or paging,
- culling,
- screen-space and world-space sizing,
- AOV export,
- optional subtle procedural motion evaluated from rest position.

### 2. Dynamic Point Cloud Layers
These are smaller or selectively animated subsets.

They support:
- vibration,
- wobble,
- drift,
- pulse,
- field-driven opacity/colour/size changes,
- motion evaluated procedurally from rest position rather than destructive edits.

### 3. Gaussian Splat Layers
These remain true Gaussian splats and render in the same camera and shot system as the point clouds.

### 4. Helper Layers
Optional overlays:
- grids,
- bounds,
- pivot markers,
- shot paths,
- camera frustums,
- target markers,
- debug field overlays.

## Navigation and Camera System
The application should feel familiar to a CloudCompare user while also supporting shot authoring.

### Navigation modes
- orbit
- pan
- dolly / zoom
- free-fly
- explicit look-at / target camera
- orbit around inferred surface target

The Camera panel exposes two orbit-control styles:
- **Current (World Up):** yaw/pitch navigation that keeps world Z vertical.
- **CloudCompare (Trackball):** an unconstrained virtual trackball that infers
  a surface pivot from the screen centre at the start of an orbit. Tangential
  drags near the trackball rim roll the camera, making it easy to redefine up.
  While orbiting, a large RGB wireframe pivot sphere and axes show the active
  rotation frame.

The selected orbit-control style is stored with the project and does not form
part of saved camera shots.

A persistent live-view row above Global Animation Position provides Top,
Front, Left, Right, Isometric, and Parallel controls. The four orthogonal
views use the documented project axes, Isometric is an elevated Front view,
and every preset rotates around the current focal point without changing its
camera distance. Parallel projection matches the perspective scale at that
focal plane so switching projection does not introduce a zoom jump.

### Surface-inferred targeting
The user should be able to orbit around what they are looking at.

Expected behaviour:
- cast a ray from screen centre or mouse position,
- find the nearest visible points,
- optionally average a small cluster of the closest valid hits for stability,
- use that position as orbit pivot,
- if no reliable point is found, fall back to current pivot, object centre, or scene centre,
- zoom should feel centred around the inferred target rather than blindly pushing camera forward.

### Shot system
The user should be able to save and interpolate camera shots.

Shot data includes:
- camera position,
- orientation quaternion,
- optional target point,
- orbit pivot point,
- FOV,
- clip planes,
- easing or interpolation settings,
- timing,
- optional per-shot style/render overrides.

Default frame rate: **30 fps**.

### Loop-transition smoothing
Two saved animations form one closed cycle after their perceived speed has
been authored. There is no order control: A-to-B-to-A and B-to-A-to-B are the
same cycle up to phase, and a canonical filename order makes the optimizer
deterministic regardless of which member is active. Loop smoothing jointly compares the signed
screen-space motion at both end-to-start seams and may move only each path's
first and last camera and focus positions, with separate bounds derived from
the adjacent terminal segment. It never changes total duration or per-key
segment frames.

The evaluated path retains its preserved original cubic spline and adds a
quadratic correction only on the first or last segment. The correction and its
velocity reach zero at the fixed adjacent key, so every evaluated frame from
key 2 through the second-from-last key remains unchanged. Reversible metadata
stores both original endpoint poses. Applying and unapplying first create
paired `_Edited` versions; Save Changes promotes both animation files and the
active project atomically, while Discard Edits restores both saved versions.
A persistent Loop Validation readout reports the normalized screen-flow
mismatch before and after smoothing for both directions, endpoint movement and
movement-cap use, and terminal perceived-speed deviation. **Validate Loop
Metrics** is read-only; after restart it reconstructs the before state from the
preserved endpoint metadata rather than changing either animation. Rejected
optimizer candidates retain the same diagnostics and an explicit rejection
reason, so a no-op is distinguishable from a very small accepted adjustment.
A linked camera can participate
only when all of its uses are movable endpoints in the selected pair.

The same pair can be compiled into a separate linked animation. Its circular
timeline draws both source bands with translucent colour, making overlap
visibly darker and any endpoint hold visibly empty. At zero padding, the
incoming first key aligns with the outgoing penultimate key at both joins, so
the complete terminal edge crossfades. Signed padding offsets that anchor:
negative values begin the blend earlier, while positive values delay it and
create a hold only once the remaining terminal edge is exhausted. A stable key in the first
source defines the loop phase, so playback can begin in the middle of A, run
through B, wrap through the beginning of A, and finish on exactly that key.
The viewport can ghost both source splines and paired samples across each
complete overlap. Its **Live Camera** control defaults to **Overlay** and can
isolate A or B; the former blended-camera preview has been removed. Overlay
renders A and B on alternating display frames, retains each source's newest
colour and depth image, and composites them with the linked weights. While one
source renders, cached depth and the known current camera pose reproject the
other source toward its present position. This reduces half-rate camera
judder, with conservative fallback around sparse depth and newly revealed
edges. It therefore keeps one frustum and one point-scene pass per display
frame instead of drawing both point sets at once; full-screen
copy/reprojection/composite bandwidth plus history memory remain.
The linked file remains an
ordinary renderable per-frame path, while schema-16 source, padding, and
start-key metadata permits a deterministic rebuild after either saved or
`_Edited` source changes.

The **Blend Partner** choice is saved with each `.ipanim.json` independently
of reciprocal velocity-link metadata. Reloading an unlinked animation restores
that preferred working partner, and flipping between A and B resolves the
preference from the animation that is actually loaded instead of retaining a
stale row assignment. A reciprocal link remains authoritative while present;
unlinking preserves the former partner as the working preference.

After choosing a Blend Partner and setting the timeline bounds, the timeline
can designate one key with a matching frame inside the start/end overlap;
velocity alignment does not need to be applied first. **Match Camera Rig To
Partner** keeps that key's focus fixed and moves only its camera position. It
expresses the partner frame's focus-to-camera offset as signed along-path,
lateral, and world-height components, then rebuilds the same rig in the
selected animation's local path frame. This preserves focus distance and
whether the view sits forward, perpendicular, or backward relative to travel
without moving the partner. Keys outside the overlap have no counterpart and
cannot run this alignment.

The paired matching-frame timelines also provide a camera-alignment clipboard.
Right-clicking an ordinary key selects its stable animation/key ID and draws a
red outline, so the selection stays attached to the same physical key when A
and B are flipped. A comparison table continuously reports the active scrub
frame, selected node, and copied snapshot: geometric camera-to-focus distance,
polar angle from world +Z, signed horizontal angle from the evaluated local
path tangent, camera and focus height above the authored ground reference, and
the shared ground sample Z. The ground reference starts at the horizontal
projection of the point one-third of the way from focus to camera, using at
most a small nearest-cell tolerance to bridge a sampling hole. It reads the authored SAND
height already retained in the 10 mm cache, regardless of whether that cache's
selected SAND source is the preferred analysis spacing or a complete-bundle
fallback such as 5 mm. MESH/MESHSampled Ground is eligible only for a retained
cell explicitly associated with vegetation, never as general sand or rock
terrain. When the one-third point misses, the lookup advances toward the camera
and takes the first usable SAND or vegetation-supported MESH cell. A
selected-node note names that source, reports the fallback percentage, or
explains why ground is unavailable. **Copy Camera
Alignment** freezes all those values and the
evaluated orientation by value, so later source-node edits cannot change the
clipboard. Paste checkboxes independently select distance, polar angle,
camera height, focus height, tangent-relative angle, and horizon/roll.
Target-driven paths remain target-driven; focus-height paste translates the
rig vertically, camera-height paste then sets its clearance exactly, and paths
that already author orientation can receive the copied horizon/roll.

**Fix A+B Lens** sits beside **Extend Both Seams…**. It writes paired edited
paths with one median authored FOV, focus-distance policy/value, and aperture
policy/value per animation, uses the smallest existing near plane and largest
existing far plane across the pair, and clears lens-only endpoint tangents.
Camera/focus paths and key timing are untouched. When this repair alone is
needed for extension, the assistant applies it only to its private baselines;
Cancel discards it and Apply Both commits it with the generated ends.

For paired slow pans that need more material beneath both wipes,
**Extend Both Seams…** opens a guided Matching-Frame assistant while A remains
the active animation. No velocity alignment has to exist first: A-start against
B-end and B-start against A-end are treated as the two already-authored 50%
blend poses. For each seam the user marks an ordered source triangle—the main
feature, a ground/front point towards the camera, and a perpendicular side point
along or opposite the pan. At the destination end, one matching feature-centre
click generates the other two nodes with the same physical offsets rotated into
that camera's local frame. Every generated node remains editable or
re-raycastable. The user then scrubs inward from the source start to choose one
half-span around that visual midpoint. The same triangle correspondence creates
motion in both directions: a pre-roll before the source's old frame zero and a
tail after the partner's old end.

The fit first makes a localized correction to each existing destination
terminal so its triangle overlays the opposite source-start triangle. Because
the two 50% poses are never perfectly aligned, that correction used to land
entirely on the final key segment and could read as a sudden movement. Each
span stage therefore sets an **Alignment ramp** — following the chosen seam
span until edited — that eases the endpoint move across the authored keys
inside its window as fractional localized corrections with matching tangents,
turning the lurch into a slow drift while the base spline, every frame count,
and the path outside the window stay exact (a zero ramp reproduces the old
behaviour). An **alignment share** additionally chooses who moves: the legacy
destination-only move, **Meet halfway** (default — both 50% cameras solve
toward a blended screen-space triangle target so each side absorbs half the
transient), or source-only. The fit then tracks the source feature's signed
pixel translation, in-plane rotation, and perspective scale through the chosen
inward span. It copies that motion forward into the partner tail and backward
into the source pre-roll. A simple generated end uses two unlinked keys; a
crossed authored key or screen-motion inflection uses three. Each animation
therefore grows at both ends while old segment durations and bulk pan speed
remain unchanged. Interior authored motion outside the alignment window is
kept exactly.

After each seam is captured, a synchronized scrub review flips between the
generated source pre-roll and privately fitted destination tail. Negative
offsets show the pre-roll, offset zero shows the adjusted original 50% endpoint
pair, and positive offsets show the tail. The review reports each side's
endpoint move, how many keys carry eased corrections, and the peak drift
speed the alignment adds — the number a visible bump corresponds to. A
ping-pong transport (Space) plays back and forth across the seam window, with
single-frame steppers and a jump-to-seam button beside the slider. Tab flips
the full A/B frame at the matched pose; while orbit-inspecting, Tab instead
moves the free camera by the rigid transform between the two matched seam
cameras, so the vantage holds still and the scene itself appears to swap —
residual misalignment shows directly, with both seam cameras drawn as
wireframe frusta. Green review keys can be dragged in the viewport (camera or
focus target), and moving a 50% midpoint carries its partner through the
captured triangles. This lets the user re-pick the destination feature and rebuild
that seam before moving to the opposite endpoint. The review can also composite
the two camera renders as an A/B hard split. Its adjustable boundary follows
the mean projected anchor position throughout the synchronized scrub, while a
stored pixel offset lets the user place the wipe beside the feature without
breaking that tracking. Both triangles remain visible, A/B can swap sides, and
the active view remains editable. Two key timelines expose the generated head,
generated tail, paired 50% controls, and nearby authored controls during this
review. Green controls can run a private one-seam smoothing pass; the midpoint
pair stays triangle-constrained, all key timing remains fixed, and the authored
selection is replayed by the final joint smoother. Water effects are temporarily suppressed to
keep this comparison responsive.

A Preview can isolate A, B, or their overlay, inspect the extended timeline
bands and residuals, and compare baseline/candidate poses. Nothing enters the
animation registry until **Apply Both** creates paired `_Edited` paths and the
reciprocal blend metadata. Before Apply, an optional transition-smoothing view
shows every A/B key and lets the user choose which controls may absorb an
abrupt alignment bend. The generated keys, both feature-aligned midpoint pairs,
and up to two authored keys inward from each end are selected by default. Each
midpoint's A/B cameras move together through the triangle correspondence rather
than remaining fixed; a hard three-node projection check preserves the feature
alignment. Green controls can be moved directly with XYZ values or the viewport
gizmo, and each edit becomes the starting point for the next pass. The displayed
camera/focus splines are resampled per segment with exact key knots after each
edit. If iterative edits leave projected drift, **Force-align triangles** keeps
the selected A or B path fixed and re-solves the paired midpoint camera/focus
controls against all three captured nodes. Separate
spatial-only actions target X velocity, Y velocity, image rotation, X plus
rotation, or perceived speed under all three speed methods; amber controls and
every segment-frame weight remain unchanged. A/B motion graphs mark both exact
50% poses. The final hard-split preview has a signed -1..1 full-cycle transport,
two feature-relative split offsets, repeated-drag wrapping, and Space-key loop
playback. **Fit final A/B cycle to 4:00** is enabled by default: Preview slows
or speeds the transport by the proposed shared factor, and Apply apportions the
two authored bulk spans plus all four generated seam halves onto an exact
7,200-frame unique cycle. Reciprocal blend durations and finite export bounds
scale with the same timing; camera/focus/lens geometry, key order, and triangle
alignment remain unchanged. Reset restores the immutable
triangle fit, and a rejected pass does not replace the current working edit.
Back, Cancel, and Escape discard all private candidates and restore the launch
camera, playhead, pivot, and matching ghost.

Once the reciprocal metadata is applied, **Seamed View** keeps the same
two-camera hard-split compositor available during ordinary linked-animation
scrubbing and playback. It resolves the partner from the link rather than the
current Blend Partner menu, maps both saved overlap bands at exact elapsed
seconds, and follows captured feature anchors when the pair has just been
applied in this session; reloaded links fall back to a deterministic sweep
from saved overlap progress. The
ending path is always left and the starting path right; switching which file
is loaded preserves the physical seam and its artistic pixel offset. Outside
an overlap, live view returns to the ordinary current animation.

The linked-view controls live above Global Animation Position. Both reciprocal
members can share one Timing Take over the complete unique loop while retaining
their own local camera timelines. Each animation stores a signed window into
that loop; ordinary A-only or B-only rendering maps its local frame into the
shared cycle. Evaluators expose virtual previous/next-cycle neighbours, rather
than adding artificial endpoint keys, so Smooth Step and manual spline motion
do not decelerate at a path boundary and Rain can be authored continuously over
the whole loop. Global Animation Position remains a camera control only;
procedural waves, Rain, and trails keep advancing independently.

Camera duration grows on the same 30 fps timebase. Every normalized timing,
water, and visual-effect key or finite activation boundary is shifted by the
prepended frame count and rescaled to the new duration, so it remains attached
to the same old camera pose. Full-range start/end boundaries remain animation
sentinels. Generated ends hold their nearest keyed values, but procedural
waves, rain, trails, and other motion use a separate effect clock rather than
the camera-path position. They keep advancing from steady elapsed time in live
view even while the camera is paused; offline renders instead sample them from
the full deterministic output frame/subframe time. Explicit export start/end
frames shift by the prepended count to retain the same old camera content; an
end frame of zero still means the complete, now-extended animation. The four reciprocal overlap extents and project link
mirror update with the paths, and A, B, and the project retain one atomic
save/discard dependency. Saving that pending dependency under new names is a
linked-pair Save As operation: both new animation files and the project are
committed together, their partner filenames are retargeted to one another, and
the calculated start/end blend durations are retained. The original files are
not overwritten.

### Animation versioning
Every registered animation keeps its last saved, disk-backed version plus at
most one explicit in-memory `_Edited` version. Authoring, linked-camera edits,
perceived-speed equalization, and loop smoothing target `_Edited`; normal load
prefers it. The Animation list exposes Saved and `_Edited` separately for
comparison, with the Saved view read-only while edits exist. Saving promotes
`_Edited` over Saved and removes the shadow. Discarding removes only the
shadow and reloads Saved.

Perceived-speed equalization and manual **Segment Frames** edits write the
incoming frame count on each key. **Equalize X Velocity** and **Equalize Y
Velocity** are deliberately spatial instead: they keep every frame weight
unchanged, integrate the absolute middle focus-plane velocity shown by the
corresponding Scene Speed graph, and move each interior camera and focus key
together to sampled positions on their existing curves. The first and last
poses remain fixed. This gives a pan roughly constant screen speed on the
chosen axis without time warping and preserves signed travel direction, so an
authored reversal still passes through zero.

**Minimize Rotation** also leaves frame weights and endpoints unchanged, but
keeps every focus control fixed and slides only interior camera controls along
the existing evaluated camera curve. Its bounded optimization reduces the
magenta rotation curve's RMS magnitude, local roughness, and energy in the
minority direction, explicitly discouraging new direction reversals.
**Equalize X + Rotation** uses the same camera-only movement while strongly
penalizing deviation from the animation's signed mean X velocity. Both are
best-effort operations: fixed focus controls, endpoints, and the available
camera curve can make a perfectly constant or one-directional result
impossible. They optimize the same 3x3 focus-plane flow proxy drawn by the UI;
they do not render scene pixels or account for depth-dependent parallax.
Spatial camera, focus, lens, and orientation splines use
independent cumulative motion-distance knots, so ordinary Segment Frames
changes traversal without bending the curves. A positive monotone C2 time map
carries non-zero velocity through each non-degenerate key; the composed
position and orientation retain continuous velocity and acceleration.
Endpoints use one-sided forward chord tangents rather than being forced to
start or finish at zero speed.

**Add Key at Playhead** splits the existing spatial parameter span and
materializes its endpoint tangents, making insertion of the evaluated pose a
spatial no-op. Manual pose edits and topology changes rebuild distance spans
from the resulting key order. **Reset Timing Weights** shares the full duration
evenly across all segments without moving any key. It also bakes any localized
alignment correction into a clean C2 spline through the adjusted poses, so
stale correction weighting cannot survive later structural or timing edits.
At the bottom of **Keys**, **Set Selected Key from Current Camera** replaces the
selected camera pose with the live viewport pose. Its focus ray uses the first
stable point-cloud surface hit; when no surface is hit, the key keeps its
previous camera-to-focus distance along the new view direction.

Selecting a focus control in the live viewport also shows a blue world-Z
rotation handle: a quarter arc from +Y to +X on the focus point's XY plane.
Dragging it orbits that key's camera position around the focus without changing
the camera's Z offset or focus distance; authored free orientation turns with
the camera. Selecting either a camera or focus control draws a thin line
between them and a small cube one-third of the way from focus to camera. The
line is clipped at the live-view boundary, so its visible portion remains drawn
when either endpoint is off screen, including a crossing whose two endpoints
are both outside. The cube carries the standard X/Y/Z axes, XY/ZX/ZY planes,
and centre view-plane drag; all translate the complete key rig by one equal
delta while the ordinary node gizmo remains local to the selected camera or
focus control. This replaces
the detached secondary plane badges and preserves distance, view direction,
authored orientation, and lens values. The same controls work on transformed
partner nodes shown by matching-frame editing. The focus control retains its
world-Z orbit handle. Double-clicking a focus control opens **Live Edit Mode** at that
exact key frame with its evaluated pose, orientation, FOV, clip planes, focus,
and aperture. Orbit, pan, dolly, and preset-view changes continuously preview
back onto that key. A faded red live-view border and a top-right
**Live Edit Mode** card expose tick/Enter to commit and cross/Escape to restore
the exact pre-edit animation and view. Other authoring controls remain locked
for the transaction.

The camera-view toolbar at the top of the controls window includes a direct
**Grid** toggle for a preview-only composition guide. It draws horizontal and
vertical thirds together with a visually stronger halfway line, so the 1/3,
1/2, and 2/3 marks remain distinct rather than forming a uniform grid. The
adjacent settings button can show halfway or thirds independently and adjusts
the guide colour, opacity, and line weight. These ImGui guide lines appear only
over the live view and are never included in frame previews or exports.

Each animation also stores a default live-view window size. Loading that
animation resizes the live view to its authored dimensions. The Project panel
stores its own window size and a **Lock Window Size** option; when locked, the
project dimensions override every animation default and remain enforced.
Legacy animations without a stored size keep the current window unchanged and
capture it the next time they are saved.

### Timing scalar effects
The Timings tab owns one ordered list of scalar-driven effects. The user can
add either a **Colourise** effect or an **Emissive** effect, reorder them by
dragging, enable or disable each effect, and trim its inclusive active range on
the animation overview.

- Colourise effects retain their palette, phase, colourise-amount, scalar
  field, bounds, fade, and smooth key tracks.
- Emissive effects have no palette UI. They apply a non-negative, smoothly
  keyable emissive level through a scalar-field bounds/fade mask; bounds and
  fade use the same keyable histogram controls as Colourise effects.
- Keys remain authorable outside an effect's active range. Rendering samples
  the same global curves at the range boundaries, so enabling an effect does
  not create a discontinuity or a separately editable shadow key.
- Colourise and Emissive effects share the same overlap budget. Five or fewer
  simultaneous effects is the responsiveness recommendation; the UI marks
  ranges that exceed it, while the renderer safely supports up to eight
  concurrent effects and gives higher list entries priority if more overlap.
- Both effect kinds, their activation ranges, and their keys are stored with
  the project and in render-setup snapshots. Emissive effects require a scalar
  field and cannot select synthetic normal components.

Water setting graphs and Timings visual-feature lanes advance the same global
animation position, so scrubbing either one continues to replay every active
keyed water and visual effect. Their camera follow is conditional: it continues
when the live view still matches the previous animation frame, but orbiting,
panning, or dollying away turns the viewport into a retained inspection view.
The Global Animation Position bar and camera-key timeline always restore and
follow the animation camera. A shared **Always Follow Camera** override appears
beside **Subtle Selected Cues** in Water and beside **Split Graphs** in Timings.
Starting ordinary animation playback with **Play** or Space follows the camera
only when the live view matches the current animation frame. If the user has
already orbited, panned, or dollied away—or navigates away during playback—the
global playhead and keyed effects continue while that inspection camera stays
put for the rest of the run. Scrubbing the global or camera-key timeline still
restores the animation camera before a later playback run.

Water setting keys offer six outgoing interpolation modes. In addition to the
automatic Smooth Step, Linear, Hold, Monotone Spline, and Centripetal
Catmull-Rom choices, **Spline Handles** exposes persisted cubic-Bezier controls
for manual time/value shaping. Selecting a key shows only the incoming and
outgoing controls used by its adjacent manual segments; dragging them reshapes
the curve without moving the key. Time-only key markers sit on a separate lower
rail beneath the value graph, so a minimum or zero value cannot overlap the
retime target. New setting tracks default their inherited keys to Monotone
Spline; existing serialized and legacy-migrated tracks preserve their previous
style. Project schema 75 and water-sources schema 29 store the handles.

The Timings tab adds a Clips lane above the run graphs, with the same lane
embedded on each water sub-tab timeline. Every feature row shows its keyed
spans as blocks whose fill brightness follows the feature's primary keyed
setting, so a rain burst or seepage response reads at a glance without curve
clutter; a feature that has keys but no clips yet appears as a dashed
loose-keys block that one double-click turns into a named clip. Loose keys can
also remain beside stored clips. Selecting exactly one stored clip makes new
keys for that feature join it; with no such selection, new keys remain loose.
Clip members use square curve dots and diamond time markers, while loose keys
remain round and linear. The first and last explicitly owned keys set each
keyed clip's bounds. Blocks drag to
offset their grouped keys in time, stretch from either edge (a multi-selection
scales about its far edge), duplicate with Alt-drag, and move or copy onto
another feature of the same kind by dragging between rows. Marquee drags and
Cmd-clicks build multi-selections that offset and scale together. Explicit key
ownership lets clips overlap and pass across one another; the normalized
domain and exact same-track key-time collisions remain guarded. Escape cancels
a live drag losslessly. Clicking a clip also selects all of its active owned
key nodes. Plain key clicks replace that node selection, Cmd/Ctrl-click toggles
individual nodes, Shift-click selects a same-setting range, and the focused
settings graph or Clips lane owns Cmd/Ctrl+A, Delete/Backspace, and Escape.
The Global Animation Position bar never receives those editing commands. One
bulk interpolation control writes the same outgoing mode to every selected key
and is preserved when the clip is captured or applied as a package.
Right-click saves a clip as a named package that records its natural length,
and the same menu applies any saved package to a same-kind feature at the
clicked position, after which each applied clip slides, shrinks, or stretches
independently — one authored seepage on/off envelope can drive many seepage
features, each with its own timing against the rain. Clips persist with the
run (introduced in project schema 74/water-sources schema 28; explicit key
ownership in project schema 76/water-sources schema 30), retime with camera-tail
extensions, and never change how the keys themselves evaluate.

Water key authoring retains one atomic edit in session history. Ctrl+Z on
Windows/Linux and Cmd+Z on macOS restores the state before the latest key,
keyed-setting, or settings-clip edit; the same shortcut then reapplies it. Each
continuous slider, key, spline-handle, or clip drag is one edit, while focused
text and numeric fields retain their own native text undo.

## Side Panel UI
The main artist-facing control surface is a side panel.

### Panel behaviour
- hidden by default at screen edge,
- reveals when the mouse approaches the edge,
- remains visible while hovered,
- double-click pins it open,
- double-click again unpins it,
- optional hotkey toggles visibility,
- pinned state is saved per project.

Water feature sliders retain their visual slider interaction but switch to an
exact numeric field when the slider bar is double-clicked. An exact `0..1`
range displays three decimals; that presentation does not quantize the stored
float or the values used by Water key interpolation. For keyable controls,
double-clicking the setting name remains the separate keying toggle.

### Panel purpose
The side panel controls:
- scene and layer visibility,
- camera settings,
- shot management,
- point-cloud visual styling,
- scalar-field bindings,
- motion settings,
- preview quality,
- render settings,
- AOV selection,
- preset save/load.

## Parameter Binding Model
Every editable visual parameter should support at least two source modes.

### 1. Constant mode
A fixed user-entered value.

Examples:
- point size = 2.5 px
- opacity = 0.7
- vibration amplitude = 0.03

### 2. Field-mapped mode
A scalar field from the point cloud drives the parameter.

The user can set:
- field source,
- input minimum,
- input maximum,
- output minimum,
- output maximum,
- clamp,
- invert,
- gamma or curve shaping,
- optional blend with a constant.

Example:
- parameter: point size
- source field: `roughness`
- input range: `-10.0 .. 125.0`
- output range: `0.5 .. 6.0 px`
- clamp: on
- invert: off

This allows the smallest roughness values to appear as small dots and the highest roughness values to appear as large dots.

## Parameters That Must Be Mappable
At minimum, point-cloud parameters should support constant and field-mapped control for:
- point size,
- opacity,
- colour ramp position,
- emissive strength,
- X-ray strength,
- depth fade,
- jitter amplitude,
- vibration amplitude,
- vibration frequency,
- phase offset,
- additive blend weight,
- world-space scale multiplier,
- screen-space scale multiplier.

## Visual Styles
Expected visual looks include:
- solid dots,
- circular discs,
- small-to-large dots by field,
- colour-by-field,
- opacity-by-field,
- X-ray / semi-transparent look,
- additive glow / emissive accents,
- high-contrast silhouette modes,
- monochrome with selective highlights,
- subtle procedural motion,
- depth-based attenuation or fog.

The style system should be extensible rather than hard-coded to a single look.

## Procedural Motion
Motion should be lightweight, deterministic, and evaluated from rest position.

Examples:
- points vibrate more when roughness is high,
- points pulse faster when density is high,
- points wobble within a bounded radius derived from a field,
- points offset along pseudo-random directions seeded by point ID.

The goal is subtle motion that preserves the point-cloud character and remains affordable during look-development.

## AOV / Data Output Strategy
The renderer should output standard compositing passes and optional data-rich passes.

### Standard AOVs
- beauty,
- alpha,
- depth,
- layer or object ID,
- world position when feasible,
- motion vectors when feasible.

### Point-cloud data passes
The user should be able to export selected scalar fields and selected mapped results, such as:
- roughness,
- height,
- classification,
- density,
- mapped point size,
- mapped opacity,
- mapped emissive strength,
- mapped motion amplitude.

These outputs allow Houdini or Adobe post workflows to retain access to point-cloud data after rendering.

## Performance Philosophy
### On M1
Prioritise:
- responsiveness,
- fast style iteration,
- smooth navigation,
- practical 1080p preview.

### On stronger Windows GPUs
Prioritise:
- final quality,
- higher point budgets,
- full AOV output,
- high-resolution sequence rendering,
- batch or command-line render support.

## Success Criteria
The project is successful when the user can:
- load a CloudCompare-exported PLY with scalar fields,
- load a Gaussian splat in the same space,
- navigate comfortably,
- edit styles from a side panel,
- switch parameters between constant and field-mapped control,
- save and interpolate shots smoothly,
- preview on M1,
- render export-quality sequences for exhibition postproduction.

## Version 1 Out of Scope
- full DCC parity with Blender/Houdini/Unreal,
- native E57/LAS/LAZ pipeline,
- node graph material editor,
- physics simulation,
- deep Adobe or Houdini integration beyond exported passes and sequences,
- farm orchestration beyond command-line batch render.
