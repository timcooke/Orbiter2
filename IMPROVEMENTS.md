# Orbiter 2 — Improvement Roadmap

Tracked here so the plan survives context resets.
Forked from Orbiter @ v0.1-orbital-ring on 2026-04-18.
Last updated: 2026-04-18

---

## Status legend
- ✅ Done — committed to `main`
- 🔄 In progress
- ⬜ Not started

---

## Step 1 — Textured Earth globe ✅
Replace the plain navy `SphereNode` with an equirectangular texture map.

**Done:**
- Copied `earth_lights_lrg.jpg` (112×56 night-lights map) to `app/src/main/assets/`
- Load via `ImageTexture.getBitmap` → `Texture.Builder` → `materialLoader.createTextureInstance`
- Fully diffuse PBR material (roughness = 1, metallic = 0) for natural lighting
- `DisposableEffect` destroys the GPU texture on composable disposal
- Sphere resolution increased to 64 × 64 stacks/slices

**Future:** swap in a higher-res NASA Blue Marble 2048 × 1024 map — same code path,
just replace the `.jpg` in `assets/`.

---

## Step 2 — Sparky spacecraft model ✅
Replace the green satellite `SphereNode` with the real Sparky mesh.

**Done:**
- `tools/obj_to_glb.py` — pure Python OBJ → glTF 2.0 binary converter (no Node/Blender)
- Converted `sparkymatmesh.obj` + `sparkymat.mtl` → `app/src/main/assets/models/sparky.glb`
  - Auto-scaled: longest axis = 0.18 ER
  - Flat-shaded per-face normals (crisp low-poly look)
  - Two material groups: `sphere` (gold, metallic=0.8) + `body` (silver, metallic=0)
- `OrbiterSceneView.kt`: replaced `SphereNode satNode` with `ModelNode` via
  `modelLoader.createModelInstance("models/sparky.glb")`
- Each frame: applies position + attitude quaternion with ECI→scene axis swap:
  `Quaternion(x=qi, y=qk, z=-qj, w=q0)`

---

## Step 3 — Coordinate axes ✅
ECI reference-frame axes (large, fixed) + body-frame stubs attached to spacecraft.

**Done:**
- ECI axes: 3 × `CylinderNode` at Earth centre, length=1.5 ER, radius=0.006 ER,
  fixed world orientation (X=red, Y=green, Z=blue; Y is north pole)
- Body axes: 3 × `CylinderNode` children of `satNode`, length=0.25 ER, radius=0.004 ER;
  same colour convention; inherit spacecraft worldQuaternion automatically
- Cylinder default axis is local +Y; each rod rotated via `quaternion`:
  X rod: -90° about Z, Y rod: identity, Z rod: +90° about X
- Attached/detached via `DisposableEffect` for correct Compose lifecycle

---

## Step 4 — Orbital trail ✅
Fading cyan path showing the last ~500 positions (~1 full orbit at 30 Hz).

**Done:**
- `ArrayDeque<Float3>(TRAIL_MAX=500)` ring-buffer in `OrbiterSceneView`; pushed each frame
- `GeometryNode` with `Geometry.Builder(LINE_STRIP)` — updated every 3 frames to reduce GC
- Per-vertex `Float4` colour with alpha = 0 (tail) → 1 (head) for the fade effect
- `materialLoader.createColorInstance(cyan, alpha=0.8)` as base material

---

## Step 5 — Star-field skybox ✅
Replaces the black void with a procedural equirectangular star field.

**Done:**
- `tools/gen_starfield_hdr.py` — pure-Python generator; outputs 1024×512 Radiance RGBE
  HDR with 3500 faint stars + 120 bright soft-glow stars (blue-white, white, warm)
- `app/src/main/assets/environments/stars.hdr` — the generated asset
- `OrbiterSceneView`: `rememberEnvironmentLoader` + `rememberEnvironment` →
  `environmentLoader.createHDREnvironment("environments/stars.hdr")`
  The same HDR also drives the IBL (indirect light), improving Earth texture shading

---

## Mission Console UI ✅
Full redesign matching the Orbiter app design handoff (implemented 2026-04-18).

**Done:**
- **OrbiterEngine**: `HoldMode` enum (FREE/PROGRADE/RETROGRADE/NORMAL/ANTINORMAL/RADIAL/ANTIRADIAL);
  `latDeg`/`lonDeg` in `OrbiterState` (GAST-corrected); `applyBurn(dvMps)` direct velocity
  mutation; `jogRCS(dx,dy,dz,mag)` body→ECI Rodrigues rotation; `setAttitudeHold()` with
  Shepperd-method quaternion override each physics step
- **OrbiterViewModel**: `ControlUiState` + `SubsystemToggles`; DSN link/latency sin-wave
  simulation; `arm/fire`, `jogRCS`, `toggleSubsystem`, `resetSim` commands
- **OrbiterScreen**: 3-column mission-console layout — left (position/attitude/orbital elements),
  centre (3D SceneView + HUD), right (attitude hold 3×3 grid, ΔV planner with ARM pulse
  animation, 6DOF RCS pad, subsystems LED matrix, comms panel, status strip)
- **OrbiterSceneView**: HUD overlays — green LED "ECI FRAME" indicator, `LAT/LON` readout,
  RGB axis legend, ground-track text; `AxisLegendItem` composable
- Landscape-locked via `AndroidManifest` `screenOrientation="landscape"`
- Roboto Mono typography throughout console panels

---

## Step 6 — Simulation time warp ⬜
Speed up simulation without changing physics accuracy.

**Plan:**
- Add `setTimeWarp(factor: Int)` to `OrbiterEngine`:
  `stepSizeSec = baseStep * factor`
- Expose through `OrbiterViewModel.setTimeWarp()`
- Add ×1 / ×10 / ×100 / ×1000 toggle buttons to `ControlsPanel`
- At ×1000: 1 real second ≈ 17 sim-minutes → watch full orbit precess in ~5 s

---

## Step 7 — Additional thrust directions ⬜
Complete 3-axis manoeuvre capability.

**Plan:**
- The domain already supports all 6 `FTxyz` force components
- Add **NORMAL +/−** (FZ, out-of-plane) and **RADIAL +/−** (computed each frame
  from the unit position vector projected onto FX/FY) press-and-hold buttons
- Update `OrbiterViewModel` with `thrustNormal(Boolean)` / `thrustRadial(Boolean)`
- Lay out as a 3×2 button grid in `ControlsPanel`

---

## Step 8 — Camera modes ⬜
Three selectable views in the 3D panel.

| Mode | Description |
|------|-------------|
| **ECI** | Current orbit-manipulator view (fixed inertial frame, touch to orbit/zoom) |
| **Chase** | Camera offset along −velocity vector, spacecraft always centred |
| **Nadir** | Camera at 2× altitude directly below satellite, looking straight up |

**Plan:**
- Store `enum CameraMode { ECI, CHASE, NADIR }` in `OrbiterViewModel`
- In `OrbiterSceneView.onFrame`: update `cameraNode.worldPosition` and
  `cameraNode.worldRotation` when mode ≠ ECI
- Add a 3-way toggle chip row above the 3D view

---

## Step 9 — Orbit ellipse preview ⬜
Visualise the current orbital ellipse and how manoeuvres reshape it.

**Plan:**
- Compute the Keplerian ellipse analytically from `OrbiterState`
  (`semiMajorAxisEr`, `eccentricity`, `inclinationDeg`, `raanDeg`, `aopDeg`)
- Sample ~180 points around the ellipse in ECI, map to scene coordinates
- Render as a closed `GeometryNode` `LINE_STRIP` (dashed white/yellow)
- Regenerate only when orbital elements change by > threshold

---

## Step 10 — Ground track ⬜
2D map inset showing the satellite's path over the Earth's surface.

**Plan:**
- Compute geodetic latitude / longitude from ECI position + Earth rotation
  (sidereal time from `OrbiterState.time`)
- Small `Canvas`-based Compose composable: Mercator grid + ground-track polyline
- Inset in the HUD (e.g., bottom-left corner of the 3D panel or a tab)
- Draw future track for next N minutes using analytic Keplerian propagation

---

## Implementation notes

### Key files
| File | Purpose |
|------|---------|
| `app/src/main/kotlin/…/ui/OrbiterSceneView.kt` | 3D scene composable |
| `app/src/main/kotlin/…/ui/OrbiterScreen.kt` | HUD layout |
| `app/src/main/kotlin/…/ui/OrbiterViewModel.kt` | ViewModel bridging engine ↔ UI |
| `domain/…/OrbiterEngine.kt` | Physics engine + `OrbiterState` |
| `app/src/main/assets/` | Runtime assets (textures, models) |

### Coordinate convention
Simulation ECI (Earth Radii) → Filament Y-up right-handed:
- `sceneX = simX`
- `sceneY = simZ` (north pole = +Y)
- `sceneZ = −simY`

### Spacecraft attitude
`OrbiterState.(q0, qi, qj, qk)` is the inertial-to-body passive rotation quaternion.
Filament uses active rotation. Apply as:
```
node.worldQuaternion = Quaternion(x=qi, y=qk, z=-qj, w=q0)
```
(with the same ECI→scene axis swap applied to the imaginary components).
