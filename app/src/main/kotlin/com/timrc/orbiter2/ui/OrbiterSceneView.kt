package com.timrc.orbiter2.ui

import androidx.compose.foundation.background
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.DisposableEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.rememberUpdatedState
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.google.android.filament.IndirectLight
import com.google.android.filament.RenderableManager
import com.google.android.filament.Skybox
import com.timrc.orbiter2.domain.OrbiterState
import dev.romainguy.kotlin.math.Float3
import dev.romainguy.kotlin.math.Float4
import dev.romainguy.kotlin.math.Quaternion
import dev.romainguy.kotlin.math.cross
import dev.romainguy.kotlin.math.normalize
import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt
import io.github.sceneview.Scene
import io.github.sceneview.environment.Environment
import io.github.sceneview.geometries.Geometry
import io.github.sceneview.node.CameraNode
import io.github.sceneview.node.CylinderNode
import io.github.sceneview.node.GeometryNode
import io.github.sceneview.node.ModelNode
import io.github.sceneview.node.SphereNode
import io.github.sceneview.rememberCameraManipulator
import io.github.sceneview.rememberCameraNode
import io.github.sceneview.rememberEngine
import io.github.sceneview.rememberEnvironment
import io.github.sceneview.rememberEnvironmentLoader
import io.github.sceneview.rememberMaterialLoader
import io.github.sceneview.rememberModelLoader
import io.github.sceneview.rememberNode
import io.github.sceneview.texture.ImageTexture

// ── Camera modes ─────────────────────────────────────────────────────────────
enum class CameraMode { ECI, CHASE }

/**
 * Compute a world-space quaternion for a Filament camera at [eye] looking toward [target].
 *
 * Filament camera convention: local −Z = forward, +Y = up, +X = right.
 * The rotation matrix columns are the local axes expressed in world space:
 *   col 0 = camera_right (+X), col 1 = camera_up (+Y), col 2 = camera_back (+Z = −fwd).
 * We then convert to quaternion via the Shepperd method.
 */
private fun cameraLookAt(eye: Float3, target: Float3): Quaternion {
    val fwd = normalize(target - eye)
    // Fallback up when fwd is nearly vertical (avoids degenerate cross-product)
    val worldUp = if (fwd.y > 0.999f || fwd.y < -0.999f) Float3(0f, 0f, if (fwd.y > 0f) -1f else 1f)
                  else Float3(0f, 1f, 0f)
    val back  = -fwd                                       // camera local +Z in world
    val right = normalize(cross(worldUp, back))            // camera local +X in world  — cross(up, +Z) = +X
    val up    = cross(back, right)                         // camera local +Y in world
    // Column-major rotation matrix  (columns = local axes in world)
    val m00 = right.x; val m01 = up.x; val m02 = back.x
    val m10 = right.y; val m11 = up.y; val m12 = back.y
    val m20 = right.z; val m21 = up.z; val m22 = back.z
    // Shepperd quaternion extraction
    val trace = m00 + m11 + m22
    return if (trace > 0f) {
        val s = 0.5f / sqrt(trace + 1f)
        Quaternion(w = 0.25f / s, x = (m21 - m12) * s, y = (m02 - m20) * s, z = (m10 - m01) * s)
    } else if (m00 > m11 && m00 > m22) {
        val s = 2f * sqrt(1f + m00 - m11 - m22)
        Quaternion(w = (m21 - m12) / s, x = 0.25f * s,  y = (m01 + m10) / s, z = (m02 + m20) / s)
    } else if (m11 > m22) {
        val s = 2f * sqrt(1f + m11 - m00 - m22)
        Quaternion(w = (m02 - m20) / s, x = (m01 + m10) / s, y = 0.25f * s,  z = (m12 + m21) / s)
    } else {
        val s = 2f * sqrt(1f + m22 - m00 - m11)
        Quaternion(w = (m10 - m01) / s, x = (m02 + m20) / s, y = (m12 + m21) / s, z = 0.25f * s)
    }
}

// ── Orbital ring constants ────────────────────────────────────────────────────
/** Number of sphere dots placed evenly around the orbital ellipse (1° spacing). */
private const val RING_DOT_COUNT = 360

/** Radius of each ring dot sphere, in Earth radii (50 km / 6371 km ≈ 0.00785 ER). */
private const val RING_DOT_RADIUS = 0.00785f

// ── Chase camera ─────────────────────────────────────────────────────────────
/**
 * Scales the CameraManipulator's eye-position down to a comfortable
 * spacecraft-viewing distance.  Default manipulator eye ≈ (0, 1, 4.5) ER,
 * magnitude ≈ 4.61 ER → scaled to ≈ 0.46 ER from the spacecraft.
 * Pinch-to-zoom changes the manipulator magnitude proportionally.
 */
private const val CHASE_ECEF_SCALE = 0.1f

// ── Model-to-body-frame alignment ────────────────────────────────────────────
/**
 * Pre-rotation applied to the Sparky mesh so that the model's visual axes
 * align with the simulation body frame:
 *   old model Z  →  body X   (nose / forward)
 *   old model X  →  body Y
 *   old model Y  →  body Z   (up)
 *
 * Two-step composition:
 *  1. 120° about (1,1,1)/√3  → old Z→body X, old Y→body Z, old X→body Y
 *     q1 = (w=0.5, x=0.5, y=0.5, z=0.5)
 *  2. 180° about model old-Z (= body X) → flips body Y and body Z
 *     q2 = (w=0, x=0, y=0, z=1)
 *  Combined: q = q1 × q2 = (w=−0.5, x=0.5, y=−0.5, z=0.5)
 *
 * Compose as:  satNode.worldQuat = attitudeQuat * MODEL_BODY_OFFSET
 * (offset applied first in model-local space, then world attitude on top).
 *
 * Body-axis stub quaternions are rederived from the inverse of MODEL_BODY_OFFSET
 * so each stub still points along the true simulation body axis:
 *   X stub: +90° about +X  (unchanged)
 *   Y stub: +90° about +Z  (was −90°)
 *   Z stub: 180° about +X  (was identity)
 */
private val MODEL_BODY_OFFSET = Quaternion(x = 0.5f, y = -0.5f, z = 0.5f, w = -0.5f)

// ── Orbital trail constants ───────────────────────────────────────────────────
/** Maximum positions in the trail ring-buffer (~1 full LEO orbit at default 10 s/step). */
private const val TRAIL_MAX = 500

/**
 * Positions over which the ribbon fades head→black (= ~1/4 of a 90-min LEO orbit).
 * 135 steps × 10 sim-s/step = 1 350 sim-seconds ≈ T/4.
 * Halve to 68 for T/8; keep TRAIL_MAX ≥ TRAIL_FADE.
 */
private const val TRAIL_FADE = 135

/** Half-width of the orbital-trail ribbon, in Earth radii (~100 km). */
private const val TRAIL_HALF_WIDTH = 0.015f

/**
 * Compute [RING_DOT_COUNT] evenly-spaced ECI positions along the Keplerian ellipse,
 * mapped to scene space (sceneX=eciX, sceneY=eciZ, sceneZ=-eciY).
 *
 * Each position is the centre of a small SphereNode, so the ring is visible from
 * any camera angle — no ribbon/normal issues.
 *
 * @param a       Semi-major axis, Earth radii
 * @param e       Eccentricity
 * @param iRad    Inclination, radians
 * @param raanRad RAAN (Ω), radians
 * @param aopRad  Argument of periapsis (ω), radians
 */
private fun computeOrbitalRingPositions(
    a: Float, e: Float,
    iRad: Float, raanRad: Float, aopRad: Float
): List<Float3> {
    if (a < 0.1f) return List(RING_DOT_COUNT) { Float3(0f, 2f, 0f) }

    // Perifocal → ECI rotation matrix columns P̂ and Q̂
    val si = sin(iRad).toFloat(); val ci = cos(iRad).toFloat()
    val sO = sin(raanRad).toFloat(); val cO = cos(raanRad).toFloat()
    val sw = sin(aopRad).toFloat();  val cw = cos(aopRad).toFloat()

    val Px =  cO * cw - sO * sw * ci;   val Qx = -cO * sw - sO * cw * ci
    val Py =  sO * cw + cO * sw * ci;   val Qy = -sO * sw + cO * cw * ci
    val Pz =  sw * si;                   val Qz =  cw * si

    val twoPi = (2.0 * PI).toFloat()

    return List(RING_DOT_COUNT) { i ->
        val nu = (i.toFloat() / RING_DOT_COUNT) * twoPi
        val r  = a * (1f - e * e) / (1f + e * cos(nu))
        val px = r * cos(nu); val py = r * sin(nu)
        val eciX = px * Px + py * Qx
        val eciY = px * Py + py * Qy
        val eciZ = px * Pz + py * Qz
        Float3(eciX, eciZ, -eciY)   // ECI → scene axis swap
    }
}

/**
 * SceneView 3D panel: textured Earth + Sparky spacecraft GLB + ECI/body-frame axes
 * + orbital trail + deep-space colour skybox.
 *
 * Coordinate mapping – simulation ECI -> Filament Y-up right-handed:
 *   sceneX = simX    (toward vernal equinox)
 *   sceneY = simZ    (north pole up)
 *   sceneZ = -simY   (right-hand rule)
 *
 * Axis colour convention (matches legacy XyzRgbTG.java): X=red  Y=green  Z=blue
 *
 * Spacecraft attitude: OrbiterState (q0, qi, qj, qk) is inertial-to-body
 * passive rotation.  Filament uses active rotation; with axis swap:
 *   node.worldQuaternion = Quaternion(x=qi, y=qk, z=-qj, w=q0)
 */
@Composable
fun OrbiterSceneView(
    state: OrbiterState,
    modifier: Modifier = Modifier
) {
    val context = LocalContext.current
    val currentState by rememberUpdatedState(state)
    // Independent frame counter for trail throttle.  Using buffer size (n % 3)
    // fails once the ring-buffer is full because n stays constant at TRAIL_MAX.
    val frameCounter = remember { intArrayOf(0) }

    // ── Orbital ring state ────────────────────────────────────────────────────
    // prevSceneY: previous frame's Y coordinate, used to detect ascending-node
    // crossing (sceneY goes from < 0 to ≥ 0).
    // ringReady: false until the ring is drawn at least once (forces first draw).
    val prevSceneY    = remember { floatArrayOf(0f) }
    val ringReady     = remember { booleanArrayOf(false) }
    val lastBurnEpoch = remember { intArrayOf(-1) }

    // Camera mode — mutableStateOf drives HUD recomposition; booleanArrayOf is the
    // stable reference read inside the Filament onFrame lambda (no snapshot needed).
    var cameraMode     by remember { mutableStateOf(CameraMode.ECI) }
    val chaseModeFlag  = remember { booleanArrayOf(false) }

    val engine          = rememberEngine()
    val modelLoader     = rememberModelLoader(engine)
    val materialLoader  = rememberMaterialLoader(engine)
    val cameraNode      = rememberCameraNode(engine) {
        worldPosition = Float3(0f, 1.0f, 4.5f)
    }

    // ── Star-field skybox + uniform ambient IBL ──────────────────────────────
    // We load the star-field HDR for the skybox background, but REPLACE its
    // derived IBL with a hand-crafted IndirectLight that uses only the L00
    // spherical harmonic (the DC / constant term).  A pure L00 irradiance is
    // identical from every direction, so Lambertian surfaces (roughness=1) are
    // uniformly lit regardless of which way their normals face — axes and the
    // globe will be fully visible from any viewing angle.
    //
    // Intensity = 72 000 lux (2× previous 36 000 — 100% increase as requested).
    // The star skybox is retained unchanged for the visual background.
    val environmentLoader = rememberEnvironmentLoader(engine)
    val starEnvironment = rememberEnvironment(engine) {
        val env = environmentLoader.createHDREnvironment("environments/stars.hdr")
        // L00-only irradiance: 3 floats = [R, G, B] for the l=0,m=0 SH band.
        // Setting only this band gives perfectly uniform ambient illumination.
        val uniformIBL = IndirectLight.Builder()
            .irradiance(1, floatArrayOf(1.0f, 1.0f, 1.0f))
            .intensity(72_000f)
            .build(engine)
        Environment(
            indirectLight = uniformIBL,
            skybox = env?.skybox ?: Skybox.Builder()
                .color(0.001f, 0.001f, 0.004f, 1.0f)
                .build(engine)
        )
    }

    // ── Earth texture ─────────────────────────────────────────────────────────
    val earthTexture = remember(engine) {
        ImageTexture.Builder()
            .bitmap(context.assets, "earth_lights_lrg.jpg")
            .build(engine)
    }

    // ── Earth sphere ──────────────────────────────────────────────────────────
    val earthNode = rememberNode {
        SphereNode(
            engine = engine,
            radius = 1.0f,
            center = Float3(0f, 0f, 0f),
            stacks = 64,
            slices = 64,
            materialInstance = materialLoader.createTextureInstance(
                earthTexture, false, 0.0f, 1.0f, 0.0f
            )
        )
    }

    // ── Sparky spacecraft model ───────────────────────────────────────────────
    val satNode = rememberNode {
        val instance = modelLoader.createModelInstance("models/sparky.glb")
        ModelNode(
            modelInstance = instance,
            autoAnimate = false,
            scaleToUnits = null,
            centerOrigin = Float3(0f, 0f, 0f)
        )
    }

    // ── ECI reference-frame axes ──────────────────────────────────────────────
    // CylinderNode default axis is local +Y; rotate each to point along scene X/Y/Z.
    //   X (+sceneX): rotate -90° about +Z  →  Y → +X
    //   Y (+sceneY): identity
    //   Z (+sceneZ): rotate +90° about +X  →  Y → +Z
    // center = Float3(0, L/2, 0) shifts the cylinder bottom to the node origin.

    // Axes start at the sphere surface (offset = earthRadius = 1.0 ER) and extend
    // outward by eciLen, so the full rod is visible with no part buried inside the
    // globe and no Z-fighting at the sphere boundary.
    val earthRadius = 1.0f
    val eciLen = 1.5f                // visible length above/beyond sphere surface
    val eciR   = 0.012f
    // center in LOCAL space puts the cylinder bottom at Y = earthRadius
    val eciCenter = Float3(0f, earthRadius + eciLen / 2f, 0f)

    val eciXNode = rememberNode {
        CylinderNode(engine, eciR, eciLen, eciCenter, 8,
            materialLoader.createColorInstance(
                color = Color(1f, 0.1f, 0.1f, 1f), metallic = 0f, roughness = 1f, reflectance = 0f)
        ).apply { quaternion = Quaternion.fromAxisAngle(Float3(0f, 0f, 1f), -90f) }
    }
    val eciYNode = rememberNode {
        // North-pole axis (+sceneY = simZ). Starts at north-pole surface, extends 1.5 ER above.
        CylinderNode(engine, eciR, eciLen, eciCenter, 8,
            materialLoader.createColorInstance(
                color = Color(0f, 1f, 0f, 1f), metallic = 0f, roughness = 1f, reflectance = 0f)
        )
    }
    val eciZNode = rememberNode {
        CylinderNode(engine, eciR, eciLen, eciCenter, 8,
            materialLoader.createColorInstance(
                color = Color(0.1f, 0.4f, 1f, 1f), metallic = 0f, roughness = 1f, reflectance = 0f)
        ).apply { quaternion = Quaternion.fromAxisAngle(Float3(1f, 0f, 0f), 90f) }
    }

    // ── Body-frame axis stubs (children of satNode) ───────────────────────────
    val bodyLen = 0.25f
    val bodyR   = 0.004f

    // Body-axis stub rotations are derived from the inverse of MODEL_BODY_OFFSET
    // (inverse permutation X→Z, Y→X, Z→Y) so each stub points along the true
    // simulation body axis even though satNode carries MODEL_BODY_OFFSET.
    //
    //  Cylinder default axis = local +Y.  Required mapping (localQuat * +Y = target):
    //    X stub: +Y → +Z  →  +90° about +X
    //    Y stub: +Y → +X  →  -90° about +Z
    //    Z stub: +Y → +Y  →  identity
    //
    // Material: fully diffuse (roughness=1, reflectance=0) so rendered colour matches
    // the legend exactly under any lighting — same as the ECI axis nodes.
    val bodyXNode = rememberNode {
        CylinderNode(engine, bodyR, bodyLen, Float3(0f, bodyLen / 2f, 0f), 6,
            materialLoader.createColorInstance(
                color = Color(1f, 0.15f, 0.15f, 1f), metallic = 0f, roughness = 1f, reflectance = 0f)
        ).apply { quaternion = Quaternion.fromAxisAngle(Float3(1f, 0f, 0f), 90f) }
    }
    val bodyYNode = rememberNode {
        CylinderNode(engine, bodyR, bodyLen, Float3(0f, bodyLen / 2f, 0f), 6,
            materialLoader.createColorInstance(
                color = Color(0.15f, 1f, 0.15f, 1f), metallic = 0f, roughness = 1f, reflectance = 0f)
        ).apply { quaternion = Quaternion.fromAxisAngle(Float3(0f, 0f, 1f), 90f) }
    }
    val bodyZNode = rememberNode {
        CylinderNode(engine, bodyR, bodyLen, Float3(0f, bodyLen / 2f, 0f), 6,
            materialLoader.createColorInstance(
                color = Color(0.15f, 0.5f, 1f, 1f), metallic = 0f, roughness = 1f, reflectance = 0f)
        ).apply { quaternion = Quaternion.fromAxisAngle(Float3(1f, 0f, 0f), 180f) }
    }

    // Attach body-axis stubs once; rememberNode disposes them automatically —
    // do NOT call removeChildNode in onDispose (the Filament Scene may already
    // be destroyed by that point, causing a PreconditionPanic).
    DisposableEffect(satNode, bodyXNode, bodyYNode, bodyZNode) {
        satNode.addChildNode(bodyXNode)
        satNode.addChildNode(bodyYNode)
        satNode.addChildNode(bodyZNode)
        // Hidden for now; a UI toggle will re-enable them later (Step 7).
        bodyXNode.isVisible = false
        bodyYNode.isVisible = false
        bodyZNode.isVisible = false
        onDispose { /* intentionally empty — SceneView/rememberNode owns cleanup */ }
    }

    // ── Orbital trail (Step 4) ────────────────────────────────────────────────
    // Ring-buffer of the last TRAIL_MAX scene-space positions.
    //
    // Geometry: TRIANGLE_STRIP ribbon, 2 vertices per trail position (left/right
    // edge), total TRAIL_MAX×2 pre-allocated vertices.  Ribbon is ~100 km wide,
    // aligned perpendicular to the orbital plane so it's visible from any typical
    // viewing angle (widthDir = cross(tangent, radial) ≈ orbital-plane normal).
    //
    // Fade technique — RGB brightness, NOT alpha:
    //   head (newest, distFromHead=0) : Float4(0.05, 1.0, 1.0, 1.0) bright cyan
    //   tail of fade window           : Float4(0, 0, 0, 1)  black
    //   beyond TRAIL_FADE             : vertex collapsed to anchorPos (seamless)
    //
    // Normals = radial (away from Earth centre) so PBR lighting is computed
    // correctly — this is why the old LINE_STRIP was invisible (no normals).

    val trailBuffer = remember { ArrayDeque<Float3>(TRAIL_MAX) }

    val trailMaterial = remember(materialLoader) {
        materialLoader.createColorInstance(
            color = Color(0.2f, 1f, 1f, 1f),   // bright teal-cyan base / fallback
            metallic = 0f, roughness = 1f, reflectance = 0f
        )
    }

    // Pre-allocate TRAIL_MAX×2 vertices (Filament VertexBuffer is fixed-size).
    // First two vertices are at slightly different Y positions to give a
    // non-zero AABB and avoid a Filament PreconditionPanic at startup.
    val trailNode = rememberNode {
        val blankVerts = List(TRAIL_MAX * 2) { i ->
            Geometry.Vertex(
                position = Float3(0f, 1.1f + i * 0.0001f, 0f),
                normal   = Float3(0f, 1f, 0f),
                color    = Float4(0f, 0f, 0f, 1f)
            )
        }
        GeometryNode(
            engine = engine,
            geometry = Geometry.Builder(RenderableManager.PrimitiveType.TRIANGLE_STRIP)
                .vertices(blankVerts)
                .indices(List(TRAIL_MAX * 2) { it })
                .build(engine),
            materialInstance = trailMaterial
        )
    }

    // ── Orbital ring (Step 9 preview) ─────────────────────────────────────────
    // One full Keplerian ellipse as RING_DOT_COUNT small SphereNodes.
    // Spheres are visible from every camera angle (no face-on ribbon issue).
    // Positions are recomputed on every ascending-node crossing.
    val ringDotMaterial = remember(materialLoader) {
        materialLoader.createColorInstance(
            color       = Color(0f, 0.9f, 1f, 1f),  // cyan
            metallic    = 0f,
            roughness   = 1f,
            reflectance = 0f
        )
    }
    val ringDotNodes = remember(engine, ringDotMaterial) {
        List(RING_DOT_COUNT) { i ->
            SphereNode(
                engine           = engine,
                radius           = RING_DOT_RADIUS,
                center           = Float3(0f, 0f, 0f),
                stacks           = 6,
                slices           = 6,
                materialInstance = ringDotMaterial
            ).also { it.worldPosition = Float3(0f, 2f + i * 0.0001f, 0f) }
        }
    }

    // Overlay the 3D scene with a small "ECI Frame" label at bottom-left.
    Box(modifier = modifier) {
    Scene(
        modifier    = Modifier.fillMaxSize(),
        engine      = engine,
        modelLoader = modelLoader,
        materialLoader = materialLoader,
        environment = starEnvironment,
        cameraNode  = cameraNode,
        // trailNode excluded — orbital trail is a feature in work (see IMPROVEMENTS.md).
        // The ribbon geometry, ring-buffer, and frame-counter code is retained below
        // and can be re-enabled by adding trailNode back to this list.
        childNodes  = remember(earthNode, satNode, eciXNode, eciYNode, eciZNode, ringDotNodes) {
            listOf(earthNode, satNode, eciXNode, eciYNode, eciZNode) + ringDotNodes
        },
        cameraManipulator = rememberCameraManipulator(
            Float3(0f, 1.0f, 4.5f),
            Float3(0f, 0f, 0f)
        ),
        onFrame = { _ ->
            val s = currentState

            // ── Spacecraft position & attitude ────────────────────────────
            val scenePos = Float3(
                x =  s.posX.toFloat(),
                y =  s.posZ.toFloat(),
                z = -s.posY.toFloat()
            )
            satNode.worldPosition = scenePos
            // Compose: attitudeQuat × MODEL_BODY_OFFSET
            //   MODEL_BODY_OFFSET rotates the mesh (old Z→body X, old Y→body Z)
            //   attitudeQuat then applies the spacecraft's inertial attitude on top.
            // Note: OrbiterState provides a PASSIVE (inertial-to-body) quaternion.
            // Filament requires an ACTIVE (body-to-inertial) quaternion. We must conjugate 
            // the passive quaternion by negating its vector components (qi, qj, qk).
            // Then we map the ECI axes to Filament axes (X -> X, Y -> Z, Z -> -Y).
            val attQuat = Quaternion(
                x = -s.qi.toFloat(),
                y = -s.qk.toFloat(),
                z =  s.qj.toFloat(), // -(-s.qj)
                w =  s.q0.toFloat()
            )
            satNode.worldQuaternion = attQuat * MODEL_BODY_OFFSET

            // ── Chase camera override (ECEF frame) ───────────────────────────────
            // The CameraManipulator maintains its own internal ECI state (orbiting
            // world origin) and is unaffected by our per-frame override.  We treat
            // its eye-position as an ECEF offset (rotating with Earth) by applying
            // Earth's rotation angle thetaGAST about scene Y before adding scenePos.
            //
            //   camPos = scenePos  +  R_y(thetaGAST) × (manipECI × CHASE_ECEF_SCALE)
            //
            // Touch drag  → manipulator changes direction → camera orbits in ECEF
            // Pinch zoom  → manipulator changes distance  → camera moves closer/farther
            // Spacecraft translates → scenePos shifts    → camera follows
            // Earth rotates → thetaGAST grows            → camera rotates with Earth
            if (chaseModeFlag[0]) {
                // The CameraManipulator's internal state is ECI-aligned (orbiting world
                // origin).  We interpret its eye position as an ECEF-frame offset by
                // rotating it by -thetaGAST about scene Y (the north-pole axis).
                //
                //   ECEF → ECI rotation by angle θ about scene Y (= ECI Z = north pole):
                //     x' =  x·cosθ + z·sinθ
                //     y' =  y  (pole axis unchanged)
                //     z' = -x·sinθ + z·cosθ
                //
                // thetaGAST accumulates at Earth's rotation rate:
                //   ωE = 4.37527e-3 rad/min → θ = ωE × s.time  (s.time is in minutes)
                val manipScaled = cameraNode.worldPosition * CHASE_ECEF_SCALE
                val thetaRad = (4.37527e-3 * s.time).toFloat()
                val cosT = cos(thetaRad)
                val sinT = sin(thetaRad)
                val eciOffset = Float3(
                    x = manipScaled.x * cosT + manipScaled.z * sinT,
                    y = manipScaled.y,
                    z = -manipScaled.x * sinT + manipScaled.z * cosT
                )
                val camPos = scenePos + eciOffset
                cameraNode.worldPosition = camPos
                cameraNode.worldQuaternion = cameraLookAt(camPos, scenePos)
            }

            // ── Body-axis stub visibility (shown only in CHASE mode) ─────────────
            val chase = chaseModeFlag[0]
            bodyXNode.isVisible = chase
            bodyYNode.isVisible = chase
            bodyZNode.isVisible = chase

            // ── Orbital ring — redraw on first frame, ascending-node, or new burn ──
            // burnFired: state.burnEpoch changed since last check.
            // Because burnEpoch is embedded in OrbiterState by publishState(), the ring
            // always redraws with the correct post-burn orbital elements (never stale).
            val burnFired = s.burnEpoch != lastBurnEpoch[0]
            if (burnFired) lastBurnEpoch[0] = s.burnEpoch

            val atAscendingNode = !ringReady[0] ||
                (prevSceneY[0] < 0f && scenePos.y >= 0f)
            prevSceneY[0] = scenePos.y
            if (atAscendingNode || burnFired) {
                ringReady[0] = true
                val positions = computeOrbitalRingPositions(
                    a       = s.semiMajorAxisEr.toFloat(),
                    e       = s.eccentricity.toFloat(),
                    iRad    = (s.inclinationDeg * PI / 180.0).toFloat(),
                    raanRad = (s.raanDeg        * PI / 180.0).toFloat(),
                    aopRad  = (s.aopDeg         * PI / 180.0).toFloat()
                )
                ringDotNodes.forEachIndexed { i, node ->
                    node.worldPosition = positions[i]
                }
            }

            // ── Trail ring-buffer (feature in work — currently disabled) ─────
            // trailNode is not in childNodes; re-enable by:
            //   1. Adding trailNode back to childNodes below.
            //   2. Removing the `if (false)` guard around this block.
            @Suppress("ConstantConditionIf")
            if (false) { // trail disabled — remove guard when re-enabling
                trailBuffer.addLast(scenePos)
                if (trailBuffer.size > TRAIL_MAX) trailBuffer.removeFirst()
                frameCounter[0]++
                val n = trailBuffer.size
                if (n >= 2 && (frameCounter[0] % 3 == 0)) {
                    val anchorIdx = (n - TRAIL_FADE).coerceAtLeast(0)
                    val anchorPos = trailBuffer[anchorIdx]

                    val verts = List(TRAIL_MAX * 2) { i ->
                        val trailIdx = i / 2
                        val side = if (i % 2 == 0) -1f else 1f  // left / right edge

                        if (trailIdx < n) {
                            val distFromHead = n - 1 - trailIdx
                            if (distFromHead < TRAIL_FADE) {
                                val pos = trailBuffer[trailIdx]

                                // Tangent: forward-diff, back-diff at ends
                                val tangent = when {
                                    trailIdx < n - 1 ->
                                        normalize(trailBuffer[trailIdx + 1] - pos)
                                    else ->
                                        normalize(pos - trailBuffer[trailIdx - 1])
                                }
                                // Radial unit vector (away from Earth centre)
                                val radial = normalize(pos)

                                // widthDir ⊥ tangent and ⊥ radial ≈ orbital-plane normal
                                val raw = cross(tangent, radial)
                                val lenSq = raw.x * raw.x + raw.y * raw.y + raw.z * raw.z
                                val widthDir = if (lenSq > 1e-6f) raw / sqrt(lenSq) else Float3(0f, 0f, 1f)

                                // Quadratic brightness fade: t=1 at head, 0 at tail
                                val t  = 1f - distFromHead.toFloat() / TRAIL_FADE.toFloat()
                                val br = t * t
                                Geometry.Vertex(
                                    position = pos + widthDir * (side * TRAIL_HALF_WIDTH),
                                    normal   = radial,
                                    color    = Float4(0.05f * br, br, br, 1f)
                                )
                            } else {
                                // Beyond fade window: collapse to anchor (zero-area tri)
                                Geometry.Vertex(position = anchorPos, normal = Float3(0f, 1f, 0f), color = Float4(0f, 0f, 0f, 1f))
                            }
                        } else {
                            // Padding slots
                            Geometry.Vertex(position = anchorPos, normal = Float3(0f, 1f, 0f), color = Float4(0f, 0f, 0f, 1f))
                        }
                    }
                    trailNode.updateGeometry(verts, listOf(List(TRAIL_MAX * 2) { it }))
                } // end if (n >= 2...)
            } // end if (false) trail disabled
        } // end onFrame
    ) // end Scene
    // ── HUD overlays ──────────────────────────────────────────────────────────
    // Top-left: camera mode indicator — tap to toggle ECI ↔ CHASE
    val isChase = cameraMode == CameraMode.CHASE
    Box(
        modifier = Modifier
            .align(Alignment.TopStart)
            .padding(8.dp, 8.dp)
            .clickable {
                val newMode = if (isChase) CameraMode.ECI else CameraMode.CHASE
                cameraMode = newMode
                chaseModeFlag[0] = (newMode == CameraMode.CHASE)
            }
    ) {
        Row(
            verticalAlignment = Alignment.CenterVertically,
            horizontalArrangement = Arrangement.spacedBy(6.dp)
        ) {
            Box(
                modifier = Modifier
                    .size(8.dp)
                    .clip(CircleShape)
                    .background(if (isChase) Color(0xFFE8A947) else Color(0xFF5CD07B))
            )
            Text(
                if (isChase) "CHASE" else "ECI FRAME",
                color = Color(0xFFC8D4EA),
                fontSize = 10.sp,
                fontFamily = FontFamily.Monospace,
                letterSpacing = 0.14.sp
            )
        }
    }
    // Top-right: lat/lon
    Text(
        text = "LAT ${"%.2f".format(state.latDeg)}\u00B0  \u00B7  LON ${"%.2f".format(state.lonDeg)}\u00B0",
        color = Color(0xFFC8D4EA),
        fontSize = 10.sp,
        fontFamily = FontFamily.Monospace,
        letterSpacing = 0.14.sp,
        modifier = Modifier
            .align(Alignment.TopEnd)
            .padding(8.dp, 8.dp)
    )
    // Bottom-left: axis legend — ECI always, body-frame only in CHASE mode.
    // Colors match the actual CylinderNode material colors.
    Column(
        modifier = Modifier
            .align(Alignment.BottomStart)
            .padding(8.dp, 8.dp),
        verticalArrangement = Arrangement.spacedBy(6.dp)
    ) {
        Row(
            horizontalArrangement = Arrangement.spacedBy(10.dp),
            verticalAlignment = Alignment.CenterVertically
        ) {
            AxisLegendItem(Color(0xFFFF1A1A), "X-ECI")   // matches Color(1f,0.1f,0.1f)
            AxisLegendItem(Color(0xFF00FF00), "Y-ECI")   // matches Color(0f,1f,0f)
            AxisLegendItem(Color(0xFF1A66FF), "Z-ECI")   // matches Color(0.1f,0.4f,1f)
        }
        if (isChase) {
            Row(
                horizontalArrangement = Arrangement.spacedBy(10.dp),
                verticalAlignment = Alignment.CenterVertically
            ) {
                AxisLegendItem(Color(0xFFFF2626), "X-BODY")  // matches Color(1f,0.15f,0.15f)
                AxisLegendItem(Color(0xFF26FF26), "Y-BODY")  // matches Color(0.15f,1f,0.15f)
                AxisLegendItem(Color(0xFF2680FF), "Z-BODY")  // matches Color(0.15f,0.5f,1f)
            }
        }
    }
    // Bottom-right: ground track
    Text(
        text = "GROUND TRACK  \u00B7  T+${(state.time / 60.0).toInt()}m",
        color = Color(0xFF6E7FA0),
        fontSize = 10.sp,
        fontFamily = FontFamily.Monospace,
        letterSpacing = 0.14.sp,
        modifier = Modifier
            .align(Alignment.BottomEnd)
            .padding(8.dp, 8.dp)
    )
    } // end Box
} // end OrbiterSceneView

@Composable
private fun AxisLegendItem(color: Color, label: String) {
    Row(verticalAlignment = Alignment.CenterVertically, horizontalArrangement = Arrangement.spacedBy(4.dp)) {
        Box(modifier = Modifier.width(14.dp).height(2.dp).background(color))
        Text(label, color = Color(0xFFC8D4EA), fontSize = 10.sp, fontFamily = FontFamily.Monospace)
    }
}
