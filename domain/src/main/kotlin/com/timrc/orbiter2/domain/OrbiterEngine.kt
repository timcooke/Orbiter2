package com.timrc.orbiter2.domain

import com.timrc.orbiter2.domain.enums.Basis3D
import com.timrc.orbiter2.domain.enums.FTxyz
import com.timrc.orbiter2.domain.enums.KepEuler
import com.timrc.orbiter2.domain.enums.XdX6DQ
import com.timrc.orbiter2.domain.envrm.Gravity
import com.timrc.orbiter2.domain.envrm.ICentralBody
import com.timrc.orbiter2.domain.math.Quaternion
import com.timrc.orbiter2.domain.math.SphericalHarmonicCoeff
import com.timrc.orbiter2.domain.math.Tuple3D
import com.timrc.orbiter2.domain.trmtm.KeplerianOE
import com.timrc.orbiter2.domain.trmtm.OrbiterSys
import com.timrc.orbiter2.domain.trmtm.StateKepEuler
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.Job
import kotlinx.coroutines.SupervisorJob
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.isActive
import kotlinx.coroutines.launch

enum class HoldMode {
    FREE, PROGRADE, RETROGRADE, NORMAL, ANTINORMAL, RADIAL, ANTIRADIAL, TARGET, SUN
}

/** All-double snapshot for zero-allocation Compose equality checks. */
data class OrbiterState(
    // Inertial position (Earth Radii)
    val posX: Double = 0.0,
    val posY: Double = 0.0,
    val posZ: Double = 0.0,
    // Inertial velocity (ER/min)
    val velX: Double = 0.0,
    val velY: Double = 0.0,
    val velZ: Double = 0.0,
    // Inertial-to-body quaternion components
    val q0: Double = 1.0,
    val qi: Double = 0.0,
    val qj: Double = 0.0,
    val qk: Double = 0.0,
    // Simulation time (minutes)
    val time: Double = 0.0,
    // Osculating Keplerian elements
    val semiMajorAxisEr: Double = 0.0,
    val eccentricity: Double = 0.0,
    val inclinationDeg: Double = 0.0,
    val raanDeg: Double = 0.0,
    val aopDeg: Double = 0.0,
    val trueAnomalyDeg: Double = 0.0,
    val latDeg: Double = 0.0,    // geocentric latitude, degrees [-90, +90]
    val lonDeg: Double = 0.0,    // geographic longitude with Earth rotation, degrees [-180, +180]
    // RPY-frame Euler angles (bank/elevation/heading)
    val bankDeg: Double = 0.0,
    val elevationDeg: Double = 0.0,
    val headingDeg: Double = 0.0,
    // Incremented every time applyBurn() is called; used to trigger immediate
    // ring redraws in the UI with the correct post-burn orbital elements.
    val burnEpoch: Int = 0,
) {
    /** Position magnitude in ER. */
    val posMagEr: Double get() = Math.sqrt(posX*posX + posY*posY + posZ*posZ)
    /** Altitude above Earth surface in km. */
    val altitudeKm: Double get() = (posMagEr - 1.0) * 6378.137
    /** Speed in km/s. */
    val speedKms: Double get() {
        val magErMin = Math.sqrt(velX*velX + velY*velY + velZ*velZ)
        return magErMin * 6378.137 / 60.0
    }
}

/**
 * Coroutine-based 6DOF spacecraft simulation engine.
 *
 * Integrates physics at [stepSizeSec] simulation-seconds per step,
 * publishes [OrbiterState] via [state] at up to [stepHz] Hz wall-clock.
 *
 * Lifecycle: call [start] with a coroutine scope, [stop] to cancel.
 */
class OrbiterEngine(
    private val stepSizeSec: Double = 10.0,
    private val stepHz: Long = 30L
) {
    private val sim = OrbiterSys()

    @Volatile private var holdMode: HoldMode = HoldMode.FREE
    @Volatile private var burnEpoch: Int = 0

    private val _state = MutableStateFlow(OrbiterState())
    val state: StateFlow<OrbiterState> = _state.asStateFlow()

    private val outputBuffer = StateKepEuler()
    private val posBuffer = Tuple3D()
    private val velBuffer = Tuple3D()
    private val qBuffer = Quaternion()

    private var job: Job? = null
    private val engineScope = CoroutineScope(SupervisorJob() + Dispatchers.Default)

    init {
        setupEarth()
        setupLEO()
    }

    fun start(externalScope: CoroutineScope = engineScope) {
        if (job?.isActive == true) return
        job = externalScope.launch(Dispatchers.Default) {
            val delayMs = 1000L / stepHz
            while (isActive) {
                sim.step(stepSizeSec / 60.0)   // convert sim-seconds → canonical minutes
                publishState()
                delay(delayMs)
            }
        }
    }

    fun stop() { job?.cancel(); job = null }

    fun setForce(axis: FTxyz, value: Double) { sim.setU(axis, value) }

    fun clearControls() { FTxyz.entries.forEach { sim.setU(it, 0.0) } }

    fun setAttitudeHold(mode: HoldMode) { holdMode = mode }

    /** Apply an instantaneous velocity impulse of [dvMps] m/s in the current prograde direction. */
    fun applyBurn(dvMps: Double) {
        val vx = sim.getX(XdX6DQ.DX); val vy = sim.getX(XdX6DQ.DY); val vz = sim.getX(XdX6DQ.DZ)
        val vMag = Math.sqrt(vx*vx + vy*vy + vz*vz)
        if (vMag < 1e-12) return
        val dvErMin = dvMps / (6_378_137.0 / 60.0)   // m/s → ER/min
        val scale = dvErMin / vMag
        sim.setX(XdX6DQ.DX, vx + vx * scale)
        sim.setX(XdX6DQ.DY, vy + vy * scale)
        sim.setX(XdX6DQ.DZ, vz + vz * scale)
        // Increment here; publishState() will embed this in the next OrbiterState,
        // guaranteeing the ring redraw sees the correct post-burn orbital elements.
        burnEpoch++
    }

    /** Apply a small RCS translational impulse. [dx],[dy],[dz] are ±1 body-frame directions; [magMps] in m/s. */
    fun jogRCS(dx: Int, dy: Int, dz: Int, magMps: Double) {
        if (dx == 0 && dy == 0 && dz == 0) return
        val dMag = Math.sqrt((dx*dx + dy*dy + dz*dz).toDouble())
        val bx = dx / dMag; val by = dy / dMag; val bz = dz / dMag
        // Rotate body impulse direction to ECI using q_conj (b2i = inverse of passive i2b)
        val q0c =  sim.getX(XdX6DQ.Q0)
        val qic = -sim.getX(XdX6DQ.QI)
        val qjc = -sim.getX(XdX6DQ.QJ)
        val qkc = -sim.getX(XdX6DQ.QK)
        // Rodrigues: v_eci = v_body + 2*q0c * cross(q_vec_c, v_body) + 2 * cross(q_vec_c, cross(q_vec_c, v_body))
        val cx = qjc*bz - qkc*by; val cy = qkc*bx - qic*bz; val cz = qic*by - qjc*bx
        val c2x = qjc*cz - qkc*cy; val c2y = qkc*cx - qic*cz; val c2z = qic*cy - qjc*cx
        val eciX = bx + 2*q0c*cx + 2*c2x
        val eciY = by + 2*q0c*cy + 2*c2y
        val eciZ = bz + 2*q0c*cz + 2*c2z
        val magErMin = magMps / (6_378_137.0 / 60.0)
        sim.setX(XdX6DQ.DX, sim.getX(XdX6DQ.DX) + eciX * magErMin)
        sim.setX(XdX6DQ.DY, sim.getX(XdX6DQ.DY) + eciY * magErMin)
        sim.setX(XdX6DQ.DZ, sim.getX(XdX6DQ.DZ) + eciZ * magErMin)
    }

    // ── private ──────────────────────────────────────────────────────────

    private fun setupEarth() { sim.enableGravity(EarthModel()) }

    private fun setupLEO() {
        val gm = EarthModel.GM
        val a = (6378.137 + 400.0) / 6378.137   // ~1.0627 ER
        val koe = KeplerianOE(gm, a, 0.0, Math.toRadians(51.6), 0.0, 0.0, 0.0)
        sim.setOE(koe)
        publishState()
    }

    private fun publishState() {
        val t = sim.getT()
        sim.getPosition(t, posBuffer)
        sim.getVelocity(t, velBuffer)
        sim.getAttitude(t, qBuffer)
        sim.getY(outputBuffer)

        // ── Lat / lon ────────────────────────────────────────────────────────────
        val px = posBuffer.get(Basis3D.I); val py = posBuffer.get(Basis3D.J); val pz = posBuffer.get(Basis3D.K)
        val rMag = Math.sqrt(px*px + py*py + pz*pz)
        val latRad = if (rMag > 1e-12) Math.asin(pz.coerceIn(-rMag, rMag) / rMag) else 0.0
        val lonEciRad = Math.atan2(py, px)
        val thetaGAST = 4.37527e-3 * t       // Earth rotation: ~0.004375 rad/min × time (min)
        var lonEcefRad = lonEciRad - thetaGAST
        while (lonEcefRad >  Math.PI) lonEcefRad -= 2.0 * Math.PI
        while (lonEcefRad < -Math.PI) lonEcefRad += 2.0 * Math.PI
        val latDeg = Math.toDegrees(latRad)
        val lonDeg = Math.toDegrees(lonEcefRad)

        // ── Attitude hold ─────────────────────────────────────────────────────────
        // Compute target quaternion for the selected hold mode and set it directly.
        // Helper convention: rows of the passive rotation matrix are the body axes
        // expressed in ECI.  shepperd() converts that 3×3 matrix to a quaternion.
        // frameBxBz(): body +X primary (exact), body +Z from secondary reference.
        // frameBzBx(): body +Z primary (exact), body +X from secondary reference.
        if (holdMode != HoldMode.FREE && holdMode != HoldMode.TARGET) {
            val vx = velBuffer.get(Basis3D.I); val vy = velBuffer.get(Basis3D.J); val vz = velBuffer.get(Basis3D.K)
            val vMag = Math.sqrt(vx*vx + vy*vy + vz*vz)
            val rx = if (rMag > 1e-12) px/rMag else 0.0   // radial unit  (ECI I,J,K)
            val ry = if (rMag > 1e-12) py/rMag else 0.0
            val rz = if (rMag > 1e-12) pz/rMag else 0.0
            val vux = if (vMag > 1e-12) vx/vMag else 0.0  // prograde unit
            val vuy = if (vMag > 1e-12) vy/vMag else 0.0
            val vuz = if (vMag > 1e-12) vz/vMag else 0.0

            val q: DoubleArray? = when (holdMode) {

                // +Veci — body +X = ECI prograde (primary), body +Z = nadir (secondary)
                HoldMode.PROGRADE -> if (rMag < 1e-12 || vMag < 1e-12) null else
                    frameBxBz(vux, vuy, vuz, -rx, -ry, -rz)

                // -Veci — body +X = ECI retrograde (primary), body +Z = nadir (secondary)
                HoldMode.RETROGRADE -> if (rMag < 1e-12 || vMag < 1e-12) null else
                    frameBxBz(-vux, -vuy, -vuz, -rx, -ry, -rz)

                // LVLH — body +Z = nadir (primary), body +X = ECEF velocity (secondary)
                // ECEF velocity in ECI: v_ecef = v_eci − ω×r, ω = [0,0,ωE] (rad/min)
                // ω×r = (−ωE·py, ωE·px, 0) → v_ecef = (vx+ωE·py, vy−ωE·px, vz)
                HoldMode.NORMAL -> if (rMag < 1e-12 || vMag < 1e-12) null else {
                    val omegaE = 4.37527e-3          // Earth rotation rate, rad/min
                    val vEx = vx + omegaE * py
                    val vEy = vy - omegaE * px
                    val vEz = vz
                    val vEMag = Math.sqrt(vEx*vEx + vEy*vEy + vEz*vEz)
                    if (vEMag < 1e-12) null else
                        frameBzBx(-rx, -ry, -rz, vEx/vEMag, vEy/vEMag, vEz/vEMag)
                }

                // ECI — body axes aligned with ECI axes → identity quaternion
                HoldMode.ANTINORMAL -> doubleArrayOf(1.0, 0.0, 0.0, 0.0)

                // ECEF — body axes co-rotate with Earth.
                // Passive ECI→ECEF rotation by thetaGAST about K axis:
                //   q = ( cos(θ/2), 0, 0, −sin(θ/2) )
                HoldMode.RADIAL -> {
                    val half = thetaGAST / 2.0
                    doubleArrayOf(Math.cos(half), 0.0, 0.0, -Math.sin(half))
                }

                // -R — keep original behaviour: body +X = anti-radial (primary),
                // body +Z derived from orbit-normal reference.
                HoldMode.ANTIRADIAL -> if (rMag < 1e-12 || vMag < 1e-12) null else {
                    val nx = ry*vuz - rz*vuy; val ny = rz*vux - rx*vuz; val nz = rx*vuy - ry*vux
                    val nMag = Math.sqrt(nx*nx + ny*ny + nz*nz)
                    if (nMag < 1e-12) null else {
                        val nux = nx/nMag; val nuy = ny/nMag; val nuz = nz/nMag
                        val bx0 = -rx; val bx1 = -ry; val bx2 = -rz
                        // by = cross(bx, orbit_normal)  →  bz = cross(bx, by)
                        var by0 = bx1*nuz - bx2*nuy
                        var by1 = bx2*nux - bx0*nuz
                        var by2 = bx0*nuy - bx1*nux
                        val byMag = Math.sqrt(by0*by0 + by1*by1 + by2*by2)
                        if (byMag < 1e-12) null else {
                            by0 /= byMag; by1 /= byMag; by2 /= byMag
                            val bz0 = bx1*by2 - bx2*by1
                            val bz1 = bx2*by0 - bx0*by2
                            val bz2 = bx0*by1 - bx1*by0
                            shepperd(bx0,bx1,bx2, by0,by1,by2, bz0,bz1,bz2)
                        }
                    }
                }

                // Sun pointing — body −Z toward Sun (primary), body +X toward ECI prograde (secondary).
                // Low-fidelity solar direction: circular Earth orbit, J2000 epoch reference.
                //   λ_sun = 280.46° + 0.9856°/day × t_min  (solar ecliptic longitude)
                //   obliquity ε = 23.4393°
                //   sun_ECI = (cos λ, cos ε · sin λ, sin ε · sin λ)   [unit vector, Earth→Sun]
                HoldMode.SUN -> if (vMag < 1e-12) null else {
                    val sunLon = Math.toRadians(280.46 + 0.0006844 * t)   // 0.9856/1440 deg/min
                    val eps    = Math.toRadians(23.4393)
                    val sI = Math.cos(sunLon)
                    val sJ = Math.cos(eps) * Math.sin(sunLon)
                    val sK = Math.sin(eps) * Math.sin(sunLon)
                    // body +Z = −sunHat  (so body −Z faces the Sun)
                    frameBzBx(-sI, -sJ, -sK, vux, vuy, vuz)
                }

                else -> null
            }

            if (q != null) {
                sim.setX(XdX6DQ.Q0, q[0]); sim.setX(XdX6DQ.QI, q[1])
                sim.setX(XdX6DQ.QJ, q[2]); sim.setX(XdX6DQ.QK, q[3])
                // Re-read attitude buffer to reflect the hold override
                sim.getAttitude(t, qBuffer)
            }
        }

        _state.value = OrbiterState(
            posX = posBuffer.get(Basis3D.I),
            posY = posBuffer.get(Basis3D.J),
            posZ = posBuffer.get(Basis3D.K),
            velX = velBuffer.get(Basis3D.I),
            velY = velBuffer.get(Basis3D.J),
            velZ = velBuffer.get(Basis3D.K),
            q0 = qBuffer.get(com.timrc.orbiter2.domain.enums.Q.Q0),
            qi = qBuffer.get(com.timrc.orbiter2.domain.enums.Q.QI),
            qj = qBuffer.get(com.timrc.orbiter2.domain.enums.Q.QJ),
            qk = qBuffer.get(com.timrc.orbiter2.domain.enums.Q.QK),
            time = t,
            semiMajorAxisEr = outputBuffer.get(KepEuler.A),
            eccentricity     = outputBuffer.get(KepEuler.E),
            inclinationDeg   = Math.toDegrees(outputBuffer.get(KepEuler.I)),
            raanDeg          = Math.toDegrees(outputBuffer.get(KepEuler.O)),
            aopDeg           = Math.toDegrees(outputBuffer.get(KepEuler.W)),
            trueAnomalyDeg   = Math.toDegrees(outputBuffer.get(KepEuler.V)),
            latDeg           = latDeg,
            lonDeg           = lonDeg,
            bankDeg          = Math.toDegrees(outputBuffer.get(KepEuler.BANK)),
            elevationDeg     = Math.toDegrees(outputBuffer.get(KepEuler.ELEV)),
            headingDeg       = Math.toDegrees(outputBuffer.get(KepEuler.HEAD)),
            burnEpoch        = this.burnEpoch,
        )
    }
}

/**
 * Minimal ICentralBody for Earth with 4×4 EGM96 gravity, fixed at origin.
 */
class EarthModel : ICentralBody {
    companion object {
        val GM = 3600.0 * 398600.4418 / (6378.137 * 6378.137 * 6378.137)
        const val RE = 1.0
    }

    private val gravity: Gravity

    init {
        val ngc = 5
        val clm = Array(ngc) { DoubleArray(ngc) }
        val slm = Array(ngc) { DoubleArray(ngc) }
        clm[2][0] = -1.08262668355e-3; clm[2][1] = -2.41400000000e-10; clm[2][2] =  1.57446037456e-6
        clm[3][0] =  2.53265648533e-6; clm[3][1] =  2.19263852917e-6;  clm[3][2] =  3.08989206881e-7; clm[3][3] =  1.00548778064e-7
        clm[4][0] =  1.61962159137e-6; clm[4][1] = -5.08799360404e-7;  clm[4][2] =  7.84175859844e-8; clm[4][3] =  5.92099402629e-8; clm[4][4] = -3.98407411766e-9
        slm[2][1] =  1.54310000000e-9; slm[2][2] = -9.03803806639e-7
        slm[3][1] =  2.68424890397e-7; slm[3][2] = -2.11437612437e-7;  slm[3][3] =  1.97222559006e-7
        slm[4][1] = -4.49144872839e-7; slm[4][2] =  1.48177868296e-7;  slm[4][3] = -1.20077667634e-8; slm[4][4] =  6.52571425370e-9
        gravity = Gravity(GM, RE, SphericalHarmonicCoeff(false, clm, slm))
    }

    override fun getGravParam(): Double = GM
    override fun getRefRadius(): Double = RE
    override fun getDegreeOrder(): Int = gravity.getDegreeOrder()
    override fun getPotential(r: Double, elevation: Double, azimuth: Double) = gravity.getPotential(r, elevation, azimuth)
    override fun getPotential(degree: Int, r: Double, elevation: Double, azimuth: Double) = gravity.getPotential(degree, r, elevation, azimuth)
    override fun getGravityModel() = gravity.getGravityModel()
    override fun getR(elevation: Double, azimuth: Double) = gravity.getR(elevation, azimuth)
    override fun getPosition(tReq: Double, tout: Tuple3D): Double { tout.set(0.0, 0.0, 0.0); return tReq }
    override fun getAttitude(tReq: Double, qout: Quaternion): Double { qout.identity(); return tReq }
}

// ── Attitude hold helpers ────────────────────────────────────────────────────────
// These are file-private (package-private in Kotlin terms) so they can live outside
// the OrbiterEngine class but still be co-located for easy maintenance.

/**
 * Convert a rotation matrix (given row-by-row, where each row is a body axis
 * expressed in the inertial frame) to a passive inertial-to-body quaternion
 * via Shepperd's method.
 *
 * Matrix layout:
 *   Row 0  =  body +X direction in inertial frame  (m00, m01, m02)
 *   Row 1  =  body +Y direction in inertial frame  (m10, m11, m12)
 *   Row 2  =  body +Z direction in inertial frame  (m20, m21, m22)
 *
 * Returns  doubleArrayOf(q0, qi, qj, qk).
 */
private fun shepperd(
    m00: Double, m01: Double, m02: Double,
    m10: Double, m11: Double, m12: Double,
    m20: Double, m21: Double, m22: Double
): DoubleArray {
    val tr = m00 + m11 + m22
    return if (tr > 0.0) {
        val s = 2.0 * Math.sqrt(tr + 1.0)
        doubleArrayOf(s/4.0, (m21-m12)/s, (m02-m20)/s, (m10-m01)/s)
    } else if (m00 > m11 && m00 > m22) {
        val s = 2.0 * Math.sqrt(1.0 + m00 - m11 - m22)
        doubleArrayOf((m21-m12)/s, s/4.0, (m01+m10)/s, (m20+m02)/s)
    } else if (m11 > m22) {
        val s = 2.0 * Math.sqrt(1.0 + m11 - m00 - m22)
        doubleArrayOf((m02-m20)/s, (m01+m10)/s, s/4.0, (m12+m21)/s)
    } else {
        val s = 2.0 * Math.sqrt(1.0 + m22 - m00 - m11)
        doubleArrayOf((m10-m01)/s, (m20+m02)/s, (m12+m21)/s, s/4.0)
    }
}

/**
 * Build a right-handed orthonormal body frame with body +X as the **primary** axis
 * (held exactly) and body +Z derived from a secondary reference direction via
 * Gram-Schmidt, then delegate to [shepperd].
 *
 * Construction:
 *   bY = normalize( cross(bzRef, bX) )
 *   bZ = cross(bX, bY)                    ← close to bzRef
 *
 * Returns null if the two input vectors are nearly parallel (degenerate frame).
 */
private fun frameBxBz(
    bx0: Double, bx1: Double, bx2: Double,        // primary: body +X  (unit vector)
    bzRef0: Double, bzRef1: Double, bzRef2: Double // secondary: approx body +Z (unit vector)
): DoubleArray? {
    var by0 = bzRef1*bx2 - bzRef2*bx1
    var by1 = bzRef2*bx0 - bzRef0*bx2
    var by2 = bzRef0*bx1 - bzRef1*bx0
    val byMag = Math.sqrt(by0*by0 + by1*by1 + by2*by2)
    if (byMag < 1e-12) return null
    by0 /= byMag; by1 /= byMag; by2 /= byMag
    val bz0 = bx1*by2 - bx2*by1
    val bz1 = bx2*by0 - bx0*by2
    val bz2 = bx0*by1 - bx1*by0
    return shepperd(bx0,bx1,bx2, by0,by1,by2, bz0,bz1,bz2)
}

/**
 * Build a right-handed orthonormal body frame with body +Z as the **primary** axis
 * (held exactly) and body +X derived from a secondary reference direction via
 * Gram-Schmidt, then delegate to [shepperd].
 *
 * Construction:
 *   bY = normalize( cross(bZ, bxRef) )
 *   bX = cross(bY, bZ)                    ← close to bxRef
 *
 * Returns null if the two input vectors are nearly parallel (degenerate frame).
 */
private fun frameBzBx(
    bz0: Double, bz1: Double, bz2: Double,        // primary: body +Z  (unit vector)
    bxRef0: Double, bxRef1: Double, bxRef2: Double // secondary: approx body +X (unit vector)
): DoubleArray? {
    var by0 = bz1*bxRef2 - bz2*bxRef1
    var by1 = bz2*bxRef0 - bz0*bxRef2
    var by2 = bz0*bxRef1 - bz1*bxRef0
    val byMag = Math.sqrt(by0*by0 + by1*by1 + by2*by2)
    if (byMag < 1e-12) return null
    by0 /= byMag; by1 /= byMag; by2 /= byMag
    val bx0 = by1*bz2 - by2*bz1
    val bx1 = by2*bz0 - by0*bz2
    val bx2 = by0*bz1 - by1*bz0
    return shepperd(bx0,bx1,bx2, by0,by1,by2, bz0,bz1,bz2)
}
