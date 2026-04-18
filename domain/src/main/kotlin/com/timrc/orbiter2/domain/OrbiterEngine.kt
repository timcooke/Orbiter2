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
        if (holdMode != HoldMode.FREE && holdMode != HoldMode.TARGET && holdMode != HoldMode.SUN) {
            val vx = velBuffer.get(Basis3D.I); val vy = velBuffer.get(Basis3D.J); val vz = velBuffer.get(Basis3D.K)
            val vMag = Math.sqrt(vx*vx + vy*vy + vz*vz)
            if (rMag > 1e-12 && vMag > 1e-12) {
                val rx = px/rMag; val ry = py/rMag; val rz = pz/rMag   // radial unit
                val vux = vx/vMag; val vuy = vy/vMag; val vuz = vz/vMag // prograde unit
                // Orbit normal = cross(r, v) normalised
                var nx = ry*vuz - rz*vuy; var ny = rz*vux - rx*vuz; var nz = rx*vuy - ry*vux
                val nMag = Math.sqrt(nx*nx + ny*ny + nz*nz)
                if (nMag > 1e-12) { nx /= nMag; ny /= nMag; nz /= nMag }

                // Body +X direction in ECI
                val (bx0, bx1, bx2) = when (holdMode) {
                    HoldMode.PROGRADE    -> Triple(vux, vuy, vuz)
                    HoldMode.RETROGRADE  -> Triple(-vux, -vuy, -vuz)
                    HoldMode.NORMAL      -> Triple(nx, ny, nz)
                    HoldMode.ANTINORMAL  -> Triple(-nx, -ny, -nz)
                    HoldMode.RADIAL      -> Triple(rx, ry, rz)
                    HoldMode.ANTIRADIAL  -> Triple(-rx, -ry, -rz)
                    else -> Triple(vux, vuy, vuz)
                }
                // Reference for body +Y: orbit normal (or radial for normal/antinormal modes)
                val (refx, refy, refz) = if (holdMode == HoldMode.NORMAL || holdMode == HoldMode.ANTINORMAL)
                    Triple(rx, ry, rz) else Triple(nx, ny, nz)
                // Gram-Schmidt body +Y
                var by0 = bx1*refz - bx2*refy; var by1 = bx2*refx - bx0*refz; var by2 = bx0*refy - bx1*refx
                val byMag = Math.sqrt(by0*by0 + by1*by1 + by2*by2)
                if (byMag > 1e-12) {
                    by0 /= byMag; by1 /= byMag; by2 /= byMag
                    // body +Z = cross(bx, by)
                    val bz0 = bx1*by2 - bx2*by1; val bz1 = bx2*by0 - bx0*by2; val bz2 = bx0*by1 - bx1*by0
                    // Passive rotation matrix rows = body axes in ECI → Shepperd method → quaternion
                    val trace = bx0 + by1 + bz2
                    val q = if (trace > 0.0) {
                        val s = 2.0 * Math.sqrt(trace + 1.0)
                        doubleArrayOf(s/4.0, (bz1-by2)/s, (bx2-bz0)/s, (by0-bx1)/s)
                    } else if (bx0 > by1 && bx0 > bz2) {
                        val s = 2.0 * Math.sqrt(1.0 + bx0 - by1 - bz2)
                        doubleArrayOf((bz1-by2)/s, s/4.0, (bx1+by0)/s, (bz0+bx2)/s)
                    } else if (by1 > bz2) {
                        val s = 2.0 * Math.sqrt(1.0 + by1 - bx0 - bz2)
                        doubleArrayOf((bx2-bz0)/s, (bx1+by0)/s, s/4.0, (by2+bz1)/s)
                    } else {
                        val s = 2.0 * Math.sqrt(1.0 + bz2 - bx0 - by1)
                        doubleArrayOf((by0-bx1)/s, (bz0+bx2)/s, (by2+bz1)/s, s/4.0)
                    }
                    sim.setX(XdX6DQ.Q0, q[0]); sim.setX(XdX6DQ.QI, q[1])
                    sim.setX(XdX6DQ.QJ, q[2]); sim.setX(XdX6DQ.QK, q[3])
                    // Re-read attitude buffer to reflect the hold override
                    sim.getAttitude(t, qBuffer)
                }
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
