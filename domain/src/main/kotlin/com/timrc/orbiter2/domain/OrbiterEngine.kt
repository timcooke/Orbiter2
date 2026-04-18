package com.timrc.orbiter2.domain

import com.timrc.orbiter2.domain.enums.Basis3D
import com.timrc.orbiter2.domain.enums.FTxyz
import com.timrc.orbiter2.domain.enums.KepEuler
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

    fun clearControls() { FTxyz.entries.forEach { sim.setU(it, 0.0) }  }

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
