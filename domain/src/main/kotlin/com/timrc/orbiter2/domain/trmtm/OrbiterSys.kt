package com.timrc.orbiter2.domain.trmtm

import com.timrc.orbiter2.domain.enums.Basis3D
import com.timrc.orbiter2.domain.enums.FTxyz
import com.timrc.orbiter2.domain.enums.Keplerian
import com.timrc.orbiter2.domain.enums.XdX6DQ
import com.timrc.orbiter2.domain.envrm.GravitationalAcceleration
import com.timrc.orbiter2.domain.envrm.ICentralBody
import com.timrc.orbiter2.domain.math.Quaternion
import com.timrc.orbiter2.domain.math.Tuple3D

/**
 * Faithful Kotlin port of VSE OrbiterSys.java (simplified — no attitude
 * control law, no star trackers).
 * Spacecraft subject to spherical-harmonic gravity + 6-element force/torque.
 */
class OrbiterSys : Simple6DOFSys() {

    // Controls: 3 body forces + 3 body torques
    private val ft = ControlFTxyz()

    // Outputs: 6 Keplerian elements + 3 Euler angles
    private val koeEa = StateKepEuler()

    // Central body providing gravity (optional)
    private var cb: ICentralBody? = null
    private var gravityCB: GravitationalAcceleration? = null

    init {
        super.enableControls(ft)
        super.enableOutputs(koeEa)
    }

    /** Enable gravity from a central body model. */
    fun enableGravity(cbIn: ICentralBody) {
        cb = cbIn
        gravityCB = cbIn.getGravityModel()
    }

    /** Disable gravity. */
    fun disableGravity() { cb = null }

    /**
     * Set initial state from osculating Keplerian elements.
     * Has no effect if the central body has not been initialized.
     */
    fun setOE(oscOE: KeplerianOE) {
        if (cb != null) {
            val pos = Tuple3D(); val vel = Tuple3D()
            oscOE.getRV(pos, vel)
            setX(XdX6DQ.X,  pos.get(Basis3D.I))
            setX(XdX6DQ.Y,  pos.get(Basis3D.J))
            setX(XdX6DQ.Z,  pos.get(Basis3D.K))
            setX(XdX6DQ.DX, vel.get(Basis3D.I))
            setX(XdX6DQ.DY, vel.get(Basis3D.J))
            setX(XdX6DQ.DZ, vel.get(Basis3D.K))
        }
    }

    fun getU(ndx: FTxyz): Double = ft.get(ndx)
    fun setU(ndx: FTxyz, ftVal: Double) { ft.put(ndx, ftVal) }

    // finishModel() cache
    private val gvC = GravityCache()

    internal override fun finishModel(
        time: Double, state: State6DQ,
        bForce: Tuple3D, bTorque: Tuple3D,
        iForce: Tuple3D, gravityI: Tuple3D
    ) {
        // Body forces from user controls
        bForce.put(Basis3D.I, ft.get(FTxyz.FX))
        bForce.put(Basis3D.J, ft.get(FTxyz.FY))
        bForce.put(Basis3D.K, ft.get(FTxyz.FZ))

        // Body torques from user controls
        bTorque.put(Basis3D.I, ft.get(FTxyz.TX))
        bTorque.put(Basis3D.J, ft.get(FTxyz.TY))
        bTorque.put(Basis3D.K, ft.get(FTxyz.TZ))

        // Gravitational acceleration (inertial field accel)
        val localCb = cb
        if (localCb != null) {
            state.getPosition(time, gvC.r)             // model pos in I
            localCb.getPosition(time, gvC.r_cb)         // central body pos in I
            gvC.r.minus(gvC.r_cb)                       // model pos relative to CB in I
            localCb.getAttitude(time, gvC.cb_i2b)       // CB inertial-to-body attitude
            gvC.r_cb.fRot(gvC.cb_i2b, gvC.r)            // model pos in CB body frame
            gravityCB!!.gravt(gvC.r_cb)
            gravityI.vRot(gvC.cb_i2b, gravityCB!!)
        } else {
            gravityI.zero()
        }
    }

    // computeOutputs() cache
    private val qeC = QuatEulerCache()

    override fun computeOutputs() {
        val time = this.getT()
        val localCb = cb
        if (localCb != null) {
            this.getPosition(time, qeC.posI)
            this.getVelocity(time, qeC.velI)
            qeC.koe.set(localCb.getGravParam(), localCb.getRefRadius(), qeC.posI, qeC.velI)
            AttitudeRPY.orbitalElem2Quat(qeC.koe, qeC.rpy2i)  // returns I-to-RPY
            qeC.rpy2i.conj()                                    // invert: RPY-to-I
            this.getAttitude(time, qeC.i2b)
            qeC.rpy2b.mult(qeC.rpy2i, qeC.i2b)
            qeC.eaAtt.fromQuatFrameRot(qeC.rpy2b)
        } else {
            qeC.koe.zero()
            this.getPosition(time, qeC.posI)
            qeC.koe.put(Keplerian.A, qeC.posI.mag())
            qeC.eaAtt.fromQuatFrameRot(
                getX(XdX6DQ.Q0), getX(XdX6DQ.QI),
                getX(XdX6DQ.QJ), getX(XdX6DQ.QK))
        }
        koeEa.set(qeC.koe, qeC.eaAtt)
    }

    private inner class GravityCache {
        val r      = Tuple3D()
        val r_cb   = Tuple3D()
        val cb_i2b = Quaternion()
    }

    private inner class QuatEulerCache {
        val posI   = Tuple3D()
        val velI   = Tuple3D()
        val i2b    = Quaternion()
        val rpy2b  = Quaternion()
        val eaAtt  = EulerAngles()
        val koe    = KeplerianOE()
        val rpy2i  = Quaternion()
    }
}
