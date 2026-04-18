package com.timrc.orbiter2.domain.trmtm

import com.timrc.orbiter2.domain.enums.Basis3D
import com.timrc.orbiter2.domain.enums.Q
import com.timrc.orbiter2.domain.enums.XdX6DQ
import com.timrc.orbiter2.domain.intxm.IDiffQ
import com.timrc.orbiter2.domain.math.Quaternion
import com.timrc.orbiter2.domain.math.Tuple
import com.timrc.orbiter2.domain.math.Tuple3D
import com.timrc.orbiter2.domain.strtm.MassDyadic

/**
 * Faithful Kotlin port of VSE Simple6DOFdq.java.
 * Differential equations for a 6DOF model subject to translational
 * and rotational accelerations.
 */
class Simple6DOFdq(private val system: Simple6DOFSys) : IDiffQ {

    private val ORDER: Int = system.getOrder()

    private val iC = InfluencesCache()
    private val eomC = EquationsOfMotionCache()

    override fun getOrder(): Int = ORDER

    override fun getXDot(t: Double, x_tup: Tuple, xd_tup: Tuple) {
        val jMat: MassDyadic

        // Convert to 6DOF state vectors to ease indexing
        eomC.x_6dq.set(x_tup)
        eomC.x_6dq.setTime(t)
        eomC.xd_6dq.set(xd_tup)
        eomC.xd_6dq.setTime(t)

        // Compute total body torques/forces and inertial forces & accelerations.
        // Clear values from previous call.
        iC.bodyForceCG.zero()
        iC.bodyTorqueCG.zero()
        iC.inertialForceCG.zero()
        iC.inertialFieldAccel.zero()
        system.finishModel(t, eomC.x_6dq,
            iC.bodyForceCG, iC.bodyTorqueCG,
            iC.inertialForceCG, iC.inertialFieldAccel)
        jMat = system.getJMat()
        val m = jMat.getMass()

        // Convert body forces to inertial frame (vRot = point rotation with i2b quaternion)
        eomC.x_6dq.getAttitude(t, eomC.i2bQ)
        eomC.forceBI.vRot(eomC.i2bQ, iC.bodyForceCG)
        // total force through CG
        iC.inertialForceCG.plus(eomC.forceBI)

        // Translational motion derivatives
        eomC.xd_6dq.put(XdX6DQ.X,  eomC.x_6dq.get(XdX6DQ.DX))
        eomC.xd_6dq.put(XdX6DQ.Y,  eomC.x_6dq.get(XdX6DQ.DY))
        eomC.xd_6dq.put(XdX6DQ.Z,  eomC.x_6dq.get(XdX6DQ.DZ))
        eomC.xd_6dq.put(XdX6DQ.DX, iC.inertialFieldAccel.get(Basis3D.I) + iC.inertialForceCG.get(Basis3D.I) / m)
        eomC.xd_6dq.put(XdX6DQ.DY, iC.inertialFieldAccel.get(Basis3D.J) + iC.inertialForceCG.get(Basis3D.J) / m)
        eomC.xd_6dq.put(XdX6DQ.DZ, iC.inertialFieldAccel.get(Basis3D.K) + iC.inertialForceCG.get(Basis3D.K) / m)

        // Rotational motion: extract body angular rates P, Q, R
        eomC.wBody.set(eomC.x_6dq, XdX6DQ.P.ordinal + 1)

        // Extract attitude quaternion from state, then strapdown equation
        eomC.i2bQ.set(eomC.x_6dq, XdX6DQ.Q0.ordinal + 1)
        eomC.di2bQ.set(eomC.wBody, eomC.i2bQ)

        // Quaternion attitude derivatives
        eomC.xd_6dq.put(XdX6DQ.Q0, eomC.di2bQ.get(Q.Q0))
        eomC.xd_6dq.put(XdX6DQ.QI, eomC.di2bQ.get(Q.QI))
        eomC.xd_6dq.put(XdX6DQ.QJ, eomC.di2bQ.get(Q.QJ))
        eomC.xd_6dq.put(XdX6DQ.QK, eomC.di2bQ.get(Q.QK))

        // Body angular acceleration: J*w_dot = torque - w × (J*w)
        eomC.jwb.mult(jMat, eomC.wBody)
        eomC.wjwb.cross(eomC.wBody, eomC.jwb)
        eomC.tqc.minus(iC.bodyTorqueCG, eomC.wjwb)
        eomC.wBodyDot.mult(jMat.getInv(), eomC.tqc)

        eomC.xd_6dq.put(XdX6DQ.P, eomC.wBodyDot.get(Basis3D.I))
        eomC.xd_6dq.put(XdX6DQ.Q, eomC.wBodyDot.get(Basis3D.J))
        eomC.xd_6dq.put(XdX6DQ.R, eomC.wBodyDot.get(Basis3D.K))

        // Update the output Tuple
        xd_tup.set(eomC.xd_6dq)
    }

    // Cache: composite forces, torques, accelerations acting on the body
    private inner class InfluencesCache {
        val bodyForceCG      = Tuple3D()
        val bodyTorqueCG     = Tuple3D()
        val inertialForceCG  = Tuple3D()
        val inertialFieldAccel = Tuple3D()
    }

    // Cache: intermediate computations
    private inner class EquationsOfMotionCache {
        val forceBI  = Tuple3D()
        val i2bQ     = Quaternion()
        val di2bQ    = Strapdown()
        val x_6dq    = State6DQ()
        val xd_6dq   = State6DQ()
        val wBody    = Tuple3D()
        val jwb      = Tuple3D()
        val wjwb     = Tuple3D()
        val tqc      = Tuple3D()
        val wBodyDot = Tuple3D()
    }
}
