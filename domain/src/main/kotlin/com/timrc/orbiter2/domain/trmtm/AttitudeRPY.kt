package com.timrc.orbiter2.domain.trmtm

import com.timrc.orbiter2.domain.enums.Keplerian
import com.timrc.orbiter2.domain.math.Matrix3X3
import com.timrc.orbiter2.domain.math.Quaternion
import com.timrc.orbiter2.domain.math.Tuple3D

/**
 * Faithful Kotlin port of VSE AttitudeRPY.java.
 * Computes inertial-to-RPY (roll/pitch/yaw) frame transformations.
 */
object AttitudeRPY {
    private val IHAT = Tuple3D(1.0, 0.0, 0.0)
    private val JHAT = Tuple3D(0.0, 1.0, 0.0)
    private val KHAT = Tuple3D(0.0, 0.0, 1.0)

    fun orbitalElem2Quat(koe: KeplerianOE, qOut: Quaternion) {
        val q2 = Quaternion(); val q12 = Quaternion()
        qOut.set(koe.get(Keplerian.O), KHAT)
        val angle = koe.get(Keplerian.I) - Math.PI / 2.0
        q2.set(angle, IHAT); q12.mult(qOut, q2)
        val angle2 = -1.0 * (koe.get(Keplerian.W) + koe.get(Keplerian.V) + Math.PI / 2.0)
        q2.set(angle2, JHAT); qOut.mult(q12, q2)
    }

    fun posVel2DCM(pos: Tuple3D, vel: Tuple3D, dcm: Matrix3X3) {
        val xHat = Tuple3D(); val yHat = Tuple3D(); val zHat = Tuple3D()
        zHat.set(pos); zHat.mult(-1.0); zHat.unitize()
        xHat.set(vel); xHat.unitize()
        yHat.cross(zHat, xHat)
        xHat.cross(yHat, zHat)
        dcm.setRows(xHat, yHat, zHat)
    }
}
