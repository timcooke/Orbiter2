package com.timrc.orbiter2.domain.trmtm

import com.timrc.orbiter2.domain.enums.Basis3D
import com.timrc.orbiter2.domain.enums.Q
import com.timrc.orbiter2.domain.math.Quaternion
import com.timrc.orbiter2.domain.math.Tuple3D

/**
 * Faithful Kotlin port of VSE Strapdown.java.
 * Computes quaternion attitude rate from body angular rates and current attitude.
 */
class Strapdown : Quaternion(0.0, 0.0, 0.0, 0.0) {

    fun set(pqr: Tuple3D, attitude: Quaternion) {
        val p  = pqr.get(Basis3D.I)
        val q  = pqr.get(Basis3D.J)
        val r  = pqr.get(Basis3D.K)
        val q0 = attitude.get(Q.Q0)
        val q1 = attitude.get(Q.QI)
        val q2 = attitude.get(Q.QJ)
        val q3 = attitude.get(Q.QK)

        put(Q.Q0,  -0.5*( p*q1 + q*q2 + r*q3))
        put(Q.QI,  -0.5*(-p*q0 - r*q2 + q*q3))
        put(Q.QJ,  -0.5*(-q*q0 + r*q1 - p*q3))
        put(Q.QK,  -0.5*(-r*q0 - q*q1 + p*q2))
    }
}
