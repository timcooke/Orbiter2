package com.timrc.orbiter2.domain.trmtm

import com.timrc.orbiter2.domain.enums.EulerA
import com.timrc.orbiter2.domain.enums.Q
import com.timrc.orbiter2.domain.math.Angles
import com.timrc.orbiter2.domain.math.Matrix3X3
import com.timrc.orbiter2.domain.math.Quaternion
import com.timrc.orbiter2.domain.math.Tuple3D
import com.timrc.orbiter2.domain.enums.Basis3D

/**
 * Faithful Kotlin port of VSE EulerAngles.java.
 * Bank, Elevation, Heading angles with aerospace 321 sequence conversions.
 */
class EulerAngles : Tuple3D() {
    private var needToInitLabels = true

    override fun getLabels(): Array<String> {
        if (needToInitLabels) {
            needToInitLabels = false
            val names = Array(EulerA.values().size) { ii -> EulerA.values()[ii].toString() }
            super.setLabels(names); return names
        }
        return super.getLabels()
    }
    override fun setLabels(lbls: Array<String>?) { needToInitLabels = false; super.setLabels(lbls) }

    fun put(ndx: EulerA, value: Double) = super.put(ndx.ordinal + 1, Angles.setPI(value))
    fun get(ndx: EulerA): Double = super.get(ndx.ordinal + 1)
    fun putDeg(ndx: EulerA, value: Double) = put(ndx, Angles.RAD_PER_DEG * value)
    fun getDeg(ndx: EulerA): Double = Angles.DEG_PER_RAD * get(ndx)

    fun toDCM(dcm: Matrix3X3): Matrix3X3 {
        val psi = get(EulerA.HEAD); val theta = get(EulerA.ELEV); val phi = get(EulerA.BANK)
        val cp = Math.cos(psi); val sp = Math.sin(psi)
        val ct = Math.cos(theta); val st = Math.sin(theta)
        val cf = Math.cos(phi); val sf = Math.sin(phi)
        dcm.put(Basis3D.I, Basis3D.I,  cp*ct);                       dcm.put(Basis3D.I, Basis3D.J,  sp*ct);                       dcm.put(Basis3D.I, Basis3D.K, -st)
        dcm.put(Basis3D.J, Basis3D.I,  cp*st*sf - sp*cf);            dcm.put(Basis3D.J, Basis3D.J,  sp*st*sf + cp*cf);            dcm.put(Basis3D.J, Basis3D.K,  ct*sf)
        dcm.put(Basis3D.K, Basis3D.I,  cp*st*cf + sp*sf);            dcm.put(Basis3D.K, Basis3D.J,  sp*st*cf - cp*sf);            dcm.put(Basis3D.K, Basis3D.K,  ct*cf)
        return dcm
    }

    fun fromDCM(dcm: Matrix3X3) { val q = Quaternion(); q.set(dcm); fromQuatFrameRot(q) }

    fun toQuatFrameRot(quat: Quaternion): Quaternion {
        val y = get(EulerA.HEAD) / 2.0; val p = get(EulerA.ELEV) / 2.0; val r = get(EulerA.BANK) / 2.0
        val sp = Math.sin(p); val sy = Math.sin(y); val sr = Math.sin(r)
        val cp = Math.cos(p); val cy = Math.cos(y); val cr = Math.cos(r)
        quat.put(Q.Q0, cr*cp*cy + sr*sp*sy)
        quat.put(Q.QI, sr*cp*cy - cr*sp*sy)
        quat.put(Q.QJ, cr*sp*cy + sr*cp*sy)
        quat.put(Q.QK, cr*cp*sy - sr*sp*cy)
        return quat
    }

    fun fromQuatFrameRot(quat: Quaternion) = fromQuatFrameRot(quat.get(Q.Q0), quat.get(Q.QI), quat.get(Q.QJ), quat.get(Q.QK))

    fun fromQuatFrameRot(q0: Double, qi: Double, qj: Double, qk: Double) {
        val m11 = 2.0*(q0*q0 + qi*qi) - 1.0; val m12 = 2.0*(qi*qj + q0*qk)
        val m13 = 2.0*(qi*qk - q0*qj);        val m23 = 2.0*(qj*qk + q0*qi)
        val m33 = 2.0*(q0*q0 + qk*qk) - 1.0
        put(EulerA.BANK, Math.atan2(m23, m33))
        put(EulerA.ELEV, Angles.setPIo2(Math.asin(-m13)))
        put(EulerA.HEAD, Math.atan2(m12, m11))
    }

    override fun toString(): String = "Bank: ${getDeg(EulerA.BANK)}  Elevation: ${getDeg(EulerA.ELEV)}  Heading: ${getDeg(EulerA.HEAD)}"
}
