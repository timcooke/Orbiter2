package com.timrc.orbiter2.domain.trmtm

import com.timrc.orbiter2.domain.enums.Basis3D
import com.timrc.orbiter2.domain.enums.Q
import com.timrc.orbiter2.domain.enums.XdX6DQ
import com.timrc.orbiter2.domain.math.Quaternion
import com.timrc.orbiter2.domain.math.Tuple
import com.timrc.orbiter2.domain.math.Tuple3D
import com.timrc.orbiter2.domain.math.VectorSpaceArgumentException

/**
 * Faithful Kotlin port of VSE State6DQ.java.
 * 13-element state vector: position, velocity, quaternion attitude, body rates.
 */
class State6DQ : Tuple {
    companion object { private const val DIM = 13 }

    private var t: Double = 0.0
    private var needToInitLabels = true

    constructor() : super(13) { put(XdX6DQ.Q0, 1.0) }

    constructor(v: DoubleArray) : super(v) {
        if (v.size != DIM) throw VectorSpaceArgumentException(
            "State6DQ must be initialized with an array of $DIM elements, not: ${v.size}")
    }

    fun setTime(newT: Double) { t = newT }

    fun put(ndx: XdX6DQ, value: Double) = super.put(ndx.ordinal + 1, value)
    fun get(ndx: XdX6DQ): Double = super.get(ndx.ordinal + 1)

    override fun getLabels(): Array<String> {
        if (needToInitLabels) {
            needToInitLabels = false
            val names = Array(XdX6DQ.values().size) { ii -> XdX6DQ.values()[ii].toString() }
            super.setLabels(names)
            return names
        }
        return super.getLabels()
    }

    override fun setLabels(lbls: Array<String>?) { needToInitLabels = false; super.setLabels(lbls) }

    fun setPosition(pos: Tuple3D) {
        put(XdX6DQ.X, pos.get(Basis3D.I)); put(XdX6DQ.Y, pos.get(Basis3D.J)); put(XdX6DQ.Z, pos.get(Basis3D.K))
    }

    fun getPosition(tReq: Double, pos: Tuple3D): Double {
        pos.put(Basis3D.I, get(XdX6DQ.X)); pos.put(Basis3D.J, get(XdX6DQ.Y)); pos.put(Basis3D.K, get(XdX6DQ.Z))
        return t
    }

    fun setVelocity(vel: Tuple3D) {
        put(XdX6DQ.DX, vel.get(Basis3D.I)); put(XdX6DQ.DY, vel.get(Basis3D.J)); put(XdX6DQ.DZ, vel.get(Basis3D.K))
    }

    fun getVelocity(tReq: Double, vel: Tuple3D): Double {
        vel.put(Basis3D.I, get(XdX6DQ.DX)); vel.put(Basis3D.J, get(XdX6DQ.DY)); vel.put(Basis3D.K, get(XdX6DQ.DZ))
        return t
    }

    fun setAttitude(att: Quaternion) {
        put(XdX6DQ.Q0, att.get(Q.Q0)); put(XdX6DQ.QI, att.get(Q.QI))
        put(XdX6DQ.QJ, att.get(Q.QJ)); put(XdX6DQ.QK, att.get(Q.QK))
    }

    fun getAttitude(tReq: Double, att: Quaternion): Double {
        att.put(Q.Q0, get(XdX6DQ.Q0)); att.put(Q.QI, get(XdX6DQ.QI))
        att.put(Q.QJ, get(XdX6DQ.QJ)); att.put(Q.QK, get(XdX6DQ.QK))
        return t
    }

    fun setAttitudeRate(pqr: Tuple3D) {
        put(XdX6DQ.P, pqr.get(Basis3D.I)); put(XdX6DQ.Q, pqr.get(Basis3D.J)); put(XdX6DQ.R, pqr.get(Basis3D.K))
    }

    fun getAttitudeRate(tReq: Double, pqr: Tuple3D): Double {
        pqr.put(Basis3D.I, get(XdX6DQ.P)); pqr.put(Basis3D.J, get(XdX6DQ.Q)); pqr.put(Basis3D.K, get(XdX6DQ.R))
        return t
    }
}
