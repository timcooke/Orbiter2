package com.timrc.orbiter2.domain.trmtm

import com.timrc.orbiter2.domain.enums.Basis3D
import com.timrc.orbiter2.domain.enums.Keplerian
import com.timrc.orbiter2.domain.math.Angles
import com.timrc.orbiter2.domain.math.Quaternion
import com.timrc.orbiter2.domain.math.Tuple
import com.timrc.orbiter2.domain.math.Tuple3D

/**
 * Faithful Kotlin port of VSE KeplerianOE.java.
 * Classic orbital elements: a, e, i, Ω, ω, ν with conversions.
 */
open class KeplerianOE : Tuple {
    companion object {
        private val IHAT = Tuple3D(1.0, 0.0, 0.0)
        private val KHAT = Tuple3D(0.0, 0.0, 1.0)
        private const val TWOPI = Angles.TPI
        private const val TOL = 0.000001
    }

    private var gm: Double = 1.0
    private var needToInitLabels = true

    constructor() : super(Keplerian.values().size) { put(Keplerian.A, 1.0) }

    constructor(gmin: Double, a: Double, e: Double, i: Double, node: Double, w: Double, nu: Double) : this() {
        gm = gmin; put(Keplerian.A, a); put(Keplerian.E, e); put(Keplerian.I, i)
        put(Keplerian.O, node); put(Keplerian.W, w); put(Keplerian.V, nu)
    }

    override fun getLabels(): Array<String> {
        if (needToInitLabels) {
            needToInitLabels = false
            val names = Array(Keplerian.values().size) { ii -> Keplerian.values()[ii].toString() }
            super.setLabels(names); return names
        }
        return super.getLabels()
    }
    override fun setLabels(lbls: Array<String>?) { needToInitLabels = false; super.setLabels(lbls) }

    fun getGM(): Double = gm
    fun setGM(gmin: Double) { gm = gmin }

    fun put(ndx: Keplerian, value: Double) = super.put(ndx.ordinal + 1, value)
    fun get(ndx: Keplerian): Double = super.get(ndx.ordinal + 1)

    fun getMeanAnomaly(): Double { val ea = getEccentricAnomaly(); return ea - get(Keplerian.E)*Math.sin(ea) }

    fun getEccentricAnomaly(): Double {
        val e = get(Keplerian.E); val nu = get(Keplerian.V)
        val oep1 = 1.0 / (1.0 + e * Math.cos(nu))
        val sinE = Math.sin(nu) * Math.sqrt(1.0 - e*e) * oep1
        val cosE = (e + Math.cos(nu)) * oep1
        return Math.atan2(sinE, cosE)
    }

    fun set(gmin: Double, rgm: Double, rin: Tuple3D, vin: Tuple3D) {
        gm = gmin
        val r = Tuple3D(); val v = Tuple3D()
        r.set(rin); v.set(vin)
        if (gm != 1.0 || rgm != 1.0) { val tmp = Math.sqrt(rgm/gm); r.mult(1.0/rgm); v.mult(tmp) }
        val rmag = r.mag(); val vmag = v.mag()
        val energy = vmag*vmag/2 - 1.0/rmag
        val tmp = vmag*vmag - 1.0/rmag
        val htmp = Tuple3D(); val ntmp = Tuple3D(); val evec = Tuple3D()
        htmp.set(r); htmp.mult(tmp)
        val rdotv = r.dot(v)
        ntmp.set(v); ntmp.mult(rdotv)
        evec.minus(htmp, ntmp)
        val emag = evec.mag(); put(Keplerian.E, emag)
        if (Math.abs(emag - 1.0) > TOL) put(Keplerian.A, -0.5*rgm/energy)
        else put(Keplerian.A, rgm * htmp.dot(htmp))
        val h = Tuple3D(); h.cross(r, v); val hmag = h.mag()
        val n = Tuple3D(); n.cross(KHAT, h); val nmag = n.mag()
        put(Keplerian.I, Math.acos(h.get(Basis3D.K)/hmag))
        if (nmag < TOL) { put(Keplerian.O, 0.0) } else {
            put(Keplerian.O, Math.acos(n.get(Basis3D.I)/nmag))
            if (n.get(Basis3D.J) < 0.0) put(Keplerian.O, TWOPI - get(Keplerian.O))
        }
        if (emag < TOL || nmag < TOL) {
            if (emag < TOL) { put(Keplerian.W, 0.0) } else {
                put(Keplerian.W, Math.acos(evec.get(Basis3D.I)/emag))
                if (evec.get(Basis3D.J) < 0.0) put(Keplerian.W, TWOPI - get(Keplerian.W))
            }
        } else {
            put(Keplerian.W, Math.acos(n.dot(evec)/(nmag*emag)))
            if (evec.get(Basis3D.K) < 0.0) put(Keplerian.W, TWOPI - get(Keplerian.W))
        }
        if (emag < TOL) { put(Keplerian.V, get(Keplerian.W)) } else {
            put(Keplerian.V, Math.acos(evec.dot(r)/(emag*rmag)))
            if (rdotv < 0.0) put(Keplerian.V, TWOPI - get(Keplerian.V))
        }
    }

    fun aeGivenPeriApo(peri: Double, apo: Double) {
        put(Keplerian.A, (apo+peri)/2.0); put(Keplerian.E, (apo-peri)/(apo+peri))
    }

    fun getRV(r: Tuple3D, v: Tuple3D) {
        val e = get(Keplerian.E); val p = get(Keplerian.A)*(1.0 - e*e)
        val nu = get(Keplerian.V); val cosv = Math.cos(nu); val sinv = Math.sin(nu)
        val ecosvp1 = 1.0 + e*cosv
        val rpqw = Tuple3D(p*cosv/ecosvp1, p*sinv/ecosvp1, 0.0)
        val coef = Math.sqrt(gm/p)
        val vpqw = Tuple3D(-coef*sinv, coef*(e + cosv), 0.0)
        val q = Quaternion(); val q2 = Quaternion(); val q12 = Quaternion()
        q.set(-get(Keplerian.W), KHAT); q2.set(-get(Keplerian.I), IHAT)
        q12.mult(q, q2); q2.set(-get(Keplerian.O), KHAT); q.mult(q12, q2)
        r.fRot(q, rpqw); v.fRot(q, vpqw)
    }
}
