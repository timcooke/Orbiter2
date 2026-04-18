package com.timrc.orbiter2.domain.math

import com.timrc.orbiter2.domain.enums.Basis3D
import com.timrc.orbiter2.domain.enums.Q

/**
 * Faithful Kotlin port of VSE Quaternion.java.
 * scalar-first convention: q = [q0, qi, qj, qk]
 */
open class Quaternion {
    companion object {
        private val IHAT = Tuple3D(1.0, 0.0, 0.0)
        private val JHAT = Tuple3D(0.0, 1.0, 0.0)
        private val KHAT = Tuple3D(0.0, 0.0, 1.0)
        private const val TOL = 0.000001
        const val KAPPA = 0.25
    }

    protected var q0: Double = 1.0
    protected var qi: Double = 0.0
    protected var qj: Double = 0.0
    protected var qk: Double = 0.0

    constructor()
    constructor(scalar: Double, v1: Double, v2: Double, v3: Double) { q0 = scalar; qi = v1; qj = v2; qk = v3 }
    constructor(q2copy: Quaternion) { set(q2copy) }

    fun put(ndx: Q, value: Double) = when (ndx) {
        Q.Q0 -> q0 = value
        Q.QI -> qi = value
        Q.QJ -> qj = value
        Q.QK -> qk = value
    }

    fun get(ndx: Q): Double = when (ndx) {
        Q.Q0 -> q0
        Q.QI -> qi
        Q.QJ -> qj
        Q.QK -> qk
    }

    fun set(s: Double, i: Double, j: Double, k: Double) { q0 = s; qi = i; qj = j; qk = k }

    fun set(igq: Quaternion) { q0 = igq.q0; qi = igq.qi; qj = igq.qj; qk = igq.qk }

    fun set(alpha: Double, t3d: Tuple3D) {
        val a = alpha / 2.0
        q0 = Math.cos(a)
        val sa = Math.sin(a)
        qi = t3d.get(Basis3D.I) * sa
        qj = t3d.get(Basis3D.J) * sa
        qk = t3d.get(Basis3D.K) * sa
    }

    fun set(alpha: Double, axis: Basis3D) = when (axis) {
        Basis3D.I -> set(alpha, IHAT)
        Basis3D.J -> set(alpha, JHAT)
        Basis3D.K -> set(alpha, KHAT)
    }

    fun set(tin: Tuple, ndx: Int) {
        val ndxend = ndx + 3
        if (tin.N < ndxend) throw VectorSpaceIndexOutOfBoundsException("Setting Tuple: index out of bounds: ($ndxend)")
        q0 = tin.get(ndx); qi = tin.get(ndx + 1); qj = tin.get(ndx + 2); qk = tin.get(ndxend)
    }

    fun set(dcm: Matrix3X3) {
        val m = dcm.valuesPtr()
        var tmp: Double
        val d4: Double
        tmp = 1 + m[0][0] + m[1][1] + m[2][2]
        if (tmp > KAPPA) {
            tmp = Math.sqrt(tmp); q0 = 0.5 * tmp; val d = 0.5 / tmp
            qi = (m[1][2] - m[2][1]) * d; qj = (m[2][0] - m[0][2]) * d; qk = (m[0][1] - m[1][0]) * d
        } else {
            tmp = 1 + m[0][0] - m[1][1] - m[2][2]
            if (tmp > KAPPA) {
                tmp = Math.sqrt(tmp); qi = 0.5 * tmp; val d = 0.5 / tmp
                q0 = (m[1][2] - m[2][1]) * d; qj = (m[0][1] + m[1][0]) * d; qk = (m[0][2] + m[2][0]) * d
            } else {
                tmp = 1 - m[0][0] + m[1][1] - m[2][2]
                if (tmp > KAPPA) {
                    tmp = Math.sqrt(tmp); qj = 0.5 * tmp; val d = 0.5 / tmp
                    q0 = (m[2][0] - m[0][2]) * d; qi = (m[0][1] + m[1][0]) * d; qk = (m[1][2] + m[2][1]) * d
                } else {
                    tmp = 1 - m[0][0] - m[1][1] + m[2][2]
                    if (tmp > KAPPA) {
                        tmp = Math.sqrt(tmp); qk = 0.5 * tmp; val d = 0.5 / tmp
                        q0 = (m[0][1] - m[1][0]) * d; qi = (m[0][2] + m[2][0]) * d; qj = (m[1][2] + m[2][1]) * d
                    } else {
                        throw SingularQuaternionException("Can't extract Quaternion from DCM")
                    }
                }
            }
        }
    }

    fun plus(q1: Quaternion, q2: Quaternion) { q0 = q1.q0 + q2.q0; qi = q1.qi + q2.qi; qj = q1.qj + q2.qj; qk = q1.qk + q2.qk }
    fun plus(q2: Quaternion) { q0 += q2.q0; qi += q2.qi; qj += q2.qj; qk += q2.qk }
    fun minus(q1: Quaternion, q2: Quaternion) { q0 = q1.q0 - q2.q0; qi = q1.qi - q2.qi; qj = q1.qj - q2.qj; qk = q1.qk - q2.qk }

    fun mult(sc: Double) { q0 *= sc; qi *= sc; qj *= sc; qk *= sc }

    fun mult(p: Quaternion, q: Quaternion) {
        q0 = p.q0*q.q0 - p.qi*q.qi - p.qj*q.qj - p.qk*q.qk
        qi = p.q0*q.qi + p.qi*q.q0 + p.qj*q.qk - p.qk*q.qj
        qj = p.q0*q.qj - p.qi*q.qk + p.qj*q.q0 + p.qk*q.qi
        qk = p.q0*q.qk + p.qi*q.qj - p.qj*q.qi + p.qk*q.q0
    }

    fun mult(phi: Matrix) {
        val p0 = q0; val pi = qi; val pj = qj; val pk = qk
        q0 = p0*phi.get(1,1) + pi*phi.get(1,2) + pj*phi.get(1,3) + pk*phi.get(1,4)
        qi = p0*phi.get(2,1) + pi*phi.get(2,2) + pj*phi.get(2,3) + pk*phi.get(2,4)
        qj = p0*phi.get(3,1) + pi*phi.get(3,2) + pj*phi.get(3,3) + pk*phi.get(3,4)
        qk = p0*phi.get(4,1) + pi*phi.get(4,2) + pj*phi.get(4,3) + pk*phi.get(4,4)
    }

    fun conj() { qi = -qi; qj = -qj; qk = -qk }

    fun conj(quat: Quaternion) { q0 = quat.q0; qi = -quat.qi; qj = -quat.qj; qk = -quat.qk }

    fun normSQ(): Double = q0*q0 + qi*qi + qj*qj + qk*qk

    fun identity() { q0 = 1.0; qi = 0.0; qj = 0.0; qk = 0.0 }

    fun standardize() { if (q0 < 0.0) mult(-1.0) }

    fun normalize() {
        val inv = 1.0 / Math.sqrt(normSQ())
        q0 *= inv; qi *= inv; qj *= inv; qk *= inv
    }

    fun normalizeTOL() {
        val mag2 = normSQ()
        if (Math.abs(mag2 - 1.0) > TOL) {
            val inv = 1.0 / Math.sqrt(mag2)
            q0 *= inv; qi *= inv; qj *= inv; qk *= inv
        }
    }

    fun invert(quat: Quaternion) {
        val n2 = quat.normSQ()
        if (n2 != 0.0) { conj(quat); mult(1.0 / n2) }
        else throw SingularQuaternionException("Can't invert Quaternion, norm^2 = $n2")
    }

    fun angle(): Double = 2.0 * Math.acos(q0)

    fun axisAngle(eigaxis: Tuple3D): Double {
        val alpha = 2.0 * Math.acos(q0)
        val sao2 = Math.sin(alpha / 2.0)
        eigaxis.put(Basis3D.I, qi / sao2)
        eigaxis.put(Basis3D.J, qj / sao2)
        eigaxis.put(Basis3D.K, qk / sao2)
        return alpha
    }

    override fun toString(): String = "q0:  $q0  q:  $qi $qj $qk"
}
