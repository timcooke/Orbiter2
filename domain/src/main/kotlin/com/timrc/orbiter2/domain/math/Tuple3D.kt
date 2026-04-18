package com.timrc.orbiter2.domain.math

import com.timrc.orbiter2.domain.enums.Basis3D
import com.timrc.orbiter2.domain.enums.Q

/**
 * Faithful Kotlin port of VSE Tuple3D.java.
 * 3-element vector with Basis3D enum and integer (1-based) access.
 */
open class Tuple3D : Tuple {

    constructor() : super(3)

    constructor(x: Double, y: Double, z: Double) : super(3) {
        vals[0] = x; vals[1] = y; vals[2] = z
    }

    constructor(t3d: DoubleArray) : super(3) {
        if (t3d.size != 3) throw VectorSpaceArgumentException(
            "Tuple3D must be initialized with an array of 3 elements, not: ${t3d.size}")
        t3d.copyInto(vals)
    }

    fun put(ndx: Basis3D, value: Double) { vals[ndx.ordinal] = value }

    fun set(i: Double, j: Double, k: Double) { vals[0] = i; vals[1] = j; vals[2] = k }

    fun set(tin: Tuple, ndx: Int) {
        val ndxend = ndx + 2
        if (tin.N < ndxend) throw VectorSpaceIndexOutOfBoundsException(
            "Setting Tuple: index out of bounds: ($ndxend)")
        put(1, tin.get(ndx)); put(2, tin.get(ndx + 1)); put(3, tin.get(ndxend))
    }

    fun set(t3d: Tuple3D) { super.set(t3d) }

    fun get(ndx: Basis3D): Double = vals[ndx.ordinal]

    fun plus(a: Tuple3D, b: Tuple3D) {
        val ap = a.valuesPtr(); val bp = b.valuesPtr()
        vals[0] = ap[0] + bp[0]; vals[1] = ap[1] + bp[1]; vals[2] = ap[2] + bp[2]
    }

    fun plus(a: Tuple3D) {
        val ap = a.valuesPtr()
        vals[0] += ap[0]; vals[1] += ap[1]; vals[2] += ap[2]
    }

    fun minus(a: Tuple3D, b: Tuple3D) {
        val ap = a.valuesPtr(); val bp = b.valuesPtr()
        vals[0] = ap[0] - bp[0]; vals[1] = ap[1] - bp[1]; vals[2] = ap[2] - bp[2]
    }

    fun minus(a: Tuple3D) {
        val ap = a.valuesPtr()
        vals[0] -= ap[0]; vals[1] -= ap[1]; vals[2] -= ap[2]
    }

    fun cross(a: Tuple3D, b: Tuple3D) {
        val ap = a.valuesPtr(); val bp = b.valuesPtr()
        vals[0] = ap[1]*bp[2] - ap[2]*bp[1]
        vals[1] = ap[2]*bp[0] - ap[0]*bp[2]
        vals[2] = ap[0]*bp[1] - ap[1]*bp[0]
    }

    /** qvq* — point rotation */
    fun vRot(q: Quaternion, r: Tuple3D) {
        val q0 = q.get(Q.Q0); val qi = q.get(Q.QI); val qj = q.get(Q.QJ); val qk = q.get(Q.QK)
        val q0q0 = q0*q0; val q0qi = q0*qi; val q0qj = q0*qj; val q0qk = q0*qk
        val qiqj = qi*qj; val qiqk = qi*qk; val qjqk = qj*qk
        val q11 = 2.0*(q0q0 + qi*qi) - 1.0; val q21 = 2.0*(qiqj + q0qk); val q31 = 2.0*(qiqk - q0qj)
        val q12 = 2.0*(qiqj - q0qk); val q22 = 2.0*(q0q0 + qj*qj) - 1.0; val q32 = 2.0*(qjqk + q0qi)
        val q13 = 2.0*(qiqk + q0qj); val q23 = 2.0*(qjqk - q0qi); val q33 = 2.0*(q0q0 + qk*qk) - 1.0
        val iv = r.valuesPtr()
        vals[0] = q11*iv[0] + q12*iv[1] + q13*iv[2]
        vals[1] = q21*iv[0] + q22*iv[1] + q23*iv[2]
        vals[2] = q31*iv[0] + q32*iv[1] + q33*iv[2]
    }

    /** q*vq — reference frame rotation */
    fun fRot(q: Quaternion, r: Tuple3D) {
        val q0 = q.get(Q.Q0); val qi = q.get(Q.QI); val qj = q.get(Q.QJ); val qk = q.get(Q.QK)
        val q0q0 = q0*q0; val q0qi = q0*qi; val q0qj = q0*qj; val q0qk = q0*qk
        val qiqj = qi*qj; val qiqk = qi*qk; val qjqk = qj*qk
        val q11 = 2.0*(q0q0 + qi*qi) - 1.0; val q21 = 2.0*(qiqj + q0qk); val q31 = 2.0*(qiqk - q0qj)
        val q12 = 2.0*(qiqj - q0qk); val q22 = 2.0*(q0q0 + qj*qj) - 1.0; val q32 = 2.0*(qjqk + q0qi)
        val q13 = 2.0*(qiqk + q0qj); val q23 = 2.0*(qjqk - q0qi); val q33 = 2.0*(q0q0 + qk*qk) - 1.0
        val iv = r.valuesPtr()
        vals[0] = q11*iv[0] + q21*iv[1] + q31*iv[2]
        vals[1] = q12*iv[0] + q22*iv[1] + q32*iv[2]
        vals[2] = q13*iv[0] + q23*iv[1] + q33*iv[2]
    }

    override fun mult(s: Double) { vals[0] *= s; vals[1] *= s; vals[2] *= s }

    override fun div(s: Double) = mult(1.0 / s)

    fun dot(tup: Tuple3D): Double {
        val tp = tup.valuesPtr()
        return vals[0]*tp[0] + vals[1]*tp[1] + vals[2]*tp[2]
    }

    override fun mag(): Double = Math.sqrt(dot(this))

    override fun unitize() = mult(1.0 / mag())

    override fun toString(): String = "x y z:  ${get(1)} ${get(2)} ${get(3)}"
}
