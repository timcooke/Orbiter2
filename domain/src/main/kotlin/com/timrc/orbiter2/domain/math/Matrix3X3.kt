package com.timrc.orbiter2.domain.math

import com.timrc.orbiter2.domain.enums.Basis3D
import com.timrc.orbiter2.domain.enums.Q

/**
 * Faithful Kotlin port of VSE Matrix3X3.java.
 * 3x3 square matrix with Basis3D enum access.
 */
open class Matrix3X3 : Matrix {
    private val vals: Array<DoubleArray>

    constructor() : super(3) { vals = valuesPtr() }

    constructor(mtrx: Array<DoubleArray>) : this() { set(mtrx) }

    override fun identity() {
        zero(); vals[0][0] = 1.0; vals[1][1] = 1.0; vals[2][2] = 1.0
    }

    fun put(row: Basis3D, col: Basis3D, value: Double) { vals[row.ordinal][col.ordinal] = value }

    fun putSym(row: Basis3D, col: Basis3D, value: Double) {
        vals[row.ordinal][col.ordinal] = value
        if (row != col) vals[col.ordinal][row.ordinal] = value
    }

    fun set(inp: Matrix3X3) {
        val ip = inp.valuesPtr()
        vals[0][0] = ip[0][0]; vals[0][1] = ip[0][1]; vals[0][2] = ip[0][2]
        vals[1][0] = ip[1][0]; vals[1][1] = ip[1][1]; vals[1][2] = ip[1][2]
        vals[2][0] = ip[2][0]; vals[2][1] = ip[2][1]; vals[2][2] = ip[2][2]
    }

    fun set(q: Quaternion) {
        val q0 = q.get(Q.Q0); val qi = q.get(Q.QI); val qj = q.get(Q.QJ); val qk = q.get(Q.QK)
        val q0q0 = q0*q0; val q0qi = q0*qi; val q0qj = q0*qj; val q0qk = q0*qk
        val qiqj = qi*qj; val qiqk = qi*qk; val qjqk = qj*qk
        vals[0][0] = 2.0*(q0q0 + qi*qi) - 1.0; vals[0][1] = 2.0*(qiqj + q0qk); vals[0][2] = 2.0*(qiqk - q0qj)
        vals[1][0] = 2.0*(qiqj - q0qk); vals[1][1] = 2.0*(q0q0 + qj*qj) - 1.0; vals[1][2] = 2.0*(qjqk + q0qi)
        vals[2][0] = 2.0*(qiqk + q0qj); vals[2][1] = 2.0*(qjqk - q0qi); vals[2][2] = 2.0*(q0q0 + qk*qk) - 1.0
    }

    fun setColumns(col1: Tuple3D, col2: Tuple3D, col3: Tuple3D) {
        vals[0][0] = col1.get(1); vals[1][0] = col1.get(2); vals[2][0] = col1.get(3)
        vals[0][1] = col2.get(1); vals[1][1] = col2.get(2); vals[2][1] = col2.get(3)
        vals[0][2] = col3.get(1); vals[1][2] = col3.get(2); vals[2][2] = col3.get(3)
    }

    fun setRows(row1: Tuple3D, row2: Tuple3D, row3: Tuple3D) {
        vals[0][0] = row1.get(1); vals[0][1] = row1.get(2); vals[0][2] = row1.get(3)
        vals[1][0] = row2.get(1); vals[1][1] = row2.get(2); vals[1][2] = row2.get(3)
        vals[2][0] = row3.get(1); vals[2][1] = row3.get(2); vals[2][2] = row3.get(3)
    }

    fun get(row: Basis3D, col: Basis3D): Double = vals[row.ordinal][col.ordinal]

    override fun mult(num: Double) {
        vals[0][0] *= num; vals[0][1] *= num; vals[0][2] *= num
        vals[1][0] *= num; vals[1][1] *= num; vals[1][2] *= num
        vals[2][0] *= num; vals[2][1] *= num; vals[2][2] *= num
    }

    override fun div(num: Double) = mult(1.0 / num)

    fun invert(m: Matrix3X3) {
        val det = m.det()
        if (Math.abs(det) > EPS) {
            vals[0][0] =        m.get(2,2)*m.get(3,3) - m.get(3,2)*m.get(2,3)
            vals[1][0] = -1.0*(m.get(2,1)*m.get(3,3) - m.get(3,1)*m.get(2,3))
            vals[2][0] =        m.get(2,1)*m.get(3,2) - m.get(3,1)*m.get(2,2)
            vals[0][1] = -1.0*(m.get(1,2)*m.get(3,3) - m.get(3,2)*m.get(1,3))
            vals[1][1] =        m.get(1,1)*m.get(3,3) - m.get(3,1)*m.get(1,3)
            vals[2][1] = -1.0*(m.get(1,1)*m.get(3,2) - m.get(3,1)*m.get(1,2))
            vals[0][2] =        m.get(1,2)*m.get(2,3) - m.get(2,2)*m.get(1,3)
            vals[1][2] = -1.0*(m.get(1,1)*m.get(2,3) - m.get(2,1)*m.get(1,3))
            vals[2][2] =        m.get(1,1)*m.get(2,2) - m.get(2,1)*m.get(1,2)
            mult(1.0 / det)
        } else {
            throw SingularMatrixException("Can't invert matrix, determinant = $det")
        }
    }

    fun rotX(alpha: Double) {
        val c = Math.cos(alpha); val s = Math.sin(alpha)
        vals[0][0] = 1.0; vals[0][1] =  0.0; vals[0][2] =  0.0
        vals[1][0] = 0.0; vals[1][1] =   c;  vals[1][2] =   s
        vals[2][0] = 0.0; vals[2][1] =  -s;  vals[2][2] =   c
    }

    fun rotY(alpha: Double) {
        val c = Math.cos(alpha); val s = Math.sin(alpha)
        vals[0][0] =  c;  vals[0][1] = 0.0; vals[0][2] = -s
        vals[1][0] = 0.0; vals[1][1] = 1.0; vals[1][2] = 0.0
        vals[2][0] =  s;  vals[2][1] = 0.0; vals[2][2] =  c
    }

    fun rotZ(alpha: Double) {
        val c = Math.cos(alpha); val s = Math.sin(alpha)
        vals[0][0] =  c;  vals[0][1] = s;   vals[0][2] = 0.0
        vals[1][0] = -s;  vals[1][1] = c;   vals[1][2] = 0.0
        vals[2][0] = 0.0; vals[2][1] = 0.0; vals[2][2] = 1.0
    }
}
