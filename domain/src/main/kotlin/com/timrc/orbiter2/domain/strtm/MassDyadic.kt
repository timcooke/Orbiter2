package com.timrc.orbiter2.domain.strtm

import com.timrc.orbiter2.domain.enums.Basis3D
import com.timrc.orbiter2.domain.math.Matrix3X3

/**
 * Faithful Kotlin port of VSE MassDyadic.java.
 * 3x3 inertia matrix with mass, lazy-cached inverse.
 */
class MassDyadic : Matrix3X3() {
    private var mass: Double = 1.0

    private val inverse = Matrix3X3()
    private var mustRefreshInverse = true

    init {
        putJ(Basis3D.I, Basis3D.I, 1.0)
        putJ(Basis3D.J, Basis3D.J, 1.0)
        putJ(Basis3D.K, Basis3D.K, 1.0)
    }

    fun getMass(): Double = mass
    fun setMass(newM: Double) { mass = newM }

    fun putJ(row: Basis3D, col: Basis3D, value: Double) {
        mustRefreshInverse = true
        if (row == col || value == 0.0) put(row, col, value)
        else putSym(row, col, -value)
    }

    fun getJ(row: Basis3D, col: Basis3D): Double {
        val v = get(row, col)
        return if (row == col || v == 0.0) v else -1.0 * v
    }

    fun setInvRefresh() { mustRefreshInverse = true }

    fun getInv(): Matrix3X3 {
        if (mustRefreshInverse) { inverse.invert(this); mustRefreshInverse = false }
        return inverse
    }
}
