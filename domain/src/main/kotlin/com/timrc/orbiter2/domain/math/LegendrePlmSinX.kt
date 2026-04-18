package com.timrc.orbiter2.domain.math

/**
 * Faithful Kotlin port of VSE LegendrePlmSinX.java.
 * Computes associated Legendre functions of sin(x), P[l][m](sin(x)).
 */
class LegendrePlmSinX(degree_order: Int) {
    private val N: Int = if (degree_order > 1) degree_order else 1
    private val alf: Array<DoubleArray> = Array(N + 1) { DoubleArray(N + 1) }

    fun getMaxDegree(): Int = N
    fun getMaxOrder(): Int = N

    fun set(x: Double) = set(Math.sin(x), Math.cos(x))

    fun set(sx: Double, cx: Double) {
        alf[0][0] = 1.0
        alf[1][0] = sx
        alf[1][1] = cx
        for (ll in 2..N) {
            for (mm in 0..N) {
                alf[ll][mm] = when {
                    ll == mm -> (2.0*ll - 1.0)*cx*alf[ll-1][ll-1]
                    mm < ll && mm != 0 -> alf[ll-2][mm] + (2.0*ll - 1.0)*cx*alf[ll-1][mm-1]
                    else -> ((2.0*ll - 1.0)*sx*alf[ll-1][0] - (ll - 1.0)*alf[ll-2][0]) / ll.toDouble()
                }
            }
        }
    }

    fun get(degree: Int, order: Int): Double {
        if (order < 0 || order > degree || degree > N) return 0.0
        return alf[degree][order]
    }

    companion object {
        fun alfSinX(x: Double, ll: Int, mm: Int): Double {
            val cx = Math.cos(x); val sx = Math.sin(x)
            return when (ll) {
                0 -> if (mm == 0) 1.0 else 0.0
                1 -> when (mm) { 0 -> sx; 1 -> cx; else -> 0.0 }
                2 -> when (mm) { 0 -> 0.5*(3.0*sx*sx - 1.0); 1 -> 3.0*sx*cx; 2 -> 3.0*cx*cx; else -> 0.0 }
                3 -> when (mm) { 0 -> 0.5*sx*(5.0*sx*sx-3.0); 1 -> 0.5*cx*(15.0*sx*sx-3.0); 2 -> 15.0*cx*cx*sx; 3 -> 15.0*cx*cx*cx; else -> 0.0 }
                4 -> when (mm) { 0 -> (35.0*sx*sx*sx*sx - 30.0*sx*sx + 3.0)/8.0; 1 -> 2.5*cx*sx*(7.0*sx*sx-3.0); 2 -> 7.5*cx*cx*(7.0*sx*sx-1.0); 3 -> 105.0*cx*cx*cx*sx; 4 -> 105.0*cx*cx*cx*cx; else -> 0.0 }
                else -> 0.0
            }
        }
    }
}
