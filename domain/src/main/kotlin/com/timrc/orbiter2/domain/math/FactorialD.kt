package com.timrc.orbiter2.domain.math

/**
 * Faithful Kotlin port of VSE FactorialD.java.
 * Pre-computes and stores factorials as doubles up to a specified maximum.
 */
class FactorialD(nmax: Int) {
    companion object {
        const val MAXN = 170
    }

    private val N: Int = if (nmax < 1 || nmax > MAXN) MAXN else nmax
    private val fvals: DoubleArray = DoubleArray(N + 1)

    init {
        fvals[0] = 1.0
        fvals[1] = 1.0
        for (ii in 2..N) fvals[ii] = ii * fvals[ii - 1]
    }

    fun getMaxFactorialArg(): Int = N

    fun get(x: Int): Double = if (x < 0 || x > N) 0.0 else fvals[x]
}
