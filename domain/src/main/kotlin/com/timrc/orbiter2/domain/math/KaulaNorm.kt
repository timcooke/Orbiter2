package com.timrc.orbiter2.domain.math

/**
 * Faithful Kotlin port of VSE KaulaNorm.java.
 * Computes and stores Kaula spherical harmonic normalization factors.
 */
class KaulaNorm(nmax: Int) {
    companion object {
        val MAX_NORMALIZATION = FactorialD.MAXN / 2
    }

    private val degOrder: Int = if (nmax < 0 || nmax > MAX_NORMALIZATION) MAX_NORMALIZATION else nmax
    private val nvals: Array<DoubleArray>

    init {
        val fact = FactorialD(2 * degOrder)
        nvals = Array(degOrder + 1) { DoubleArray(degOrder + 1) }
        for (ll in 0..degOrder) {
            for (mm in 0..ll) {
                val k = if (mm == 0) 1.0 else 2.0
                val fnum = fact.get(ll - mm)
                val fden = fact.get(ll + mm)
                nvals[ll][mm] = Math.sqrt(fnum * (2*ll + 1.0) * k / fden)
            }
        }
    }

    fun getDegreeOrder(): Int = degOrder

    fun get(ll: Int, mm: Int): Double {
        if (ll < 0 || ll > MAX_NORMALIZATION || mm < 0 || mm > ll) return 0.0
        return nvals[ll][mm]
    }
}
