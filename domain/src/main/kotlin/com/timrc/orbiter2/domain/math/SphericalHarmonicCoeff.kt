package com.timrc.orbiter2.domain.math

/**
 * Faithful Kotlin port of VSE SphericalHarmonicCoeff.java.
 * Manages spherical harmonic coefficient sets (normalized or unnormalized).
 */
class SphericalHarmonicCoeff(
    areNormalized: Boolean,
    private val clm: Array<DoubleArray>,
    private val slm: Array<DoubleArray>
) {
    companion object {
        val MAX_NORMALIZATION = KaulaNorm.MAX_NORMALIZATION
        private const val SIN = 0
        private const val COS = 1
    }

    private val normalized: Boolean = areNormalized
    private var normalizedOut: Boolean = areNormalized
    private var converting: Boolean = false
    private val degreeOrder: Int = clm.size - 1
    private val nlmK: KaulaNorm = KaulaNorm(2 * degreeOrder)

    fun normalizedStorage(): Boolean = normalized
    fun needToConvert(): Boolean = converting

    fun setNormalizedOutput(newNormOutOpt: Boolean) {
        normalizedOut = newNormOutOpt
        converting = normalized != normalizedOut
    }

    fun cValues(): Array<DoubleArray> = cs(COS)
    fun sValues(): Array<DoubleArray> = cs(SIN)

    private fun cs(sinOrCos: Int): Array<DoubleArray> {
        val vals = if (sinOrCos == SIN) slm else clm
        var rsize = degreeOrder
        if (converting && degreeOrder > MAX_NORMALIZATION) rsize = MAX_NORMALIZATION
        val rvals = Array(rsize + 1) { DoubleArray(rsize + 1) }
        for (ll in 0..rsize) {
            for (mm in 0..ll) {
                rvals[ll][mm] = if (converting) {
                    if (normalized) nlmK.get(ll, mm) * vals[ll][mm]
                    else vals[ll][mm] / nlmK.get(ll, mm)
                } else vals[ll][mm]
            }
        }
        return rvals
    }
}
