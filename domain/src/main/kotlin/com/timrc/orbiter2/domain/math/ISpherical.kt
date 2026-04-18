package com.timrc.orbiter2.domain.math

/**
 * Faithful Kotlin port of VSE ISpherical.java.
 * Returns radius/range as a function of elevation and azimuth.
 */
interface ISpherical {
    fun getR(elevation: Double, azimuth: Double): Double
}
