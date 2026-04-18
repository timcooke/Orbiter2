package com.timrc.orbiter2.domain.envrm

/**
 * Faithful Kotlin port of VSE IGravity.java.
 * Interface for gravitational model access.
 */
interface IGravity {
    fun getGravParam(): Double
    fun getRefRadius(): Double
    fun getDegreeOrder(): Int
    fun getPotential(r: Double, elevation: Double, azimuth: Double): Double
    fun getPotential(degree: Int, r: Double, elevation: Double, azimuth: Double): Double
    fun getGravityModel(): GravitationalAcceleration
}
