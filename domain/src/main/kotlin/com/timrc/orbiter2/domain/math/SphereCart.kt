package com.timrc.orbiter2.domain.math

import com.timrc.orbiter2.domain.enums.Basis3D

/**
 * Faithful Kotlin port of VSE SphereCart.java.
 * Sphere model: sets radius/elevation/azimuth, auto-updates Cartesian components.
 */
class SphereCart : Tuple3D() {
    private var r = 0.0
    private var el = 0.0
    private var az = 0.0
    private var cel = 1.0
    private var sel = 0.0

    fun setRElAz(radius: Double, elevation: Double, azimuth: Double) {
        r = radius; el = elevation; cel = Math.cos(el); sel = Math.sin(el); az = azimuth; update()
    }

    fun setElevation(elevation: Double) {
        el = elevation; cel = Math.cos(el); sel = Math.sin(el); update()
    }

    fun setRadiusAzimuth(radius: Double, azimuth: Double) {
        r = radius; az = azimuth; update()
    }

    fun getR(elevation: Double, azimuth: Double): Double = r

    private fun update() {
        put(Basis3D.I, r * cel * Math.cos(az))
        put(Basis3D.J, r * cel * Math.sin(az))
        put(Basis3D.K, r * sel)
    }
}
