package com.timrc.orbiter2.domain.envrm

import com.timrc.orbiter2.domain.enums.Basis3D
import com.timrc.orbiter2.domain.math.Tuple3D
import com.timrc.orbiter2.domain.trmtm.Acceleration

/**
 * Faithful Kotlin port of VSE GravitationalAcceleration.java.
 * Extends Acceleration; delegates to Gravity for spherical-harmonic model.
 */
class GravitationalAcceleration(private val gravityModel: Gravity) : Acceleration() {

    private val degreeOrder: Int = gravityModel.getDegreeOrder()

    fun gravt(r: Double, lat: Double, lon: Double) = gravt(degreeOrder, r, lat, lon)

    fun gravt(degree: Int, r: Double, lat: Double, lon: Double) {
        val slat = Math.sin(lat)
        val clat = Math.cos(lat)
        gravityModel.gravt(degree, r, slat, clat, lon,
            r * clat * Math.cos(lon),
            r * clat * Math.sin(lon),
            r * slat,
            this)
    }

    fun gravt(pos: Tuple3D) = gravt(degreeOrder, pos)

    fun gravt(degree: Int, pos: Tuple3D) {
        val rx = pos.get(Basis3D.I)
        val ry = pos.get(Basis3D.J)
        val rz = pos.get(Basis3D.K)
        val rp2 = rx * rx + ry * ry
        val r   = Math.sqrt(rp2 + rz * rz)
        val slat = rz / r
        val clat = Math.sqrt(rp2) / r
        val lon  = Math.atan2(ry, rx)
        gravityModel.gravt(degree, r, slat, clat, lon, rx, ry, rz, this)
    }
}
