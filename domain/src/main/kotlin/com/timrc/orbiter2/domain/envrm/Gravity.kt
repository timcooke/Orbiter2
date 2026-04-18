package com.timrc.orbiter2.domain.envrm

import com.timrc.orbiter2.domain.enums.Basis3D
import com.timrc.orbiter2.domain.math.ISpherical
import com.timrc.orbiter2.domain.math.LegendrePlmSinX
import com.timrc.orbiter2.domain.math.SphericalHarmonicCoeff
import com.timrc.orbiter2.domain.trmtm.Acceleration

/**
 * Faithful Kotlin port of VSE Gravity.java.
 * Spherical-harmonic gravitational potential and acceleration for a
 * non-spherical central body using unnormalized zonal/sectorial/tesseral
 * coefficients.  Immutable for thread safety.
 */
class Gravity(
    gmIn: Double,
    reIn: Double,
    shc: SphericalHarmonicCoeff
) : ISpherical, IGravity {

    private val degreeOrder: Int
    private val gm: Double = gmIn
    private val re: Double = reIn
    private val clm: Array<DoubleArray>
    private val slm: Array<DoubleArray>
    private val aP: LegendrePlmSinX

    init {
        shc.setNormalizedOutput(false)
        clm = shc.cValues()
        slm = shc.sValues()
        degreeOrder = clm.size - 1
        aP = LegendrePlmSinX(degreeOrder)
    }

    override fun getRefRadius(): Double = re
    override fun getGravParam(): Double = gm
    override fun getDegreeOrder(): Int = degreeOrder

    override fun getPotential(r: Double, lat: Double, lon: Double): Double =
        getPotential(degreeOrder, r, lat, lon)

    override fun getPotential(degree: Int, r: Double, lat: Double, lon: Double): Double {
        var deg = if (degree > degreeOrder) degreeOrder else degree
        val reOverR = re / r
        val gmOverR = gm / r
        var reOverR_ll = reOverR
        var pot = 0.0

        aP.set(lat)
        for (ll in 2..deg) {
            var pot_ll = 0.0
            for (mm in 0..ll) {
                val mlam = lon * mm.toDouble()
                pot_ll += aP.get(ll, mm) * (clm[ll][mm] * Math.cos(mlam) + slm[ll][mm] * Math.sin(mlam))
            }
            reOverR_ll *= reOverR
            pot_ll *= reOverR_ll
            pot += pot_ll
        }
        pot += 1.0
        pot *= gmOverR
        return pot
    }

    override fun getR(elevation: Double, azimuth: Double): Double =
        getPotential(degreeOrder, re, elevation, azimuth)

    override fun getGravityModel(): GravitationalAcceleration = GravitationalAcceleration(this)

    /**
     * Package-internal: compute gravitational acceleration into accel.
     * Called by GravitationalAcceleration.
     */
    internal fun gravt(
        degree: Int,
        r: Double, slat: Double, clat: Double, lon: Double,
        ri: Double, rj: Double, rk: Double,
        accel: Acceleration
    ) {
        val deg = if (degree > degreeOrder) degreeOrder else degree
        val reOverR = re / r
        val gmOverR = gm / r
        var reOverR_ll = reOverR
        val tanLat = slat / clat
        var dudr = 0.0
        var dudlat = 0.0
        var dudlon = 0.0

        aP.set(slat, clat)

        for (ll in 2..deg) {
            var dudr_ll = 0.0
            var dudlat_ll = 0.0
            var dudlon_ll = 0.0
            for (mm in 0..ll) {
                val mlam = lon * mm.toDouble()
                val cmlam = Math.cos(mlam)
                val smlam = Math.sin(mlam)
                val alfSinX_ll_mm = aP.get(ll, mm)
                val cssum = clm[ll][mm] * cmlam + slm[ll][mm] * smlam
                dudr_ll   += alfSinX_ll_mm * cssum
                dudlat_ll += (aP.get(ll, mm + 1) - mm * tanLat * alfSinX_ll_mm) * cssum
                dudlon_ll += mm * alfSinX_ll_mm * (slm[ll][mm] * cmlam - clm[ll][mm] * smlam)
            }
            reOverR_ll *= reOverR
            dudr_ll   *= reOverR_ll * (ll + 1).toDouble()
            dudlat_ll *= reOverR_ll
            dudlon_ll *= reOverR_ll
            dudr   += dudr_ll
            dudlat += dudlat_ll
            dudlon += dudlon_ll
        }
        dudr    = (-gmOverR / r) * (1.0 + dudr)
        dudlat *= gmOverR
        dudlon *= gmOverR

        val r2        = r * r
        val dudrOverR = dudr / r
        val ri2rj2    = ri * ri + rj * rj
        val rirj      = Math.sqrt(ri2rj2)
        val dlatTerm  = dudrOverR - rk * dudlat / (r2 * rirj)
        val dlonTerm  = dudlon / ri2rj2

        accel.put(Basis3D.I, dlatTerm * ri - dlonTerm * rj)
        accel.put(Basis3D.J, dlatTerm * rj + dlonTerm * ri)
        accel.put(Basis3D.K, dudrOverR * rk + dudlat * rirj / r2)
    }
}
