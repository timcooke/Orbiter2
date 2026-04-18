package com.timrc.orbiter2.domain

import com.timrc.orbiter2.domain.math.Matrix
import com.timrc.orbiter2.domain.math.SphereCart
import com.timrc.orbiter2.domain.math.SphericalHarmonicCoeff
import com.timrc.orbiter2.domain.envrm.Gravity
import com.timrc.orbiter2.domain.enums.Basis3D
import org.junit.Assert.assertEquals
import org.junit.Test

/**
 * Port of VSE TestGravt.java.
 * Validates the 4×4 EGM96 gravity model against known outputs.
 */
class GravityTest {

    // Tolerance for acceleration comparison (unitless ER/min² values ≈ 1e-3)
    private val TOL = 1.0e-9

    private fun buildGravity(): Gravity {
        val ngc = 5
        val clM = Matrix(ngc, ngc)
        val slM = Matrix(ngc, ngc)

        // EGM96 4×4 unnormalized coefficients
        clM.put(3, 1, -1.08262668355e-3)   // deg=2, ord=0, -J2
        clM.put(3, 2, -2.41400000000e-10)   // deg=2, ord=1
        clM.put(3, 3,  1.57446037456e-6)    // deg=2, ord=2
        clM.put(4, 1,  2.53265648533e-6)    // deg=3, ord=0
        clM.put(4, 2,  2.19263852917e-6)    // deg=3, ord=1
        clM.put(4, 3,  3.08989206881e-7)    // deg=3, ord=2
        clM.put(4, 4,  1.00548778064e-7)    // deg=3, ord=3
        clM.put(5, 1,  1.61962159137e-6)    // deg=4, ord=0
        clM.put(5, 2, -5.08799360404e-7)    // deg=4, ord=1
        clM.put(5, 3,  7.84175859844e-8)    // deg=4, ord=2
        clM.put(5, 4,  5.92099402629e-8)    // deg=4, ord=3
        clM.put(5, 5, -3.98407411766e-9)    // deg=4, ord=4

        slM.put(3, 2,  1.54310000000e-9)    // deg=2, ord=1
        slM.put(3, 3, -9.03803806639e-7)    // deg=2, ord=2
        slM.put(4, 2,  2.68424890397e-7)    // deg=3, ord=1
        slM.put(4, 3, -2.11437612437e-7)    // deg=3, ord=2
        slM.put(4, 4,  1.97222559006e-7)    // deg=3, ord=3
        slM.put(5, 2, -4.49144872839e-7)    // deg=4, ord=1
        slM.put(5, 3,  1.48177868296e-7)    // deg=4, ord=2
        slM.put(5, 4, -1.20077667634e-8)    // deg=4, ord=3
        slM.put(5, 5,  6.52571425370e-9)    // deg=4, ord=4

        val gm = 3600.0 * 398600.4418 / (6378.137 * 6378.137 * 6378.137)  // ER³/min²
        val shc = SphericalHarmonicCoeff(false, clM.values(), slM.values())
        return Gravity(gm, 1.0, shc)
    }

    @Test
    fun gravityAtDeg4Lat50Lon100() {
        val geo = buildGravity()
        val accel = geo.getGravityModel()

        val r   = 1.5
        val lat = Math.toRadians(50.0)
        val lon = Math.toRadians(-100.0)

        // Test via (r, lat, lon)
        accel.gravt(4, r, lat, lon)
        assertEquals("ax  lat=50 lon=-100 via r/lat/lon",  0.000273974088, accel.get(Basis3D.I), TOL)
        assertEquals("ay  lat=50 lon=-100 via r/lat/lon",  0.001553775992, accel.get(Basis3D.J), TOL)
        assertEquals("az  lat=50 lon=-100 via r/lat/lon", -0.001882985691, accel.get(Basis3D.K), TOL)

        // Test via SphereCart position vector
        val pos = SphereCart(); pos.setRElAz(r, lat, lon)
        accel.gravt(4, pos)
        assertEquals("ax  lat=50 lon=-100 via pos",  0.000273974088, accel.get(Basis3D.I), TOL)
        assertEquals("ay  lat=50 lon=-100 via pos",  0.001553775992, accel.get(Basis3D.J), TOL)
        assertEquals("az  lat=50 lon=-100 via pos", -0.001882985691, accel.get(Basis3D.K), TOL)
    }

    @Test
    fun gravityAtDeg4LatMinus50Lon100() {
        val geo = buildGravity()
        val accel = geo.getGravityModel()

        val r   = 1.5
        val lat = Math.toRadians(-50.0)
        val lon = Math.toRadians(100.0)

        accel.gravt(4, r, lat, lon)
        assertEquals("ax  lat=-50 lon=100 via r/lat/lon",  0.000273975775, accel.get(Basis3D.I), TOL)
        assertEquals("ay  lat=-50 lon=100 via r/lat/lon", -0.001553773172, accel.get(Basis3D.J), TOL)
        assertEquals("az  lat=-50 lon=100 via r/lat/lon",  0.001882993210, accel.get(Basis3D.K), TOL)

        val pos = SphereCart(); pos.setRElAz(r, lat, lon)
        accel.gravt(4, pos)
        assertEquals("ax  lat=-50 lon=100 via pos",  0.000273975775, accel.get(Basis3D.I), TOL)
        assertEquals("ay  lat=-50 lon=100 via pos", -0.001553773172, accel.get(Basis3D.J), TOL)
        assertEquals("az  lat=-50 lon=100 via pos",  0.001882993210, accel.get(Basis3D.K), TOL)
    }
}
