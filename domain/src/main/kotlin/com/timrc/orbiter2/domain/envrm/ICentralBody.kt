package com.timrc.orbiter2.domain.envrm

import com.timrc.orbiter2.domain.math.ISpherical
import com.timrc.orbiter2.domain.math.Quaternion
import com.timrc.orbiter2.domain.math.Tuple3D

/**
 * Faithful Kotlin port of VSE ICentralBody.java.
 * Interface for a central body: gravity + spherical shape + position/attitude.
 */
interface ICentralBody : ISpherical, IGravity {
    fun getPosition(tReq: Double, tout: Tuple3D): Double
    fun getAttitude(tReq: Double, qout: Quaternion): Double
}
