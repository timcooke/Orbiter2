package com.timrc.orbiter2.domain.trmtm

import com.timrc.orbiter2.domain.math.Quaternion
import com.timrc.orbiter2.domain.math.Tuple3D

/**
 * Faithful Kotlin port of VSE IPosVelAtt.java.
 * Interface for accessing current position, velocity, and attitude.
 */
interface IPosVelAtt {
    fun getPosition(tReq: Double, tout: Tuple3D): Double
    fun getVelocity(tReq: Double, tout: Tuple3D): Double
    fun getAttitude(tReq: Double, qout: Quaternion): Double
}
