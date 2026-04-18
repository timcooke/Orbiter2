package com.timrc.orbiter2.domain.intxm

import com.timrc.orbiter2.domain.math.Tuple

/**
 * Faithful Kotlin port of VSE ISysEqns.java.
 * Interface for a system of 1st-order differential equations with
 * state, control, and output vectors plus a stepper.
 */
interface ISysEqns {
    fun getOrder(): Int
    fun getNumControls(): Int
    fun getNumOutputs(): Int
    fun getT(): Double
    fun setT(t0: Double)
    fun getX(out: Tuple): Double
    fun setX(inTuple: Tuple)
    fun getXNames(): Array<String>
    fun getY(out: Tuple)
    fun getYNames(): Array<String>
    fun getU(out: Tuple)
    fun setU(inTuple: Tuple)
    fun getUNames(): Array<String>
    fun step()
    fun step(userDelta: Double)
}
