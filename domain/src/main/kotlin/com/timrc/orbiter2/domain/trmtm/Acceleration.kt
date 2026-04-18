package com.timrc.orbiter2.domain.trmtm

import com.timrc.orbiter2.domain.enums.Basis3D
import com.timrc.orbiter2.domain.enums.DDX3D
import com.timrc.orbiter2.domain.math.Tuple3D

/**
 * Faithful Kotlin port of VSE Acceleration.java.
 * Specialization of Tuple3D for acceleration vectors.
 */
open class Acceleration : Tuple3D() {
    fun get(ndx: DDX3D): Double = get(ndx.ordinal + 1)
}
