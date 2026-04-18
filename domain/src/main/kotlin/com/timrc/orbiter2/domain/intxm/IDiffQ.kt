package com.timrc.orbiter2.domain.intxm

import com.timrc.orbiter2.domain.math.Tuple

/** Faithful Kotlin port of VSE IDiffQ.java. System of 1st order differential equations. */
interface IDiffQ {
    fun getOrder(): Int
    fun getXDot(t: Double, x: Tuple, xd: Tuple)
}
