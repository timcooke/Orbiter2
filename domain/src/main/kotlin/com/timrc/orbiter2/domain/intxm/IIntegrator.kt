package com.timrc.orbiter2.domain.intxm

import com.timrc.orbiter2.domain.math.Tuple

/** Faithful Kotlin port of VSE IIntegrator.java. Numeric integration interface. */
interface IIntegrator {
    fun step(t0: Double, delta: Double, x0: Tuple, dq: IDiffQ, x: Tuple): Double
}
