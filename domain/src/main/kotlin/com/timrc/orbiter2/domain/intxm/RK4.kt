package com.timrc.orbiter2.domain.intxm

import com.timrc.orbiter2.domain.math.Tuple

/**
 * Faithful Kotlin port of VSE RK4.java.
 * Runge-Kutta 4th Order integrator for a system of 1st order ODEs.
 */
class RK4(orderIn: Int) : IIntegrator {
    private val order: Int = orderIn
    private val xd: Tuple = Tuple(order)
    private val x: Tuple = Tuple(order)
    private val xa: Tuple = Tuple(order)

    override fun step(t0: Double, delta: Double, x0: Tuple, dq: IDiffQ, xx: Tuple): Double {
        /* first */
        var tt = t0
        dq.getXDot(tt, x0, xd)
        for (ii in 1..order) {
            val v = xd.get(ii) * delta
            xa.put(ii, v)
            x.put(ii, x0.get(ii) + 0.5 * xa.get(ii))
        }

        /* second */
        val t = tt + 0.5 * delta
        dq.getXDot(t, x, xd)
        for (ii in 1..order) {
            val q = xd.get(ii) * delta
            x.put(ii, x0.get(ii) + 0.5 * q)
            xa.put(ii, xa.get(ii) + q + q)
        }

        /* third */
        dq.getXDot(t, x, xd)
        for (ii in 1..order) {
            val q = xd.get(ii) * delta
            x.put(ii, x0.get(ii) + q)
            xa.put(ii, xa.get(ii) + q + q)
        }

        /* fourth */
        tt += delta
        dq.getXDot(tt, x, xd)
        for (ii in 1..order) {
            xx.put(ii, x0.get(ii) + (xa.get(ii) + xd.get(ii) * delta) / 6.0)
        }
        return tt
    }
}
