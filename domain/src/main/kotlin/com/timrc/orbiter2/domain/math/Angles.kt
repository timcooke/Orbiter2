package com.timrc.orbiter2.domain.math

object Angles {
    const val PIO2: Double = Math.PI / 2.0
    const val PI: Double = Math.PI
    const val TPI: Double = Math.PI * 2.0
    const val RAD_PER_DEG: Double = Math.PI / 180.0
    const val DEG_PER_RAD: Double = 1.0 / RAD_PER_DEG
    const val ARCSEC_PER_DEG: Double = 3600.0
    const val ARCSEC_PER_RAD: Double = ARCSEC_PER_DEG * DEG_PER_RAD
    const val DEG_PER_HOUR: Double = 15.0
    const val RAD_PER_HOUR: Double = DEG_PER_HOUR * RAD_PER_DEG
    const val RAD_PER_SEC: Double = RAD_PER_HOUR / 3600.0

    fun setPIo2(ang: Double): Double {
        var a = ang
        if (a > PIO2) {
            while (a > PIO2) a -= PI
        } else if (a < -PIO2) {
            while (a < -PIO2) a += PI
        }
        return a
    }

    fun setPI(ang: Double): Double {
        var a = ang
        if (a > PI) {
            while (a > PI) a -= TPI
        } else if (a < -PI) {
            while (a < -PI) a += TPI
        }
        return a
    }

    fun set2PI(ang: Double): Double {
        var a = ang
        if (a > TPI) {
            while (a > TPI) a -= TPI
        } else if (a < -TPI) {
            while (a < -TPI) a += TPI
        }
        return a
    }
}
