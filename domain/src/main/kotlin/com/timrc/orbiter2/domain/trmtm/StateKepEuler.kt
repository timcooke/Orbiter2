package com.timrc.orbiter2.domain.trmtm

import com.timrc.orbiter2.domain.enums.EulerA
import com.timrc.orbiter2.domain.enums.KepEuler
import com.timrc.orbiter2.domain.enums.Keplerian
import com.timrc.orbiter2.domain.math.Tuple

/**
 * Faithful Kotlin port of VSE StateKepEuler.java.
 * 9-element tuple: Keplerian orbital elements + Euler attitude angles.
 */
class StateKepEuler : Tuple(KepEuler.values().size) {
    private var needToInitLabels = true

    override fun getLabels(): Array<String> {
        if (needToInitLabels) {
            needToInitLabels = false
            val names = Array(KepEuler.values().size) { ii -> KepEuler.values()[ii].toString() }
            super.setLabels(names); return names
        }
        return super.getLabels()
    }
    override fun setLabels(lbls: Array<String>?) { needToInitLabels = false; super.setLabels(lbls) }

    fun put(ndx: KepEuler, value: Double) = super.put(ndx.ordinal + 1, value)
    fun get(ndx: KepEuler): Double = super.get(ndx.ordinal + 1)

    fun set(kep: KeplerianOE, att: EulerAngles) {
        put(KepEuler.A, kep.get(Keplerian.A)); put(KepEuler.E, kep.get(Keplerian.E))
        put(KepEuler.I, kep.get(Keplerian.I)); put(KepEuler.O, kep.get(Keplerian.O))
        put(KepEuler.W, kep.get(Keplerian.W)); put(KepEuler.V, kep.get(Keplerian.V))
        put(KepEuler.BANK, att.get(EulerA.BANK)); put(KepEuler.ELEV, att.get(EulerA.ELEV))
        put(KepEuler.HEAD, att.get(EulerA.HEAD))
    }
}
