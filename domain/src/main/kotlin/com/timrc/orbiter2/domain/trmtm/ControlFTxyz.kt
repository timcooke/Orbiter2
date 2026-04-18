package com.timrc.orbiter2.domain.trmtm

import com.timrc.orbiter2.domain.enums.FTxyz
import com.timrc.orbiter2.domain.math.Tuple
import com.timrc.orbiter2.domain.math.VectorSpaceArgumentException

/**
 * Faithful Kotlin port of VSE ControlFTxyz.java.
 * 6-element control vector: forces and torques in x, y, z.
 */
class ControlFTxyz : Tuple {
    companion object { private const val DIM = 6 }
    private var needToInitLabels = true

    constructor() : super(6)
    constructor(v: DoubleArray) : super(v) {
        if (v.size != DIM) throw VectorSpaceArgumentException(
            "ControlFTxyz must be initialized with an array of $DIM elements, not: ${v.size}")
    }

    override fun getLabels(): Array<String> {
        if (needToInitLabels) {
            needToInitLabels = false
            val names = Array(FTxyz.values().size) { ii -> FTxyz.values()[ii].toString() }
            super.setLabels(names); return names
        }
        return super.getLabels()
    }

    override fun setLabels(lbls: Array<String>?) { needToInitLabels = false; super.setLabels(lbls) }

    fun put(ndx: FTxyz, value: Double) = super.put(ndx.ordinal + 1, value)
    fun get(ndx: FTxyz): Double = super.get(ndx.ordinal + 1)
}
