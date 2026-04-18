package com.timrc.orbiter2.ui

import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.timrc.orbiter2.domain.OrbiterEngine
import com.timrc.orbiter2.domain.OrbiterState
import com.timrc.orbiter2.domain.enums.FTxyz
import kotlinx.coroutines.flow.StateFlow

class OrbiterViewModel : ViewModel() {

    private val engine = OrbiterEngine(stepSizeSec = 10.0, stepHz = 30L)

    val state: StateFlow<OrbiterState> = engine.state

    init {
        engine.start(viewModelScope)
    }

    fun thrustPrograde(on: Boolean) {
        // Thrust along body +X axis (prograde in nominal attitude)
        engine.setForce(FTxyz.FX, if (on) 1e-5 else 0.0)
    }

    fun thrustRetrograde(on: Boolean) {
        engine.setForce(FTxyz.FX, if (on) -1e-5 else 0.0)
    }

    fun clearControls() { engine.clearControls() }

    override fun onCleared() {
        engine.stop()
        super.onCleared()
    }
}
