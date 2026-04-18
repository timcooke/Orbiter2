package com.timrc.orbiter2.ui

import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.timrc.orbiter2.domain.HoldMode
import com.timrc.orbiter2.domain.OrbiterEngine
import com.timrc.orbiter2.domain.OrbiterState
import com.timrc.orbiter2.domain.enums.FTxyz
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.launch

data class SubsystemToggles(
    val sas: Boolean = true,
    val rcs: Boolean = true,
    val autoNav: Boolean = false,
    val docking: Boolean = false,
    val lights: Boolean = true,
    val solar: Boolean = true
)

data class ControlUiState(
    val holdMode: HoldMode = HoldMode.FREE,
    val refBank: Float = 0f,
    val refElev: Float = 0f,
    val refHead: Float = 0f,
    val attGain: Float = 65f,
    val dvMps: Float = 12.5f,
    val prop: Float = 72.4f,
    val armed: Boolean = false,
    val rcsMag: Float = 0.25f,
    val toggles: SubsystemToggles = SubsystemToggles(),
    val link: Float = 38.2f,
    val latency: Float = 1.12f
)

class OrbiterViewModel : ViewModel() {

    private val engine = OrbiterEngine(stepSizeSec = 10.0, stepHz = 30L)

    val state: StateFlow<OrbiterState> = engine.state

    private val _ctrl = MutableStateFlow(ControlUiState())
    val ctrl: StateFlow<ControlUiState> = _ctrl.asStateFlow()

    init {
        engine.start(viewModelScope)
        // Simulate DSN link quality and latency fluctuation
        viewModelScope.launch {
            while (true) {
                delay(500)
                val t = System.currentTimeMillis() * 0.001
                _ctrl.value = _ctrl.value.copy(
                    link    = (38.2f + (Math.sin(t * 0.8) * 4.0)).toFloat().coerceIn(0f, 60f),
                    latency = (1.0f  + (Math.sin(t * 0.5) * 0.2)).toFloat().coerceAtLeast(0.01f)
                )
            }
        }
    }

    // ── Attitude hold ─────────────────────────────────────────────────────
    fun setHoldMode(mode: HoldMode) {
        engine.setAttitudeHold(mode)
        _ctrl.value = _ctrl.value.copy(holdMode = mode)
    }

    // ── Attitude gain ─────────────────────────────────────────────────────
    fun setAttGain(gain: Float) { _ctrl.value = _ctrl.value.copy(attGain = gain) }

    // ── Δv planner ────────────────────────────────────────────────────────
    fun setDv(dv: Float) { _ctrl.value = _ctrl.value.copy(dvMps = dv) }

    fun toggleArm() {
        _ctrl.value = _ctrl.value.copy(armed = !_ctrl.value.armed)
    }

    fun fire() {
        val c = _ctrl.value
        if (!c.armed) return
        engine.applyBurn(c.dvMps.toDouble())
        val propUsed = (c.dvMps * 0.02f).coerceAtMost(c.prop)
        _ctrl.value = c.copy(armed = false, prop = (c.prop - propUsed).coerceAtLeast(0f))
    }

    // ── RCS jog ───────────────────────────────────────────────────────────
    fun jogRCS(dx: Int, dy: Int, dz: Int) {
        if (!_ctrl.value.toggles.rcs) return
        engine.jogRCS(dx, dy, dz, _ctrl.value.rcsMag.toDouble())
        val propUsed = (_ctrl.value.rcsMag * 0.02f).coerceAtMost(_ctrl.value.prop)
        _ctrl.value = _ctrl.value.copy(prop = (_ctrl.value.prop - propUsed).coerceAtLeast(0f))
    }

    fun setRcsMag(mag: Float) { _ctrl.value = _ctrl.value.copy(rcsMag = mag) }

    // ── Subsystems ────────────────────────────────────────────────────────
    fun toggleSubsystem(key: String) {
        val t = _ctrl.value.toggles
        _ctrl.value = _ctrl.value.copy(toggles = when (key) {
            "sas"     -> t.copy(sas     = !t.sas)
            "rcs"     -> t.copy(rcs     = !t.rcs)
            "autoNav" -> t.copy(autoNav = !t.autoNav)
            "docking" -> t.copy(docking = !t.docking)
            "lights"  -> t.copy(lights  = !t.lights)
            "solar"   -> t.copy(solar   = !t.solar)
            else      -> t
        })
    }

    // ── Reset ─────────────────────────────────────────────────────────────
    fun resetSim() {
        _ctrl.value = _ctrl.value.copy(prop = 100f, armed = false, dvMps = 12.5f)
    }

    // ── Legacy thrust (keep for compatibility) ────────────────────────────
    fun thrustPrograde(on: Boolean)   { engine.setForce(FTxyz.FX, if (on) 1e-5 else 0.0) }
    fun thrustRetrograde(on: Boolean) { engine.setForce(FTxyz.FX, if (on) -1e-5 else 0.0) }
    fun clearControls() { engine.clearControls() }

    override fun onCleared() { engine.stop(); super.onCleared() }
}
