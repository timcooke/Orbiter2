package com.timrc.orbiter2.ui

import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.interaction.MutableInteractionSource
import androidx.compose.foundation.interaction.collectIsPressedAsState
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.ColumnScope
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxHeight
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.Button
import androidx.compose.material3.ButtonDefaults
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.remember
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.lifecycle.compose.collectAsStateWithLifecycle
import androidx.lifecycle.viewmodel.compose.viewModel
import com.timrc.orbiter2.domain.OrbiterState
import kotlin.math.abs

// ── Palette ──────────────────────────────────────────────────────────────────
private val BgColor      = Color(0xFF050A14)
private val PanelColor   = Color(0x991A2236)
private val BorderColor  = Color(0xFF2A4070)
private val LabelColor   = Color(0xFF5B7FBB)
private val ValueColor   = Color(0xFFB8D4FF)
private val AccentGreen  = Color(0xFF3DFF8F)
private val AccentRed    = Color(0xFFFF4D4D)
private val AccentYellow = Color(0xFFFFD166)

@Composable
fun OrbiterScreen(vm: OrbiterViewModel = viewModel()) {
    val state by vm.state.collectAsStateWithLifecycle()

    Box(
        modifier = Modifier
            .fillMaxSize()
            .background(BgColor)
    ) {
        Column(modifier = Modifier.fillMaxSize()) {
            // ── Top bar ──────────────────────────────────────────────────
            TopBar(state)

            // ── Main body ────────────────────────────────────────────────
            Row(
                modifier = Modifier
                    .fillMaxSize()
                    .padding(horizontal = 8.dp, vertical = 4.dp),
                horizontalArrangement = Arrangement.spacedBy(8.dp)
            ) {
                // Left panel: position + orbital elements
                Column(
                    modifier = Modifier
                        .width(200.dp)
                        .fillMaxHeight(),
                    verticalArrangement = Arrangement.spacedBy(8.dp)
                ) {
                    PositionPanel(state)
                    OrbitalElementsPanel(state)
                }

                // Centre: live 3D scene
                OrbiterSceneView(
                    state = state,
                    modifier = Modifier
                        .weight(1f)
                        .fillMaxHeight()
                )

                // Right panel: attitude + controls
                Column(
                    modifier = Modifier
                        .width(180.dp)
                        .fillMaxHeight(),
                    verticalArrangement = Arrangement.spacedBy(8.dp)
                ) {
                    AttitudePanel(state)
                    ControlsPanel(vm)
                }
            }
        }
    }
}

// ── Top bar ──────────────────────────────────────────────────────────────────

@Composable
private fun TopBar(state: OrbiterState) {
    Row(
        modifier = Modifier
            .fillMaxWidth()
            .background(PanelColor)
            .padding(horizontal = 16.dp, vertical = 6.dp),
        horizontalArrangement = Arrangement.SpaceBetween,
        verticalAlignment = Alignment.CenterVertically
    ) {
        HudLabel("T+", formatTime(state.time))
        HudLabel("ALT", "${state.altitudeKm.fmt(1)} km")
        HudLabel("SPD", "${state.speedKms.fmt(3)} km/s")
        HudLabel("|r|", "${state.posMagEr.fmt(5)} ER")
    }
}

// ── Position panel ────────────────────────────────────────────────────────────

@Composable
private fun PositionPanel(state: OrbiterState) {
    HudPanel("POSITION  (ER)") {
        HudRow("X", state.posX.fmt(5))
        HudRow("Y", state.posY.fmt(5))
        HudRow("Z", state.posZ.fmt(5))
        Spacer(Modifier.height(4.dp))
        HudRow("Vx", state.velX.fmt(5))
        HudRow("Vy", state.velY.fmt(5))
        HudRow("Vz", state.velZ.fmt(5))
    }
}

// ── Orbital elements panel ────────────────────────────────────────────────────

@Composable
private fun OrbitalElementsPanel(state: OrbiterState) {
    HudPanel("ORBITAL ELEMENTS") {
        HudRow("a",  "${state.semiMajorAxisEr.fmt(5)} ER")
        HudRow("e",  state.eccentricity.fmt(6))
        HudRow("i",  "${state.inclinationDeg.fmt(2)}°")
        HudRow("Ω",  "${state.raanDeg.fmt(2)}°")
        HudRow("ω",  "${state.aopDeg.fmt(2)}°")
        HudRow("ν",  "${state.trueAnomalyDeg.fmt(2)}°")
    }
}

// ── Attitude panel ────────────────────────────────────────────────────────────

@Composable
private fun AttitudePanel(state: OrbiterState) {
    HudPanel("ATTITUDE  (RPY)") {
        HudRow("Bank", "${state.bankDeg.fmt(2)}°")
        HudRow("Elev", "${state.elevationDeg.fmt(2)}°")
        HudRow("Head", "${state.headingDeg.fmt(2)}°")
        Spacer(Modifier.height(4.dp))
        HudRow("q0", state.q0.fmt(5))
        HudRow("qi", state.qi.fmt(5))
        HudRow("qj", state.qj.fmt(5))
        HudRow("qk", state.qk.fmt(5))
    }
}

// ── Controls panel ────────────────────────────────────────────────────────────

@Composable
private fun ControlsPanel(vm: OrbiterViewModel) {
    HudPanel("CONTROLS") {
        ThrustButton(
            label = "▲  PROGRADE",
            color = AccentGreen,
            onPress = { vm.thrustPrograde(true) },
            onRelease = { vm.thrustPrograde(false) }
        )
        Spacer(Modifier.height(6.dp))
        ThrustButton(
            label = "▼  RETROGRADE",
            color = AccentRed,
            onPress = { vm.thrustRetrograde(true) },
            onRelease = { vm.thrustRetrograde(false) }
        )
    }
}

@Composable
private fun ThrustButton(
    label: String,
    color: Color,
    onPress: () -> Unit,
    onRelease: () -> Unit
) {
    val interactionSource = remember { MutableInteractionSource() }
    val pressed by interactionSource.collectIsPressedAsState()

    LaunchedEffect(pressed) {
        if (pressed) onPress() else onRelease()
    }

    Button(
        onClick = {},
        interactionSource = interactionSource,
        modifier = Modifier.fillMaxWidth(),
        colors = ButtonDefaults.buttonColors(
            containerColor = if (pressed) color else color.copy(alpha = 0.25f),
            contentColor = if (pressed) BgColor else color
        ),
        shape = RoundedCornerShape(4.dp)
    ) {
        Text(label, fontFamily = FontFamily.Monospace, fontSize = 11.sp)
    }
}

// ── Shared HUD primitives ─────────────────────────────────────────────────────

@Composable
private fun HudPanel(title: String, content: @Composable ColumnScope.() -> Unit) {
    Column(
        modifier = Modifier
            .fillMaxWidth()
            .background(PanelColor, RoundedCornerShape(4.dp))
            .border(1.dp, BorderColor, RoundedCornerShape(4.dp))
            .padding(horizontal = 10.dp, vertical = 8.dp)
    ) {
        Text(
            title,
            color = LabelColor,
            fontSize = 9.sp,
            fontWeight = FontWeight.Bold,
            fontFamily = FontFamily.Monospace,
            letterSpacing = 1.5.sp
        )
        Spacer(Modifier.height(6.dp))
        content()
    }
}

@Composable
private fun HudRow(label: String, value: String) {
    Row(
        modifier = Modifier
            .fillMaxWidth()
            .padding(vertical = 1.dp),
        horizontalArrangement = Arrangement.SpaceBetween
    ) {
        Text(label, color = LabelColor,  fontSize = 11.sp, fontFamily = FontFamily.Monospace)
        Text(value, color = ValueColor,  fontSize = 11.sp, fontFamily = FontFamily.Monospace)
    }
}

@Composable
private fun HudLabel(label: String, value: String) {
    Row(verticalAlignment = Alignment.CenterVertically) {
        Text("$label  ", color = LabelColor,   fontSize = 11.sp, fontFamily = FontFamily.Monospace)
        Text(value,      color = AccentYellow, fontSize = 12.sp, fontFamily = FontFamily.Monospace,
            fontWeight = FontWeight.Bold)
    }
}

// ── Formatting helpers ────────────────────────────────────────────────────────

private fun Double.fmt(decimals: Int): String {
    val sign = if (this < 0.0) "-" else " "
    return "$sign${String.format("%.${decimals}f", abs(this))}"
}

private fun formatTime(minutes: Double): String {
    val totalSec = (minutes * 60.0).toLong()
    val h = totalSec / 3600
    val m = (totalSec % 3600) / 60
    val s = totalSec % 60
    return "%02d:%02d:%02d".format(h, m, s)
}
