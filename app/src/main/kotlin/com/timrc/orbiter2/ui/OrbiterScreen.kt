package com.timrc.orbiter2.ui

import androidx.compose.animation.core.InfiniteRepeatableSpec
import androidx.compose.animation.core.RepeatMode
import androidx.compose.animation.core.animateFloat
import androidx.compose.animation.core.infiniteRepeatable
import androidx.compose.animation.core.rememberInfiniteTransition
import androidx.compose.animation.core.tween
import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.ColumnScope
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.WindowInsets
import androidx.compose.foundation.layout.fillMaxHeight
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.safeDrawing
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.layout.windowInsetsPadding
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.Slider
import androidx.compose.material3.SliderDefaults
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.graphics.Brush
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.text.style.TextOverflow
import androidx.compose.ui.unit.Dp
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.lifecycle.compose.collectAsStateWithLifecycle
import androidx.lifecycle.viewmodel.compose.viewModel
import com.timrc.orbiter2.domain.HoldMode
import com.timrc.orbiter2.domain.OrbiterState
import kotlinx.coroutines.delay
import kotlin.math.abs
import kotlin.math.min

// ── Design tokens ──────────────────────────────────────────────────────────────
private val BgColor      = Color(0xFF05060F)
private val PanelColor   = Color(0xFF13213D)
private val PanelColor2  = Color(0xFF0F1A30)
private val PanelBorder  = Color(0xFF2A3A5A)
private val PanelBorder2 = Color(0xFF1A2A46)
private val InkColor     = Color(0xFFC8D4EA)
private val InkDim       = Color(0xFF6E7FA0)
private val InkHi        = Color(0xFFFFFFFF)
private val Amber        = Color(0xFFE8A947)
private val AccGreen     = Color(0xFFB7E24F)
private val AccRed       = Color(0xFFFF6A3D)
private val StatusGreen  = Color(0xFF5CD07B)
private val MONO         = FontFamily.Monospace
private val PANEL_SHAPE  = RoundedCornerShape(2.dp)

// ── Formatting helpers ─────────────────────────────────────────────────────────
private fun Float.fmt1(): String = "%.1f".format(this)
private fun Float.fmt2(): String = "%.2f".format(this)
private fun Double.fmt(decimals: Int): String = (if (this < 0) "-" else " ") + "%.${decimals}f".format(abs(this))
private fun Double.fmtDeg(decimals: Int): String = "${"%.${decimals}f".format(this)}°"
private fun formatMET(minutes: Double): String {
    val s = (minutes * 60.0).toLong()
    return "%02d:%02d:%02d".format(s / 3600, (s % 3600) / 60, s % 60)
}

// ── Root screen ────────────────────────────────────────────────────────────────
@Composable
fun OrbiterScreen(vm: OrbiterViewModel = viewModel()) {
    val state by vm.state.collectAsStateWithLifecycle()
    val ctrl  by vm.ctrl.collectAsStateWithLifecycle()

    Column(
        modifier = Modifier
            .fillMaxSize()
            .background(BgColor)
            .windowInsetsPadding(WindowInsets.safeDrawing)
    ) {
        MissionTopBar(state)
        Row(
            modifier = Modifier
                .fillMaxSize()
                .padding(8.dp),
            horizontalArrangement = Arrangement.spacedBy(8.dp)
        ) {
            LeftColumn(
                state = state,
                modifier = Modifier
                    .fillMaxHeight()
                    .width(180.dp)
            )
            OrbiterSceneView(
                state    = state,
                modifier = Modifier
                    .weight(1f)
                    .fillMaxHeight()
            )
            RightColumn(
                ctrl = ctrl,
                state = state,
                vm = vm,
                modifier = Modifier
                    .fillMaxHeight()
                    .width(220.dp)
            )
        }
    }
}

// ── MissionTopBar ──────────────────────────────────────────────────────────────
@Composable
private fun MissionTopBar(state: OrbiterState) {
    Row(
        modifier = Modifier
            .fillMaxWidth()
            .height(34.dp)
            .background(BgColor.copy(alpha = 0.80f))
            .border(
                width = 1.dp,
                color = PanelBorder2,
                shape = RoundedCornerShape(bottomStart = 0.dp, bottomEnd = 0.dp)
            )
            .padding(horizontal = 12.dp),
        verticalAlignment = Alignment.CenterVertically,
        horizontalArrangement = Arrangement.SpaceBetween
    ) {
        TopBarCell(key = "T+", value = formatMET(state.time))
        TopBarCell(key = "ALT", value = "${"%.1f".format(state.altitudeKm)} km")
        TopBarCell(key = "SPD", value = "${"%.3f".format(state.speedKms)} km/s")
        TopBarCell(key = "|r|", value = "${"%.5f".format(state.posMagEr)} ER")
        TopBarCell(key = "MODE", value = "NOMINAL", valueColor = StatusGreen)
    }
}

@Composable
private fun TopBarCell(key: String, value: String, valueColor: Color = Amber) {
    Row(
        verticalAlignment = Alignment.CenterVertically,
        horizontalArrangement = Arrangement.spacedBy(4.dp)
    ) {
        Text(
            text = key,
            color = InkDim,
            fontSize = 9.sp,
            fontFamily = MONO,
            letterSpacing = 0.8.sp
        )
        Text(
            text = value,
            color = valueColor,
            fontSize = 11.sp,
            fontFamily = MONO,
            fontWeight = FontWeight.Bold
        )
    }
}

// ── LeftColumn ─────────────────────────────────────────────────────────────────
@Composable
private fun LeftColumn(state: OrbiterState, modifier: Modifier = Modifier) {
    Column(
        modifier = modifier
            .verticalScroll(rememberScrollState()),
        verticalArrangement = Arrangement.spacedBy(6.dp)
    ) {
        PositionPanel(state)
        AttitudePanel(state)
        OrbitalElementsPanel(state)
    }
}

// ── PositionPanel ──────────────────────────────────────────────────────────────
@Composable
private fun PositionPanel(state: OrbiterState) {
    ConsolePanel(title = "POSITION", subtitle = "ER / ER·min⁻¹") {
        TelemetryRow("X",  state.posX.fmt(5), "ER")
        TelemetryRow("Y",  state.posY.fmt(5), "ER")
        TelemetryRow("Z",  state.posZ.fmt(5), "ER")
        PanelDivider()
        TelemetryRow("Vx", state.velX.fmt(5), "ER/m")
        TelemetryRow("Vy", state.velY.fmt(5), "ER/m")
        TelemetryRow("Vz", state.velZ.fmt(5), "ER/m")
    }
}

// ── AttitudePanel ──────────────────────────────────────────────────────────────
@Composable
private fun AttitudePanel(state: OrbiterState) {
    ConsolePanel(title = "ATTITUDE", subtitle = "RPY / QUAT") {
        TelemetryRow("Bank", state.bankDeg.fmtDeg(2))
        TelemetryRow("Elev", state.elevationDeg.fmtDeg(2))
        TelemetryRow("Head", state.headingDeg.fmtDeg(2))
        PanelDivider()
        TelemetryRow("q0", state.q0.fmt(5))
        TelemetryRow("qi", state.qi.fmt(5))
        TelemetryRow("qj", state.qj.fmt(5))
        TelemetryRow("qk", state.qk.fmt(5))
    }
}

// ── OrbitalElementsPanel ───────────────────────────────────────────────────────
@Composable
private fun OrbitalElementsPanel(state: OrbiterState) {
    ConsolePanel(title = "ORBITAL ELEMENTS", subtitle = "OSCULATING") {
        TelemetryRow("a", state.semiMajorAxisEr.fmt(5), "ER")
        TelemetryRow("e", state.eccentricity.fmt(5))
        TelemetryRow("i", state.inclinationDeg.fmtDeg(2))
        TelemetryRow("\u03A9", state.raanDeg.fmtDeg(2))
        TelemetryRow("\u03C9", state.aopDeg.fmtDeg(2))
        TelemetryRow("\u03BD", state.trueAnomalyDeg.fmtDeg(2))
    }
}

// ── RightColumn ────────────────────────────────────────────────────────────────
@Composable
private fun RightColumn(
    ctrl: ControlUiState,
    state: OrbiterState,
    vm: OrbiterViewModel,
    modifier: Modifier = Modifier
) {
    Column(
        modifier = modifier
            .verticalScroll(rememberScrollState()),
        verticalArrangement = Arrangement.spacedBy(6.dp)
    ) {
        AttitudeHoldPanel(ctrl = ctrl, vm = vm)
        ThrustPlannerPanel(ctrl = ctrl, vm = vm)
        RCSPadPanel(ctrl = ctrl, vm = vm)
        SubsystemsPanel(ctrl = ctrl, vm = vm)
        CommsPanel(ctrl = ctrl)
        StatusStrip(ctrl = ctrl, vm = vm)
    }
}

// ── AttitudeHoldPanel ──────────────────────────────────────────────────────────
@Composable
private fun AttitudeHoldPanel(ctrl: ControlUiState, vm: OrbiterViewModel) {
    ConsolePanel(title = "ATTITUDE HOLD", subtitle = ctrl.holdMode.name) {
        // 3×3 hold mode button grid
        val modes = listOf(
            HoldMode.PROGRADE    to "+Veci",  // body +X = ECI prograde,    +Z = nadir
            HoldMode.RETROGRADE  to "-Veci",  // body +X = ECI retrograde,  +Z = nadir
            HoldMode.NORMAL      to "LVLH",   // body +Z = nadir (primary), +X = ECEF velocity
            HoldMode.ANTINORMAL  to "ECI",    // body axes = ECI axes (identity)
            HoldMode.RADIAL      to "ECEF",   // body axes co-rotate with Earth
            HoldMode.ANTIRADIAL  to "-R",     // body +X = anti-radial (unchanged)
            HoldMode.TARGET      to "\u2192T",
            HoldMode.SUN         to "\u2609", // body -Z toward Sun, +X = ECI prograde
            HoldMode.FREE        to "\u2014"
        )
        Column(verticalArrangement = Arrangement.spacedBy(3.dp)) {
            for (row in 0 until 3) {
                Row(
                    modifier = Modifier.fillMaxWidth(),
                    horizontalArrangement = Arrangement.spacedBy(3.dp)
                ) {
                    for (col in 0 until 3) {
                        val idx = row * 3 + col
                        val (mode, label) = modes[idx]
                        val active = ctrl.holdMode == mode
                        Box(
                            modifier = Modifier
                                .weight(1f)
                                .height(26.dp)
                                .clip(PANEL_SHAPE)
                                .background(
                                    if (active) Amber.copy(alpha = 0.14f).compositeOver(PanelColor2)
                                    else PanelColor2
                                )
                                .border(
                                    width = 1.dp,
                                    color = if (active) Amber else PanelBorder2,
                                    shape = PANEL_SHAPE
                                )
                                .clickable { vm.setHoldMode(mode) },
                            contentAlignment = Alignment.Center
                        ) {
                            Text(
                                text = label,
                                color = if (active) Amber else InkColor,
                                fontSize = 10.sp,
                                fontFamily = MONO,
                                fontWeight = if (active) FontWeight.Bold else FontWeight.Normal,
                                textAlign = TextAlign.Center
                            )
                        }
                    }
                }
            }
        }
        PanelDivider()
        TelemetryRow("Ref Bank", ctrl.refBank.fmt2() + "\u00B0")
        TelemetryRow("Ref Elev", ctrl.refElev.fmt2() + "\u00B0")
        TelemetryRow("Ref Head", ctrl.refHead.fmt2() + "\u00B0")
        PanelDivider()
        SliderRow(
            label = "Gain",
            value = ctrl.attGain,
            valueText = "${ctrl.attGain.toInt()}%",
            onValueChange = { vm.setAttGain(it) },
            valueRange = 0f..100f
        )
    }
}

// Helper: compositeOver so Amber.copy(alpha) blends on top of PanelColor2
private fun Color.compositeOver(background: Color): Color {
    val a = this.alpha
    return Color(
        red   = this.red   * a + background.red   * (1f - a),
        green = this.green * a + background.green * (1f - a),
        blue  = this.blue  * a + background.blue  * (1f - a),
        alpha = 1f
    )
}

// ── ThrustPlannerPanel ─────────────────────────────────────────────────────────
@Composable
private fun ThrustPlannerPanel(ctrl: ControlUiState, vm: OrbiterViewModel) {
    val infiniteTransition = rememberInfiniteTransition(label = "arm_pulse")
    val pulseAlpha by infiniteTransition.animateFloat(
        initialValue = 0.4f,
        targetValue = 0f,
        animationSpec = infiniteRepeatable(tween(1200), RepeatMode.Restart),
        label = "pulse_alpha"
    )

    ConsolePanel(title = "MAIN ENGINE", subtitle = "\u0394v PLANNER") {
        TelemetryRow("\u0394v",       ctrl.dvMps.fmt2() + " m/s")
        TelemetryRow("Duration",      (ctrl.dvMps / 14f).fmt2() + " s")
        TelemetryRow("TIG",           "T-00:00.0")
        PanelDivider()
        SliderRow(
            label = "\u0394v",
            value = ctrl.dvMps,
            valueText = ctrl.dvMps.fmt2() + " m/s",
            onValueChange = { vm.setDv(it) },
            valueRange = 0f..250f
        )
        // Propellant bar
        Column(modifier = Modifier.fillMaxWidth().padding(vertical = 4.dp)) {
            Row(
                modifier = Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.SpaceBetween
            ) {
                Text("PROP", color = InkDim, fontSize = 9.sp, fontFamily = MONO, letterSpacing = 0.8.sp)
                Text("${"%.1f".format(ctrl.prop)}%", color = InkColor, fontSize = 9.sp, fontFamily = MONO)
            }
            Spacer(Modifier.height(3.dp))
            Box(
                modifier = Modifier
                    .fillMaxWidth()
                    .height(5.dp)
                    .clip(PANEL_SHAPE)
                    .background(PanelBorder2)
            ) {
                Box(
                    modifier = Modifier
                        .fillMaxWidth(fraction = (ctrl.prop / 100f).coerceIn(0f, 1f))
                        .fillMaxHeight()
                        .background(
                            Brush.horizontalGradient(listOf(Amber, AccGreen))
                        )
                )
            }
        }
        PanelDivider()
        Row(
            modifier = Modifier.fillMaxWidth().padding(top = 2.dp),
            horizontalArrangement = Arrangement.spacedBy(6.dp)
        ) {
            // ARM button
            Box(
                modifier = Modifier
                    .weight(1f)
                    .height(30.dp)
                    .clip(PANEL_SHAPE)
                    .background(
                        if (ctrl.armed) Amber.copy(alpha = 0.14f).compositeOver(PanelColor2)
                        else PanelColor2
                    )
                    .border(
                        width = 1.dp,
                        color = if (ctrl.armed) Amber else PanelBorder,
                        shape = PANEL_SHAPE
                    )
                    .border(
                        width = 2.dp,
                        color = Amber.copy(alpha = if (ctrl.armed) pulseAlpha else 0f),
                        shape = PANEL_SHAPE
                    )
                    .clickable { vm.toggleArm() },
                contentAlignment = Alignment.Center
            ) {
                Text(
                    text = if (ctrl.armed) "ARMED" else "ARM",
                    color = if (ctrl.armed) Amber else InkColor,
                    fontSize = 10.sp,
                    fontFamily = MONO,
                    fontWeight = FontWeight.Bold
                )
            }
            // FIRE button
            Box(
                modifier = Modifier
                    .weight(1f)
                    .height(30.dp)
                    .clip(PANEL_SHAPE)
                    .background(
                        if (ctrl.armed) AccRed.copy(alpha = 0.14f).compositeOver(PanelColor2)
                        else PanelColor2
                    )
                    .border(
                        width = 1.dp,
                        color = if (ctrl.armed) AccRed else PanelBorder2,
                        shape = PANEL_SHAPE
                    )
                    .then(
                        if (ctrl.armed) Modifier.clickable { vm.fire() } else Modifier
                    ),
                contentAlignment = Alignment.Center
            ) {
                Text(
                    text = "FIRE",
                    color = if (ctrl.armed) AccRed else InkDim,
                    fontSize = 10.sp,
                    fontFamily = MONO,
                    fontWeight = FontWeight.Bold
                )
            }
        }
    }
}

// ── RCSPadPanel ────────────────────────────────────────────────────────────────
@Composable
private fun RCSPadPanel(ctrl: ControlUiState, vm: OrbiterViewModel) {
    ConsolePanel(title = "RCS", subtitle = "6DOF TRANSLATION") {
        // 3×3 grid: row 0 = [+Y, +Z, +X], row 1 = [-X, (center), -Y], row 2 = [↙, -Z, ↗]
        data class RcsBtn(val label: String, val dx: Int, val dy: Int, val dz: Int, val isCenter: Boolean = false)
        val grid = listOf(
            listOf(RcsBtn("+Y", 0, 1, 0), RcsBtn("+Z", 0, 0, 1), RcsBtn("+X", 1, 0, 0)),
            listOf(RcsBtn("-X", -1, 0, 0), RcsBtn("", 0, 0, 0, isCenter = true), RcsBtn("-Y", 0, -1, 0)),
            listOf(RcsBtn("\u2199", -1, -1, 0), RcsBtn("-Z", 0, 0, -1), RcsBtn("\u2197", 1, 1, 0))
        )

        Column(verticalArrangement = Arrangement.spacedBy(3.dp)) {
            for (rowBtns in grid) {
                Row(
                    modifier = Modifier.fillMaxWidth(),
                    horizontalArrangement = Arrangement.spacedBy(3.dp)
                ) {
                    for (btn in rowBtns) {
                        if (btn.isCenter) {
                            // Center indicator
                            Box(
                                modifier = Modifier
                                    .weight(1f)
                                    .height(28.dp)
                                    .clip(PANEL_SHAPE)
                                    .background(PanelColor2)
                                    .border(1.dp, PanelBorder2, PANEL_SHAPE),
                                contentAlignment = Alignment.Center
                            ) {
                                Text(
                                    "RCS",
                                    color = InkDim,
                                    fontSize = 8.sp,
                                    fontFamily = MONO,
                                    letterSpacing = 0.5.sp
                                )
                            }
                        } else {
                            var pressed by remember { mutableStateOf(false) }
                            LaunchedEffect(pressed) {
                                if (pressed) {
                                    delay(220)
                                    pressed = false
                                }
                            }
                            Box(
                                modifier = Modifier
                                    .weight(1f)
                                    .height(28.dp)
                                    .clip(PANEL_SHAPE)
                                    .background(
                                        if (pressed) AccGreen.copy(alpha = 0.18f).compositeOver(PanelColor2)
                                        else PanelColor2
                                    )
                                    .border(
                                        1.dp,
                                        if (pressed) AccGreen else PanelBorder2,
                                        PANEL_SHAPE
                                    )
                                    .clickable {
                                        pressed = true
                                        vm.jogRCS(btn.dx, btn.dy, btn.dz)
                                    },
                                contentAlignment = Alignment.Center
                            ) {
                                Text(
                                    text = btn.label,
                                    color = if (pressed) AccGreen else InkColor,
                                    fontSize = 10.sp,
                                    fontFamily = MONO,
                                    fontWeight = FontWeight.Medium,
                                    textAlign = TextAlign.Center
                                )
                            }
                        }
                    }
                }
            }
        }
        PanelDivider()
        SliderRow(
            label = "Pulse",
            value = ctrl.rcsMag,
            valueText = "${"%.2f".format(ctrl.rcsMag)} m/s",
            onValueChange = { vm.setRcsMag(it) },
            valueRange = 0.01f..2f
        )
    }
}

// ── SubsystemsPanel ────────────────────────────────────────────────────────────
@Composable
private fun SubsystemsPanel(ctrl: ControlUiState, vm: OrbiterViewModel) {
    ConsolePanel(title = "SUBSYSTEMS") {
        data class SysDef(val label: String, val key: String, val active: Boolean)
        val t = ctrl.toggles
        val systems = listOf(
            SysDef("SAS",       "sas",     t.sas),
            SysDef("RCS",       "rcs",     t.rcs),
            SysDef("AUTO-NAV",  "autoNav", t.autoNav),
            SysDef("DOCK MODE", "docking", t.docking),
            SysDef("EXT LIGHTS","lights",  t.lights),
            SysDef("SOLAR DEP", "solar",   t.solar)
        )
        // 2 columns × 3 rows
        for (row in 0 until 3) {
            Row(
                modifier = Modifier.fillMaxWidth().padding(vertical = 2.dp),
                horizontalArrangement = Arrangement.spacedBy(4.dp)
            ) {
                for (col in 0 until 2) {
                    val idx = row * 2 + col
                    val sys = systems[idx]
                    Box(
                        modifier = Modifier
                            .weight(1f)
                            .height(24.dp)
                            .clip(PANEL_SHAPE)
                            .background(PanelColor2)
                            .border(
                                1.dp,
                                if (sys.active) StatusGreen.copy(alpha = 0.5f) else PanelBorder2,
                                PANEL_SHAPE
                            )
                            .clickable { vm.toggleSubsystem(sys.key) }
                            .padding(horizontal = 6.dp),
                        contentAlignment = Alignment.CenterStart
                    ) {
                        Row(
                            verticalAlignment = Alignment.CenterVertically,
                            horizontalArrangement = Arrangement.spacedBy(5.dp)
                        ) {
                            Box(
                                modifier = Modifier
                                    .size(7.dp)
                                    .clip(CircleShape)
                                    .background(
                                        if (sys.active) StatusGreen else PanelBorder
                                    )
                            )
                            Text(
                                text = sys.label,
                                color = if (sys.active) InkColor else InkDim,
                                fontSize = 8.sp,
                                fontFamily = MONO,
                                maxLines = 1,
                                overflow = TextOverflow.Ellipsis
                            )
                        }
                    }
                }
            }
        }
    }
}

// ── CommsPanel ─────────────────────────────────────────────────────────────────
@Composable
private fun CommsPanel(ctrl: ControlUiState) {
    ConsolePanel(title = "COMMS", subtitle = "DSN") {
        TelemetryRow("Station",  "CANBERRA DSS-43")
        TelemetryRow("Link",     ctrl.link.fmt1() + " dB")
        TelemetryRow("Latency",  ctrl.latency.fmt2() + " s")
        PanelDivider()
        // Signal strength bar
        Column(modifier = Modifier.fillMaxWidth().padding(vertical = 4.dp)) {
            Row(
                modifier = Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.SpaceBetween
            ) {
                Text("SIGNAL", color = InkDim, fontSize = 9.sp, fontFamily = MONO, letterSpacing = 0.8.sp)
                Text("${min(100f, ctrl.link * 2f).toInt()}%", color = InkColor, fontSize = 9.sp, fontFamily = MONO)
            }
            Spacer(Modifier.height(3.dp))
            Box(
                modifier = Modifier
                    .fillMaxWidth()
                    .height(5.dp)
                    .clip(PANEL_SHAPE)
                    .background(PanelBorder2)
            ) {
                Box(
                    modifier = Modifier
                        .fillMaxWidth(fraction = (min(100f, ctrl.link * 2f) / 100f).coerceIn(0f, 1f))
                        .fillMaxHeight()
                        .background(StatusGreen)
                )
            }
        }
    }
}

// ── StatusStrip ────────────────────────────────────────────────────────────────
@Composable
private fun StatusStrip(ctrl: ControlUiState, vm: OrbiterViewModel) {
    val infiniteTransition = rememberInfiniteTransition(label = "eng_pulse")
    val engPulse by infiniteTransition.animateFloat(
        initialValue = 1f,
        targetValue = 0.2f,
        animationSpec = infiniteRepeatable(tween(700), RepeatMode.Reverse),
        label = "eng_pulse_alpha"
    )

    Column(
        modifier = Modifier
            .fillMaxWidth()
            .clip(PANEL_SHAPE)
            .background(PanelColor2)
            .border(1.dp, PanelBorder2, PANEL_SHAPE)
            .padding(8.dp),
        verticalArrangement = Arrangement.spacedBy(5.dp)
    ) {
        // Status LED row
        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.spacedBy(8.dp),
            verticalAlignment = Alignment.CenterVertically
        ) {
            StatusLed("CORE", StatusGreen)
            StatusLed("GNC",  StatusGreen)
            StatusLed("PWR",  StatusGreen)
            StatusLed("RCS",  if (ctrl.toggles.rcs) StatusGreen else PanelBorder)
            StatusLed(
                label = "ENG",
                color = if (ctrl.armed) Amber.copy(alpha = engPulse) else PanelBorder
            )
        }

        // ARMED caution banner
        if (ctrl.armed) {
            Box(
                modifier = Modifier
                    .fillMaxWidth()
                    .clip(PANEL_SHAPE)
                    .background(Amber.copy(alpha = 0.08f).compositeOver(PanelColor2))
                    .border(1.dp, Amber, PANEL_SHAPE)
                    .padding(horizontal = 6.dp, vertical = 4.dp),
                contentAlignment = Alignment.Center
            ) {
                Text(
                    text = "\u26A0 ENGINE ARMED \u2014 CHECK ATTITUDE",
                    color = Amber,
                    fontSize = 8.sp,
                    fontFamily = MONO,
                    fontWeight = FontWeight.Bold,
                    letterSpacing = 0.5.sp
                )
            }
        }

        // Reset sim button
        Box(
            modifier = Modifier
                .fillMaxWidth()
                .height(26.dp)
                .clip(PANEL_SHAPE)
                .background(Color.Transparent)
                .border(1.dp, PanelBorder, PANEL_SHAPE)
                .clickable { vm.resetSim() },
            contentAlignment = Alignment.Center
        ) {
            Text(
                "RESET SIM",
                color = InkDim,
                fontSize = 9.sp,
                fontFamily = MONO,
                letterSpacing = 0.8.sp
            )
        }
    }
}

@Composable
private fun StatusLed(label: String, color: Color) {
    Row(
        verticalAlignment = Alignment.CenterVertically,
        horizontalArrangement = Arrangement.spacedBy(3.dp)
    ) {
        Box(
            modifier = Modifier
                .size(6.dp)
                .clip(CircleShape)
                .background(color)
        )
        Text(label, color = InkDim, fontSize = 8.sp, fontFamily = MONO)
    }
}

// ── Shared primitives ──────────────────────────────────────────────────────────
@Composable
private fun ConsolePanel(
    title: String,
    subtitle: String? = null,
    modifier: Modifier = Modifier,
    content: @Composable ColumnScope.() -> Unit
) {
    Column(
        modifier = modifier
            .fillMaxWidth()
            .clip(PANEL_SHAPE)
            .background(
                Brush.verticalGradient(listOf(PanelColor, PanelColor2))
            )
            .border(1.dp, PanelBorder, PANEL_SHAPE)
            .padding(horizontal = 10.dp, vertical = 8.dp)
    ) {
        // Header row
        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.SpaceBetween,
            verticalAlignment = Alignment.CenterVertically
        ) {
            Text(
                text = title,
                color = InkDim,
                fontSize = 8.sp,
                fontFamily = MONO,
                fontWeight = FontWeight.Bold,
                letterSpacing = 1.2.sp
            )
            if (subtitle != null) {
                Text(
                    text = subtitle,
                    color = Amber,
                    fontSize = 8.sp,
                    fontFamily = MONO,
                    maxLines = 1,
                    overflow = TextOverflow.Ellipsis
                )
            }
        }
        Spacer(Modifier.height(6.dp))
        content()
    }
}

@Composable
private fun TelemetryRow(
    label: String,
    value: String,
    unit: String? = null,
    highlight: Boolean = false
) {
    Row(
        modifier = Modifier
            .fillMaxWidth()
            .height(24.dp),
        horizontalArrangement = Arrangement.SpaceBetween,
        verticalAlignment = Alignment.CenterVertically
    ) {
        Text(
            text = label,
            color = InkDim,
            fontSize = 10.sp,
            fontFamily = MONO
        )
        Row(
            verticalAlignment = Alignment.CenterVertically,
            horizontalArrangement = Arrangement.spacedBy(3.dp)
        ) {
            Text(
                text = value,
                color = if (highlight) Amber else InkHi,
                fontSize = 10.sp,
                fontFamily = MONO
            )
            if (unit != null) {
                Text(
                    text = unit,
                    color = InkDim,
                    fontSize = 8.sp,
                    fontFamily = MONO
                )
            }
        }
    }
}

@Composable
private fun ColumnScope.PanelDivider() {
    Spacer(
        modifier = Modifier
            .padding(vertical = 3.dp)
            .fillMaxWidth()
            .height(1.dp)
            .background(PanelBorder2)
    )
}

@Composable
private fun SliderRow(
    label: String,
    value: Float,
    valueText: String,
    onValueChange: (Float) -> Unit,
    valueRange: ClosedFloatingPointRange<Float>
) {
    Column(modifier = Modifier.fillMaxWidth()) {
        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.SpaceBetween,
            verticalAlignment = Alignment.CenterVertically
        ) {
            Text(label, color = InkDim, fontSize = 9.sp, fontFamily = MONO)
            Text(valueText, color = InkColor, fontSize = 9.sp, fontFamily = MONO)
        }
        Slider(
            value = value,
            onValueChange = onValueChange,
            valueRange = valueRange,
            modifier = Modifier.fillMaxWidth().height(24.dp),
            colors = SliderDefaults.colors(
                thumbColor = Amber,
                activeTrackColor = Amber,
                inactiveTrackColor = PanelBorder2
            )
        )
    }
}
