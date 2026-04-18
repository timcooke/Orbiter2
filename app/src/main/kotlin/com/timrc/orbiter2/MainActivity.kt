package com.timrc.orbiter2

import android.os.Bundle
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.enableEdgeToEdge
import androidx.compose.runtime.Composable
import com.timrc.orbiter2.ui.OrbiterScreen
import com.timrc.orbiter2.ui.theme.OrbiterTheme

class MainActivity : ComponentActivity() {
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        enableEdgeToEdge()
        setContent {
            OrbiterTheme {
                OrbiterApp()
            }
        }
    }
}

@Composable
fun OrbiterApp() {
    OrbiterScreen()
}
