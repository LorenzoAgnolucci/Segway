package it.turboillegali.segwaycontrol

import android.content.Context
import android.os.Bundle
import android.os.SystemClock
import android.view.Menu
import android.view.MenuItem
import android.view.MotionEvent
import android.view.View
import android.widget.SeekBar
import android.widget.SeekBar.OnSeekBarChangeListener
import androidx.activity.viewModels
import androidx.appcompat.app.AlertDialog
import androidx.appcompat.app.AppCompatActivity
import androidx.core.text.HtmlCompat
import androidx.core.text.HtmlCompat.FROM_HTML_MODE_COMPACT
import androidx.databinding.DataBindingUtil
import androidx.lifecycle.Observer
import com.jmedeisis.bugstick.JoystickListener
import it.turboillegali.segwaycontrol.databinding.ActivityMainBinding
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.MainScope
import kotlinx.coroutines.launch
import timber.log.Timber
import kotlin.math.cos
import kotlin.math.roundToInt
import kotlin.math.sin


class MainActivity : AppCompatActivity(), CoroutineScope by MainScope() {
    private lateinit var binding: ActivityMainBinding
    private lateinit var viewModel: MainViewModel
    private val alertBuilder: AlertDialog.Builder by lazy { AlertDialog.Builder(this) }

    override fun onCreate(savedInstanceState: Bundle?) {
        Thread.sleep(500)
        setTheme(R.style.AppTheme)
        super.onCreate(savedInstanceState)
        supportActionBar?.setDisplayShowHomeEnabled(true)
        supportActionBar?.setIcon(R.drawable.ic_toolbar)
        supportActionBar?.setDisplayShowTitleEnabled(false)

        binding = DataBindingUtil.setContentView(this, R.layout.activity_main)
        binding.lifecycleOwner = this
        val model: MainViewModel by viewModels()
        viewModel = model
        binding.viewModel = model

        viewModel.connectionError.observe(this, Observer { params ->
            showErrorDialog(params.first, params.second)
        })

        val blockingSeekbarListener = object : OnSeekBarChangeListener {
            var originalProgress = 0
            override fun onStopTrackingTouch(seekBar: SeekBar) {}
            override fun onStartTrackingTouch(seekBar: SeekBar) {
                originalProgress = seekBar.progress
            }

            override fun onProgressChanged(seekBar: SeekBar, arg1: Int, fromUser: Boolean) {
                if (fromUser) {
                    seekBar.progress = originalProgress
                }
            }
        }
        binding.speedSeekbar.setOnSeekBarChangeListener(blockingSeekbarListener)
        binding.steerSeekbar.setOnSeekBarChangeListener(blockingSeekbarListener)

        binding.joystick.setJoystickListener(object : JoystickListener {
            override fun onDrag(degrees: Float, offset: Float) {
                val maxSpeedValue = this@MainActivity.resources.getInteger(R.integer.max_speed_seek)
                val maxSteerValue = this@MainActivity.resources.getInteger(R.integer.max_steer_seek)
                val speed = maxSpeedValue * offset * sin(degrees * Math.PI / 180)
                val steer = maxSteerValue * offset * cos(degrees * Math.PI / 180)

                model.currentSpeed.value = speed.roundToInt()
                model.currentSteer.value = steer.roundToInt()

                Timber.d("Setting value: $speed $steer")
            }

            override fun onDown() {}

            override fun onUp() {
                model.currentSteer.value = 0
                model.currentSpeed.value = 0
            }
        })
    }

    override fun onCreateOptionsMenu(menu: Menu?): Boolean {
        menuInflater.inflate(R.menu.settings, menu)
        return true
    }

    override fun onOptionsItemSelected(item: MenuItem): Boolean {
        return when (item.itemId) {
            R.id.ip_setting -> {
                SettingsDialogFragment().show(supportFragmentManager, "settings")
                true
            }
            else -> super.onOptionsItemSelected(item)
        }
    }

    fun onConnectClick(view: View) {
        launch {
            view.isEnabled = false
            viewModel.connect(getIPAddressPreference())
        }
    }

    fun onStartClick(view: View) {
        launch {
            viewModel.startRobot()
        }
    }

    fun onStopClick(view: View) {
        launch {
            viewModel.stopRobot()
        }

        // If the user presses the `Stop` button while the joystick is dragged
        // the joystick view is disabled and UP event is not received, and the joystick gets "stuck" in the last position
        // Here we generate a fake `MotionEvent` to call Bugstick listener `onUp` method and reset the joystick position to (0,0)
        val coords = IntArray(2)
        binding.joystick.getLocationOnScreen(coords)
        val x = coords[0]
        val y = coords[1]

        val downTime = SystemClock.uptimeMillis()
        val eventTime = SystemClock.uptimeMillis() + 100
        val metaState = 0
        val motionEvent = MotionEvent.obtain(
            downTime,
            eventTime,
            MotionEvent.ACTION_UP,
            x.toFloat(),
            y.toFloat(),
            metaState
        )

        binding.joystick.dispatchTouchEvent(motionEvent)
        motionEvent.recycle()
    }

    fun onExitClick(view: View) {
        launch {
            viewModel.disconnect()
        }
    }

    private fun showErrorDialog(message: String?, force: Boolean = false) {
        if (viewModel.currentState.value != LEGOState.Disconnected || force) {
            Exception().printStackTrace()
            alertBuilder
                .setTitle("Connection error")
                .setMessage(
                    HtmlCompat.fromHtml(
                        "Unable to connect to robot with IP <font face='monospace'>${getIPAddressPreference()}</font>.<br>" +
                                "<b>Cause</b>: " + message, FROM_HTML_MODE_COMPACT))
                .setPositiveButton(android.R.string.ok, null)
                .setIcon(R.drawable.ic_error)
                .show()

            viewModel.currentState.value = LEGOState.Disconnected
        }
    }

    private fun getIPAddressPreference(): String? {
        val sharedPreferences = getSharedPreferences(getString(R.string.default_prefs), Context.MODE_PRIVATE)
        val ipAddressPreferenceKey = getString(R.string.ip_address_pref_key)
        val ipAddress = sharedPreferences.getString(ipAddressPreferenceKey, getString(R.string.default_ip))
        return ipAddress
    }
}
