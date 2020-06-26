package it.turboillegali.segwaycontrol

import androidx.lifecycle.LiveData
import androidx.lifecycle.MutableLiveData
import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.jcraft.jsch.Channel
import com.jcraft.jsch.JSch
import com.jcraft.jsch.Session
import kotlinx.coroutines.*
import timber.log.Timber
import java.io.OutputStream
import java.io.PipedInputStream
import java.io.PipedOutputStream
import java.lang.Exception


enum class LEGOState {
    Disconnected,
    Connected,
    Idle,
    Running
}

class MainViewModel : ViewModel() {
    val currentState: MutableLiveData<LEGOState> = MutableLiveData(LEGOState.Disconnected)
    val currentSpeed: MutableLiveData<Int> = MutableLiveData(0)
    val currentSteer: MutableLiveData<Int> = MutableLiveData(0)
    val connectionError: LiveData<Pair<String?, Boolean>>
        get() = _connectionError

    private var session: Session? = null
    private var channel: Channel? = null
    private var outputStream: OutputStream? = null
    private val _connectionError: MutableLiveData<Pair<String?, Boolean>> = MutableLiveData()

    init {
        currentSpeed.observeForever { speed ->
            viewModelScope.launch {
                try {
                    setSpeed(speed)
                } catch (e: Exception) {
                    _connectionError.value = Pair(e.message, false)
                }
            }
        }
        currentSteer.observeForever { steer ->
            viewModelScope.launch {
                try {
                    setSteer(steer)
                } catch (e: Exception) {
                    _connectionError.value = Pair(e.message, false)
                }
            }
        }
    }

    override fun onCleared() {
        super.onCleared()
        viewModelScope.launch {
            try {
                disconnect()
            } catch (e: Exception) {
                Timber.d("Failed to disconnect from robot")
            }
        }
    }

    suspend fun startRobot() {
        try {
            withContext(Dispatchers.IO) {
                outputStream?.write("start\n".toByteArray())
            }
            currentState.value = LEGOState.Running
        } catch (e: Exception) {
            _connectionError.value = Pair(e.message, false)
        }
    }

    suspend fun connect(address: String?) {
        try {
            withContext(Dispatchers.IO) {
                session = JSch().getSession("robot", address)
                session?.setConfig("StrictHostKeyChecking", "no")
                session?.setPassword("maker")
                session?.connect(10_000)
                channel = session?.openChannel("shell")
                val inputStream = PipedInputStream()
                outputStream = PipedOutputStream(inputStream)
                channel?.outputStream = System.out
                channel?.setExtOutputStream(System.out)
                channel?.inputStream = inputStream
                channel?.connect()

                outputStream?.write("sudo ./segway\n".toByteArray())

                delay(500)

                outputStream?.write("maker\n".toByteArray())
                outputStream?.flush()

                delay(1000)
            }

            currentState.value = LEGOState.Connected
        } catch (e: Exception) {
            _connectionError.value = Pair(e.message, true)
        }
    }

    suspend fun stopRobot() {
        try {
            currentState.value = LEGOState.Idle

            withContext(Dispatchers.IO) {
                outputStream?.write("stop\n".toByteArray())
                outputStream?.flush()
            }

            currentSpeed.value = 0
            currentSteer.value = 0
        } catch (e: Exception) {
            _connectionError.value = Pair(e.message, false)
        }
    }

    suspend fun disconnect() {
        try {
            withContext(Dispatchers.IO) {
                outputStream?.write("exit\n".toByteArray())
                outputStream?.flush()
                channel?.disconnect()
                session?.disconnect()
            }

            currentState.value = LEGOState.Disconnected
        } catch (e: Exception) {
            _connectionError.value = Pair(e.message, false)
        }
    }

    private suspend fun setSpeed(speed: Int) {
        withContext(Dispatchers.IO) {
            outputStream?.write("speed_ref=${speed}\n".toByteArray())
            outputStream?.flush()
        }
    }

    private suspend fun setSteer(steer: Int) {
        withContext(Dispatchers.IO) {
            outputStream?.write("steer_ref=${steer}\n".toByteArray())
            outputStream?.flush()
        }
    }

}