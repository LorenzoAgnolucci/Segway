package it.turboillegali.segwaycontrol

import android.app.AlertDialog
import android.app.Dialog
import android.content.Context
import android.os.Bundle
import android.view.WindowManager.LayoutParams.SOFT_INPUT_STATE_VISIBLE
import android.widget.Button
import androidx.core.widget.addTextChangedListener
import androidx.databinding.DataBindingUtil
import androidx.fragment.app.DialogFragment
import it.turboillegali.segwaycontrol.databinding.FragmentSettingsBinding
import timber.log.Timber


class SettingsDialogFragment : DialogFragment() {

    lateinit var okButton: Button

    override fun onCreateDialog(savedInstanceState: Bundle?): Dialog {
        return activity?.let {
            val builder = AlertDialog.Builder(it)
            val binding: FragmentSettingsBinding = DataBindingUtil.inflate(it.layoutInflater, R.layout.fragment_settings, null, false)

            val sharedPreferences = it.getSharedPreferences(it.getString(R.string.default_prefs), Context.MODE_PRIVATE)
            val ipAddressPreferenceKey = it.getString(R.string.ip_address_pref_key)

            binding.ipAddressText.setText(sharedPreferences.getString(ipAddressPreferenceKey, it.getString(R.string.default_ip)))
            binding.ipAddressText.addTextChangedListener { textEditable ->
                val text = textEditable.toString().trim()
                val validIpAddress = Regex("""\d+\.\d+\.\d+\.\d+""").matches(text)

                okButton.isEnabled = validIpAddress

                if (!validIpAddress) {
                    binding.ipAddressLayout.error = "Invalid IP address"
                } else {
                    binding.ipAddressLayout.error = null
                }
            }

            val dialog = builder.setView(binding.root)
                .setTitle("Settings")
                .setIcon(R.drawable.ic_settings_light)
                .setPositiveButton(android.R.string.ok) { _, _ ->
                    Timber.w("Setting ${binding.ipAddressText.text.toString()}")
                    with(sharedPreferences.edit()) {
                        putString(ipAddressPreferenceKey, binding.ipAddressText.text.toString())
                        apply()
                    }
                    dialog?.dismiss()
                }
                .setNegativeButton(android.R.string.cancel) { dialog, _ -> dialog.cancel() }
                .create()

            dialog.setOnShowListener {
                okButton = dialog.getButton(AlertDialog.BUTTON_POSITIVE)
            }

            binding.ipAddressText.requestFocus()
            dialog.window?.setSoftInputMode(SOFT_INPUT_STATE_VISIBLE);
            return dialog
        } ?: throw IllegalStateException("Activity cannot be null")
    }
}