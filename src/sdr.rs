use nusb::transfer::{Bulk, ControlIn, ControlOut, ControlType, In, Recipient};
use nusb::{DeviceInfo, Interface, MaybeFuture, list_devices};
use std::sync::Arc;
use std::sync::atomic::AtomicBool;
use std::thread::{self, JoinHandle};
use std::time::Duration;
use thiserror::Error;

use wide::{CmpEq, i16x8};

use crate::gain;
use crate::interface::{self, FX3Command, REG_ADC_ENABLE, RadioModel, Register};

#[cfg(target_os = "windows")]
use crate::win_usb::WinUsb;

const BUILTIN_FIRMWARE: &[u8] = include_bytes!(concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/firmware/RX888_FW.img"
));

/// Callback type for async read operations
/// Receives a slice of received data
pub type ReadCallback = Arc<dyn Fn(Option<&[i16]>) + Send + Sync>;

/// Error type returned by all Radio operations.
#[derive(Debug, Error)]
pub enum SdrError {
    /// No RX888-family device found at the requested USB index.
    #[error("Device not found")]
    DeviceNotFound,

    /// Device is present but not running at SuperSpeed (USB 3.0).
    #[error("Device not in SuperSpeed mode")]
    NotSuperSpeed,

    /// USB device or interface could not be opened or claimed.
    #[error("Failed to open device: {0}")]
    DeviceOpenFailed(String),

    /// Installed firmware version does not match the required version.
    #[error(
        "Firmware version mismatch: expected {expected_major}.{expected_minor}, got {got_major}.{got_minor}"
    )]
    FirmwareVersionMismatch {
        expected_major: u8,
        expected_minor: u8,
        got_major: u8,
        got_minor: u8,
    },

    /// A low-level USB transfer or register operation failed.
    #[error("USB communication error: {0}")]
    CommunicationError(String),

    /// The requested parameter change is not allowed while the device is streaming.
    #[error("Cannot change setting while device is running")]
    DeviceRunning,

    /// `read_async()` was called while a read operation is already in progress.
    #[error("Async read already in progress; call read_cancel() first")]
    AlreadyRunning,

    /// A supplied parameter value is out of the allowed range.
    #[error("Invalid parameter: {0}")]
    InvalidParameter(String),
}

#[derive(PartialEq)]
enum DeviceState {
    Idle,
    Running,
}

/// ADC filter for RX888 PRO
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(C)]
pub enum FilterMode {
    /// 64Mhz LPF Filter
    Freq64MHz = 0,

    /// 32Mhz LPF Filter
    Freq32MHz = 1,

    /// BPF Filter for FM Undersampling
    FMUndersample = 2,

    /// Bypass mode: anti-aliasing must be handled by the input signal
    Bypass = 3,
}

/// Physical RX888-family SDR device API.
///
/// This struct wraps the FX3 USB interface and firmware registers to control
/// RX888/RX888r2/RX888plus devices. It provides a synchronous control surface
/// (open, set gains/frequency, query ranges) and an asynchronous streaming API
/// via `read_async` that delivers raw ADC bytes to a user callback.
///
/// Key points:
/// - Device discovery uses VID/PID and requires SuperSpeed USB.
/// - Model and firmware version are validated at open; mismatches error out.
/// - Some parameters (crystal frequency, direct sampling) are only changeable
///   while idle. Gains and center frequency apply immediately when running.
/// - Async streaming spawns a background thread that reads bulk USB and invokes
///   the user callback with contiguous slices. Call `read_cancel` to stop.
/// - Threading: `read_async` creates one reader thread; cancellation is via
///   an atomic flag and FX3 register writes.
pub struct Radio {
    // readonly fields
    device_info: DeviceInfo,
    interface: nusb::Interface,
    firmware_version: u16,
    model: RadioModel,

    // static parameters, set before starting
    // and not changed during operation
    xtal_freq: u32,
    direct_sampling: bool,

    // dynamic parameters
    center_freq: u64,
    if_gain: f32,
    rf_gain: f32,

    // state
    state: DeviceState,
    adc_flags: u8,
    adc_filter: FilterMode,

    // async read state
    cancel_flag: Option<Arc<AtomicBool>>,
    read_thread: Option<JoinHandle<()>>,
}

impl Radio {
    fn read_register(interface: &Interface, reg: Register) -> Result<u32, SdrError> {
        let data = interface
            .control_in(
                ControlIn {
                    control_type: ControlType::Vendor,
                    recipient: Recipient::Device,
                    request: FX3Command::REGOP as u8,
                    value: 0,
                    index: reg as u16,
                    length: 4,
                },
                Duration::from_millis(500),
            )
            .wait()
            .map_err(|e| SdrError::CommunicationError(e.to_string()))?;
        Ok(u32::from_le_bytes([data[0], data[1], data[2], data[3]]))
    }

    fn write_register(interface: &Interface, reg: Register, value: u32) -> Result<(), SdrError> {
        log::debug!("Writing register {:?} with value {}", reg, value);
        let data = value.to_le_bytes();

        // The FX3 forwards some of these to the tuner over I2C and stalls the
        // control endpoint when that transaction does not go through, which is
        // what made roughly one stream start in six fail on
        // REG_TUNER_CENTER_FREQ_LOW. Nothing is wrong with the USB state: a
        // stall on the control endpoint is cleared by the next SETUP packet, so
        // asking again is all it takes, and the retry costs nothing when the
        // write succeeds first time as it usually does.
        const ATTEMPTS: u32 = 4;
        let mut last = None;
        for attempt in 0..ATTEMPTS {
            if attempt != 0 {
                thread::sleep(Duration::from_millis(2));
            }
            match interface
                .control_out(
                    ControlOut {
                        control_type: ControlType::Vendor,
                        recipient: Recipient::Device,
                        request: FX3Command::REGOP as u8,
                        value: 0,
                        index: reg as u16,
                        data: &data,
                    },
                    Duration::from_millis(500),
                )
                .wait()
            {
                Ok(_) => {
                    if attempt != 0 {
                        log::debug!("register {reg:?} accepted on attempt {}", attempt + 1);
                    }
                    return Ok(());
                }
                // Name the register in the error. These used to arrive as a bare
                // "endpoint stalled", which says nothing about which of the seven
                // writes in a stream start was the one that failed.
                Err(e) => last = Some(format!("{reg:?}: {e}")),
            }
        }
        Err(SdrError::CommunicationError(format!(
            "{} (after {ATTEMPTS} attempts)",
            last.unwrap_or_else(|| format!("{reg:?}: unknown failure"))
        )))
    }

    pub fn open(index: u32) -> Result<Self, SdrError> {
        if let Some(device_info) = Radio::find_device(index) {
            Radio::new(device_info)
        } else {
            Err(SdrError::DeviceNotFound)
        }
    }

    pub(crate) fn new(device_info: DeviceInfo) -> Result<Self, SdrError> {
        let device = device_info
            .open()
            .wait()
            .map_err(|e| SdrError::DeviceOpenFailed(e.to_string()))?;
        let interface = device
            .claim_interface(0)
            .wait()
            .map_err(|e| SdrError::DeviceOpenFailed(e.to_string()))?;

        let raw = Self::read_register(&interface, Register::REG_INFO_RESET)?;
        let bytes = raw.to_le_bytes();
        let firmware_version = u16::from_be_bytes([bytes[1], bytes[2]]);
        let model = bytes[0];

        let expected_version =
            ((interface::FIRMWARE_VER_MAJOR as u16) << 8) | (interface::FIRMWARE_VER_MINOR as u16);

        if firmware_version != expected_version {
            return Err(SdrError::FirmwareVersionMismatch {
                expected_major: interface::FIRMWARE_VER_MAJOR as u8,
                expected_minor: interface::FIRMWARE_VER_MINOR as u8,
                got_major: (firmware_version >> 8) as u8,
                got_minor: (firmware_version & 0xFF) as u8,
            });
        }

        Ok(Radio {
            device_info,
            interface,
            model: match model {
                0x03 => RadioModel::RX888,
                0x04 => RadioModel::RX888r2,
                0x05 => RadioModel::RX888plus,
                0x07 => RadioModel::RX888pro,
                _ => RadioModel::NORADIO,
            },
            firmware_version,
            center_freq: 0,
            if_gain: 0.0,
            rf_gain: 0.0,
            direct_sampling: true,
            xtal_freq: if model == RadioModel::RX888pro as u8 {
                61_440_000
            } else {
                64_000_000
            },
            state: DeviceState::Idle,
            adc_flags: 0,
            cancel_flag: None,
            read_thread: None,

            adc_filter: FilterMode::Freq64MHz,
        })
    }

    /// Return nusb `DeviceInfo` for this radio.
    pub fn device_info(&self) -> &DeviceInfo {
        &self.device_info
    }

    /// Find an RX888-family device by zero-based `index`.
    ///
    /// Filters enumerated USB devices by expected VID/PID and SuperSpeed,
    /// returning the Nth match. Useful when multiple radios are connected.
    pub fn find_device(index: u32) -> Option<DeviceInfo> {
        let mut flashed_devices = 0;

        // Try to find bootloader devices using nusb first
        let devices = list_devices().wait().ok()?;
        let bootloader_devices: Vec<_> = devices
            .into_iter()
            .filter(|d| {
                d.vendor_id() == interface::FIRMWARE_VID
                    && d.product_id() == interface::BOOTLOADER_PID
            })
            .collect();

        // Try flashing with nusb
        for device_info in &bootloader_devices {
            if let Ok(device) = device_info.open().wait() {
                log::info!("Found bootloader device via nusb, attempting to flash firmware...");
                if let Ok(interface) = device.claim_interface(0).wait()
                    && let Ok(()) = crate::flash::download_firmware(&interface, BUILTIN_FIRMWARE)
                {
                    log::info!("Firmware flashed successfully via nusb.");
                    flashed_devices += 1;
                    continue;
                }
            }

            // If nusb failed to open or flash, try WinUsb on Windows
            #[cfg(target_os = "windows")]
            {
                log::info!("nusb failed to access device, trying WinUsb...");
                if let Some(win_usb) =
                    WinUsb::open(interface::FIRMWARE_VID, interface::BOOTLOADER_PID)
                {
                    log::info!(
                        "Found bootloader device via WinUsb, attempting to flash firmware..."
                    );
                    if let Ok(()) = crate::flash::download_firmware(&win_usb, BUILTIN_FIRMWARE) {
                        log::info!("Firmware flashed successfully via WinUsb.");
                        flashed_devices += 1;
                    }
                }
            }
        }

        if flashed_devices > 0 {
            // The FX3 resets and re-enumerates under the firmware PID after an
            // upload. A single flat sleep is a coin flip: too short and the
            // device is not back yet, so this returns None and the caller
            // reports "no device found"; long enough to always be safe and
            // every call pays for it. Poll instead, then let the device settle
            // -- it enumerates a little before it will actually stream, and
            // reads issued in that window come back empty.
            const REENUMERATE_TIMEOUT: Duration = Duration::from_secs(10);
            const POLL_INTERVAL: Duration = Duration::from_millis(100);
            const SETTLE: Duration = Duration::from_millis(500);

            let deadline = std::time::Instant::now() + REENUMERATE_TIMEOUT;
            loop {
                let back = list_devices().wait().ok().is_some_and(|devs| {
                    devs.into_iter().any(|d| {
                        d.vendor_id() == interface::FIRMWARE_VID
                            && d.product_id() == interface::FIRMWARE_PID
                    })
                });
                if back {
                    break;
                }
                if std::time::Instant::now() >= deadline {
                    log::warn!(
                        "device did not re-enumerate within {:?} after firmware upload",
                        REENUMERATE_TIMEOUT
                    );
                    break;
                }
                thread::sleep(POLL_INTERVAL);
            }
            thread::sleep(SETTLE);
        }
        let devices = list_devices().wait().ok()?;
        devices
            .into_iter()
            .filter(|d| {
                d.vendor_id() == interface::FIRMWARE_VID
                    && d.product_id() == interface::FIRMWARE_PID
            })
            .nth(index as usize)
    }

    /// Set external crystal frequency used by the ADC clocking logic.
    /// For the SDR hardware, xtal_freq equals the real sample rate.
    ///
    /// Constraints: must be idle; changing while running returns an error.
    pub fn set_xtal_freq(&mut self, freq: u32) -> Result<(), SdrError> {
        if self.state != DeviceState::Idle {
            return Err(SdrError::DeviceRunning);
        }

        self.xtal_freq = freq;

        Ok(())
    }

    /// Get configured crystal frequency in Hz.
    /// For the SDR hardware, this is the real sample rate.
    pub fn get_xtal_freq(&self) -> u32 {
        self.xtal_freq
    }

    /// Enable/disable direct sampling mode.
    ///
    /// When `true`, ADC is routed directly (HF bands); when `false`, tuner
    /// path is used (VHF/UHF). Must be idle to change.
    pub fn set_direct_sampling(&mut self, mode: bool) -> Result<(), SdrError> {
        if self.state != DeviceState::Idle {
            return Err(SdrError::DeviceRunning);
        }
        self.direct_sampling = mode;
        Ok(())
    }

    /// Query direct sampling mode.
    pub fn get_direct_sampling(&self) -> bool {
        self.direct_sampling
    }

    /// Query the tuner PLL lock status (REG_TUNER bit 1, read-only).
    ///
    /// Only meaningful in tuner mode: in direct sampling there is no PLL to
    /// lock and this reports false.
    pub fn get_pll_lock(&self) -> Result<bool, SdrError> {
        const REG_TUNER_PLL_LOCK: u32 = 1 << 1;
        let value = Self::read_register(&self.interface, Register::REG_TUNER)?;
        Ok(value & REG_TUNER_PLL_LOCK != 0)
    }

    /// Get detected radio model.
    pub fn get_model(&self) -> crate::interface::RadioModel {
        self.model
    }

    /// Return IF gain range (min, max) in dB according to current model/mode.
    pub fn get_if_gain_range(&self) -> (f32, f32) {
        crate::gain::get_if_gain_range(self.model, self.direct_sampling)
    }

    /// Return IF gain steps slice for current model/mode.
    pub fn get_if_gain_steps(&self) -> &'static [f32] {
        crate::gain::get_if_gain_steps(self.model, self.direct_sampling)
    }

    /// Return RF gain range (min, max) in dB according to current model/mode.
    pub fn get_rf_gain_range(&self) -> (f32, f32) {
        crate::gain::get_rf_gain_range(self.model, self.direct_sampling)
    }

    /// Return RF gain steps slice for current model/mode.
    pub fn get_rf_gain_steps(&self) -> &'static [f32] {
        crate::gain::get_rf_gain_steps(self.model, self.direct_sampling)
    }

    /// Set IF gain in dB.
    ///
    /// Maps to a hardware-specific gain index depending on model and mode.
    /// When running, applies immediately via firmware registers; otherwise
    /// stored and applied on start.
    pub fn set_if_gain(&mut self, gain: f32) -> Result<(), SdrError> {
        self.if_gain = gain;

        if self.state == DeviceState::Running {
            // figure out the right index for the gain
            // Map the requested gain (dB) into a hardware index per model/mode
            let gain_index = gain::if_gain_to_index(self.model, self.direct_sampling, gain);

            // write index to appropriate register so firmware can apply it
            if self.direct_sampling {
                Self::write_register(
                    &self.interface,
                    Register::REG_DIRECT_IF_GAIN,
                    gain_index as u32,
                )?;
            } else {
                Self::write_register(
                    &self.interface,
                    Register::REG_TUNER_IF_GAIN,
                    gain_index as u32,
                )?;
            }

            let steps = gain::get_if_gain_steps(self.model, self.direct_sampling);
            if !steps.is_empty() && (gain_index as usize) < steps.len() {
                self.if_gain = steps[gain_index as usize];
            }
        }

        Ok(())
    }

    /// Get current IF gain in dB.
    pub fn get_if_gain(&self) -> f32 {
        self.if_gain
    }

    /// Set RF gain in dB.
    ///
    /// Maps to a hardware-specific index; applies immediately when running.
    pub fn set_rf_gain(&mut self, gain: f32) -> Result<(), SdrError> {
        self.rf_gain = gain;

        if self.state == DeviceState::Running {
            // Map RF gain and write to firmware register for immediate application
            let gain_index = gain::rf_gain_to_index(self.model, self.direct_sampling, gain);

            if self.direct_sampling {
                Self::write_register(
                    &self.interface,
                    Register::REG_DIRECT_RF_GAIN,
                    gain_index as u32,
                )?;
            } else {
                Self::write_register(
                    &self.interface,
                    Register::REG_TUNER_RF_GAIN,
                    gain_index as u32,
                )?;
            }

            let rf_steps = gain::get_rf_gain_steps(self.model, self.direct_sampling);
            if !rf_steps.is_empty() && (gain_index as usize) < rf_steps.len() {
                self.rf_gain = rf_steps[gain_index as usize];
            }
        }

        Ok(())
    }

    /// Get current RF gain in dB.
    pub fn get_rf_gain(&self) -> f32 {
        self.rf_gain
    }

    /// Set physical radio center frequency in Hz.
    ///
    /// When running, writes high/low 32-bit halves to firmware registers for
    /// immediate retune. Stored otherwise and applied on start.
    pub fn set_center_freq(&mut self, freq: u64) -> Result<(), SdrError> {
        self.center_freq = freq;

        if self.state == DeviceState::Running {
            Self::write_register(
                &self.interface,
                Register::REG_TUNER_CENTER_FREQ_HIGH,
                (freq >> 32) as u32,
            )?;
            Self::write_register(
                &self.interface,
                Register::REG_TUNER_CENTER_FREQ_LOW,
                (freq & 0xFFFFFFFF) as u32,
            )?;
        }

        Ok(())
    }

    /// Get current physical center frequency in Hz.
    pub fn get_center_freq(&self) -> u64 {
        self.center_freq
    }

    /// Get validated firmware version (major<<8 | minor).
    pub fn get_firmware_version(&self) -> u16 {
        self.firmware_version
    }

    /// Start asynchronous streaming from the FX3.
    ///
    /// Spawns a reader thread that performs bulk transfers and invokes
    /// `callback(&[u8])` with raw ADC bytes. If `blocking` is true, the call
    /// will join the thread (and thus not return) until `read_cancel()` is
    /// invoked from another thread. Validates SuperSpeed mode prior to start.
    /// The register writes that arm the hardware for streaming.
    ///
    /// Split out from read_async so a failure part-way through is one error for
    /// the caller to undo, rather than seven early returns each leaving the
    /// device in a different half-configured state.
    fn arm_for_streaming(&mut self) -> Result<(), SdrError> {
        Self::write_register(
            &self.interface,
            Register::REG_TUNER,
            if self.direct_sampling { 0 } else { 1 },
        )?;

        Self::write_register(&self.interface, Register::REG_ADCFREQ, self.xtal_freq)?;
        Self::write_register(
            &self.interface,
            Register::REG_DIRECT_ADC_FILTER,
            self.adc_filter as u32,
        )?;
        // these are applied immediately because the state is already running
        self.set_center_freq(self.center_freq)?;
        self.set_if_gain(self.if_gain)?;
        self.set_rf_gain(self.rf_gain)?;

        Self::write_register(
            &self.interface,
            Register::REG_ADC,
            (self.adc_flags | REG_ADC_ENABLE) as u32,
        )?;
        Ok(())
    }

    pub fn read_async<F>(&mut self, callback: F) -> Result<(), SdrError>
    where
        F: Fn(Option<&[i16]>) + Send + Sync + 'static,
    {
        // Check if already running
        if self.read_thread.is_some() {
            return Err(SdrError::AlreadyRunning);
        }

        if self.device_info().speed() != Some(nusb::Speed::Super) {
            return Err(SdrError::NotSuperSpeed);
        }

        // The settings below only reach the hardware when the device is already
        // marked running, so the state has to be set before them rather than
        // after. That makes everything from here on responsible for putting it
        // back on the way out: any of these writes can fail -- they are 500 ms
        // control transfers with no retry -- and returning straight out left the
        // device marked running with no reader thread behind it. A caller that
        // trusted the state then saw a stream that was not there, and the next
        // start had to work it out for itself.
        self.state = DeviceState::Running;
        if let Err(e) = self.arm_for_streaming() {
            self.state = DeviceState::Idle;
            return Err(e);
        }

        // Create shared cancel flag
        let cancel_flag = Arc::new(AtomicBool::new(false));
        let cancel_flag_clone = cancel_flag.clone();

        // Wrap callback in Arc
        let callback = Arc::new(callback);

        // Clone interface for the thread
        let interface = self.interface.clone();

        let rando_flag = self.adc_flags & interface::REG_ADC_RANDO != 0;

        // Spawn read thread
        let thread = thread::spawn(move || {
            let worker = AsyncReadWorker {
                interface,
                cancel_flag: cancel_flag_clone,
                rando_flag,
                callback,
            };
            worker.start();
        });

        // Store cancel flag and thread
        self.cancel_flag = Some(cancel_flag);
        self.read_thread = Some(thread);

        Ok(())
    }

    /// Cancel asynchronous streaming started by `read_async`.
    ///
    /// Signals the reader thread via an atomic flag, joins it, and clears the
    /// ADC enable bit to stop FX3 streaming. Safe to call if not running.
    pub fn read_cancel(&mut self) -> Result<(), SdrError> {
        if self.read_thread.is_none() {
            // Not running
            return Ok(());
        }

        if let Some(cancel_flag) = self.cancel_flag.take() {
            // Signal cancellation
            cancel_flag.store(true, std::sync::atomic::Ordering::SeqCst);

            // Wait for thread to finish
            if let Some(thread) = self.read_thread.take() {
                thread.join().ok();
            }

            // Stop FX3
            Self::write_register(&self.interface, Register::REG_ADC, self.adc_flags as u32)?;

            // power off most stuffs
            Self::write_register(&self.interface, Register::REG_TUNER, 0)?;
            Self::write_register(&self.interface, Register::REG_DIRECT_ANT_BIAS, 0)?;
            Self::write_register(&self.interface, Register::REG_TUNER_ANT_BIAS, 0)?;

            self.state = DeviceState::Idle;
        }

        Ok(())
    }

    pub fn enable_adc_dither(&mut self, enable: bool) -> Result<(), SdrError> {
        if enable {
            self.adc_flags |= interface::REG_ADC_DITHER;
        } else {
            self.adc_flags &= !interface::REG_ADC_DITHER;
        }

        if self.state == DeviceState::Running {
            // Apply immediately
            Self::write_register(
                &self.interface,
                Register::REG_ADC,
                (self.adc_flags | interface::REG_ADC_ENABLE) as u32,
            )?;
        }

        Ok(())
    }

    pub fn enable_adc_pga(&mut self, enable: bool) -> Result<(), SdrError> {
        if enable {
            self.adc_flags |= interface::REG_ADC_PGA;
        } else {
            self.adc_flags &= !interface::REG_ADC_PGA;
        }

        if self.state == DeviceState::Running {
            // Apply immediately
            Self::write_register(
                &self.interface,
                Register::REG_ADC,
                (self.adc_flags | interface::REG_ADC_ENABLE) as u32,
            )?;
        }

        Ok(())
    }

    pub fn enable_adc_rando(&mut self, enable: bool) -> Result<(), SdrError> {
        if self.state != DeviceState::Idle {
            return Err(SdrError::DeviceRunning);
        }

        if enable {
            self.adc_flags |= interface::REG_ADC_RANDO;
        } else {
            self.adc_flags &= !interface::REG_ADC_RANDO;
        }

        Ok(())
    }

    /// Enable or disable antenna bias voltage
    /// - `index`: 0 for direct sampling mode, 1 for tuner mode
    /// - `enable`: true to enable, false to disable
    ///
    /// Returns: Result<(), SdrError>
    ///
    /// Note: This setting takes effect immediately.
    pub fn enable_antenna_bias(&mut self, index: i32, enable: bool) -> Result<(), SdrError> {
        let reg = match index {
            0 => Register::REG_DIRECT_ANT_BIAS,
            1 => Register::REG_TUNER_ANT_BIAS,
            _ => {
                return Err(SdrError::InvalidParameter(format!(
                    "Invalid antenna bias index: {}",
                    index
                )));
            }
        };

        Self::write_register(&self.interface, reg, if enable { 1 } else { 0 })?;
        Ok(())
    }

    pub fn enable_hf_highz(&mut self, enable: bool) -> Result<(), SdrError> {
        if enable {
            self.adc_flags |= interface::REG_HF_HIGHZ;
        } else {
            self.adc_flags &= !interface::REG_HF_HIGHZ;
        }

        if self.state == DeviceState::Running {
            // Apply immediately
            Self::write_register(
                &self.interface,
                Register::REG_ADC,
                (self.adc_flags | interface::REG_ADC_ENABLE) as u32,
            )?;
        }

        Ok(())
    }

    /// Enable or disable external clock mode (HF high-Z).
    /// When enabled, the ADC clock is driven by an external source connected to
    /// the HF input, and the internal crystal is disconnected.
    ///
    /// <param name="enable">true to enable external clock mode, false to disable</param>
    /// <returns>Result<(), SdrError></returns>
    pub fn enable_ext_clock(&mut self, enable: bool) -> Result<(), SdrError> {
        if enable {
            self.adc_flags |= interface::REG_EXT_CLOCK;
        } else {
            self.adc_flags &= !interface::REG_EXT_CLOCK;
        }

        if self.state == DeviceState::Running {
            // Apply immediately
            Self::write_register(
                &self.interface,
                Register::REG_ADC,
                (self.adc_flags | interface::REG_ADC_ENABLE) as u32,
            )?;
        }

        Ok(())
    }

    pub fn set_adc_filter(&mut self, filter: FilterMode) -> Result<(), SdrError> {
        self.adc_filter = filter;

        if self.state == DeviceState::Running {
            // Apply immediately
            Self::write_register(
                &self.interface,
                Register::REG_DIRECT_ADC_FILTER,
                self.adc_filter as u32,
            )?;
        }

        Ok(())
    }

    /// Get tuner status:
    /// Returns: Result<(bool, bool), SdrError>
    /// - PLL locked: true if PLL is locked, false otherwise
    /// - Harmonic mode: true if Harmonic is used, false otherwise
    /// - Last freq tune success: true if last frequency tune was successful, false otherwise
    pub fn get_tuner_status(&self) -> Result<(bool, bool), SdrError> {
        let status = Self::read_register(&self.interface, Register::REG_TUNER)?;
        let locked = (status & 0x2) != 0;
        let harmonic = (status & 0x4) != 0;
        Ok((locked, harmonic))
    }

    /// Set external GPIO state for the 7 GPIO pins on the I/O expander.
    /// - `gpio_state`: 6-bit value representing the desired output state of the GPIO pins
    ///
    /// Returns: Result<(), SdrError>
    ///
    /// Note: Only bits 0-6 are valid for GPIO state; bit 7 is ignored.
    pub fn set_ext_gpio(&mut self, gpio_state: u8) -> Result<(), SdrError> {
        // Only bits 0-5 are valid for GPIO state
        // return an error if invalid bits are set
        if gpio_state & 0xC0 != 0 {
            return Err(SdrError::InvalidParameter(
                "Invalid GPIO state: only bits 0-5 are valid".to_string(),
            ));
        }
        let gpio_state = gpio_state & 0x3F;
        Self::write_register(&self.interface, Register::REG_EXT_GPIO, gpio_state as u32)?;
        Ok(())
    }

    /// Get current external GPIO state for the 7 GPIO pins on the I/O expander.
    ///
    /// Returns: Result<u8, SdrError>
    ///     - 7-bit value representing the current output state of the GPIO pins
    ///
    /// Note: Only bits 0-6 are valid for GPIO state; bit 7 is always 0.
    pub fn get_ext_gpio(&self) -> Result<u8, SdrError> {
        let state = Self::read_register(&self.interface, Register::REG_EXT_GPIO)?;
        Ok((state & 0x7F) as u8)
    }

    /// Enable or disable the preamplifier in direct sampling mode.
    /// When enabled, the preamp is inserted in the signal path to provide gain
    /// for weak signals. This setting applies both in direct sampling mode and tuner mode.
    pub fn enable_preamp(&mut self, enable: bool) -> Result<(), SdrError> {
        if self.direct_sampling {
            Self::write_register(
                &self.interface,
                Register::REG_DIRECT_PREAMP,
                if enable { 1 } else { 0 },
            )?;
        } else {
            Self::write_register(
                &self.interface,
                Register::REG_TUNER_PREAMP,
                if enable { 1 } else { 0 },
            )?;
        }
        Ok(())
    }
}

struct AsyncReadWorker {
    interface: Interface,
    cancel_flag: Arc<AtomicBool>,
    rando_flag: bool,
    callback: ReadCallback,
}

impl Drop for Radio {
    fn drop(&mut self) {
        // Cancel any ongoing async read
        self.read_cancel().ok();
    }
}

impl AsyncReadWorker {
    /// Consecutive failed transfers tolerated before the stream is given up.
    /// A single stall is often transient; the endpoint recovers once the
    /// remaining queued transfers drain.
    const MAX_CONSECUTIVE_ERRORS: u32 = 32;

    pub fn start(&self) {
        // Do not unwrap: the release profile builds with panic="abort", so a
        // missing endpoint would abort the host process instead of reporting a
        // streaming failure through the callback.
        let mut endpoint = match self.interface.endpoint::<Bulk, In>(0x81) {
            Ok(ep) => ep,
            Err(e) => {
                log::error!("could not claim bulk endpoint 0x81: {}", e);
                (self.callback)(None);
                return;
            }
        };

        // Pre-submit buffers for async transfers
        for _ in 0..32 {
            // buffer size is 16 (maxburst) * 1024 (SS packet size)
            let buf = endpoint.allocate(16 * 1024);
            endpoint.submit(buf);
        }

        let mut errors: u32 = 0;

        // Main read loop
        loop {
            // Check for cancellation
            if self.cancel_flag.load(std::sync::atomic::Ordering::SeqCst) {
                break;
            }

            // A timeout is not the end of the stream. Treating `None` as a loop
            // exit ends streaming silently on any gap longer than the poll
            // interval -- which is routine at low sample rates, where one 16 KiB
            // transfer can take longer than this to fill.
            let Some(transfer) = endpoint.wait_next_complete(Duration::from_millis(100)) else {
                continue;
            };

            match transfer.status {
                Ok(()) => {
                    errors = 0;

                    // Call user callback with received data
                    if transfer.actual_len > 0 {
                        // SAFETY: We ensure the buffer is properly aligned and sized for i16
                        let len = transfer.actual_len / 2;
                        let data = unsafe {
                            std::slice::from_raw_parts_mut(
                                transfer.buffer.as_ptr() as *mut i16,
                                len,
                            )
                        };
                        if self.rando_flag {
                            Self::derando(data);
                        }
                        (self.callback)(Some(data));
                    }

                    // Re-submit the buffer for further reading
                    endpoint.submit(transfer.buffer);
                }
                Err(e) => {
                    // Some errors, such as stalls, are transient: keep the
                    // buffer in flight and only give up once they persist.
                    errors += 1;
                    log::warn!("USB transfer error ({}/{}): {}",
                        errors, Self::MAX_CONSECUTIVE_ERRORS, e);

                    if errors >= Self::MAX_CONSECUTIVE_ERRORS {
                        log::error!("too many consecutive USB transfer errors, stopping");
                        // signal the error to the callback so it can clean up
                        (self.callback)(None);
                        break;
                    }

                    endpoint.submit(transfer.buffer);
                }
            }
        }
    }

    // Implement De-rand algorithem
    // if (d & 0x01 == 0x01)
    //      d = d xor (-2)
    // else
    //      d = d; (unchanged)
    fn derando(data: &mut [i16]) {
        const LANES: usize = i16x8::LANES as usize;
        let xor_vec = i16x8::splat(-2);

        let mut i = 0usize;
        let len = data.len();

        // process chunks of 8
        while i + LANES <= len {
            // load 8 values (unaligned load)
            let v = i16x8::from_slice_unaligned(&data[i..i + LANES]);

            // compute mask: (v & 1) == 1
            // wide supports bitwise and and comparisons that produce a mask-like vector
            let low_bit = v & i16x8::splat(1);
            let cmp = low_bit.simd_eq(i16x8::splat(1)); // produces a mask-like vector

            // select xor_vec where cmp is true, else zero
            let to_xor = cmp.blend(xor_vec, i16x8::splat(0));

            // apply xor
            let out = v ^ to_xor;

            // store back
            data[i..i + LANES].copy_from_slice(out.as_array());

            i += LANES;
        }

        // Scalar tail. Without this the last len % 8 samples are passed through
        // still randomized: a full 16 KiB transfer happens to be a multiple of
        // 8 samples, but a short transfer is not, and neither is any caller
        // that hands us an arbitrary slice.
        while i < len {
            if data[i] & 1 == 1 {
                data[i] ^= -2;
            }
            i += 1;
        }
    }
}

#[cfg(test)]
mod tests {
    use serial_test::serial;

    use crate::interface::{FIRMWARE_VER_MAJOR, FIRMWARE_VER_MINOR};

    use super::*;

    /// Reference implementation, one sample at a time.
    fn derando_scalar(data: &[i16]) -> Vec<i16> {
        data.iter()
            .map(|&d| if d & 1 == 1 { d ^ -2i16 } else { d })
            .collect()
    }

    #[test]
    fn test_rando() {
        let mut data = vec![1, 2, 3, 4, 5, 6, 7, 8, 9, -1, -2, -3, 9, -1, -2, -3];
        let expected = derando_scalar(&data);

        AsyncReadWorker::derando(&mut data);

        assert_eq!(expected, data);
    }

    /// The vectorized path handles 8 samples at a time. Exercise every possible
    /// remainder: a length that is a multiple of 8 leaves no tail, so the
    /// original 16-element test could not catch samples being skipped there.
    #[test]
    fn test_rando_unaligned_lengths() {
        for len in 0..40usize {
            let mut data: Vec<i16> = (0..len).map(|i| (i as i16) * 7 - 13).collect();
            let expected = derando_scalar(&data);

            AsyncReadWorker::derando(&mut data);

            assert_eq!(expected, data, "mismatch at length {}", len);
        }
    }

    /// De-randomizing twice must return the original: the transform is an
    /// involution, which is what makes it safe to apply in place.
    #[test]
    fn test_rando_is_involution() {
        let original: Vec<i16> = (-300..300).step_by(7).collect();
        let mut data = original.clone();

        AsyncReadWorker::derando(&mut data);
        AsyncReadWorker::derando(&mut data);

        assert_eq!(original, data);
    }

    #[test]
    fn test_find_device() {
        let device = Radio::find_device(10);
        assert!(device.is_none());

        let device = Radio::find_device(0);
        assert!(device.is_some());

        let device_info = device.unwrap();
        assert_eq!(device_info.vendor_id(), interface::FIRMWARE_VID);
        assert_eq!(device_info.speed(), Some(nusb::Speed::Super));
    }

    #[test]
    #[serial]
    fn test_new_radio() {
        let device_info = Radio::find_device(0).expect("Device not found");
        let radio = Radio::new(device_info).expect("Failed to create radio");
        assert_eq!(radio.device_info().vendor_id(), interface::FIRMWARE_VID);
        assert!(radio.get_xtal_freq() > 0);
        assert!(radio.get_direct_sampling());
        assert_eq!(radio.get_if_gain(), 0.0);
        assert_eq!(radio.get_rf_gain(), 0.0);
        assert_eq!(radio.get_center_freq(), 0);
        assert_eq!(
            radio.get_firmware_version(),
            ((FIRMWARE_VER_MAJOR << 8) + FIRMWARE_VER_MINOR) as u16
        );
        assert_ne!(radio.model, RadioModel::NORADIO);

        drop(radio);
    }

    #[test]
    #[serial]
    fn test_read_direct() {
        let period_secs = 3;
        let device_info = Radio::find_device(0).expect("Device not found");
        let mut radio = Radio::new(device_info).expect("Failed to create radio");

        let count = Arc::new(std::sync::atomic::AtomicUsize::new(0));
        let count_clone = count.clone();

        assert!(radio.set_direct_sampling(true).is_ok());
        assert!(radio.set_xtal_freq(64_000_000).is_ok());

        // Start async read with callback
        radio
            .read_async(move |data| {
                count_clone.fetch_add(data.unwrap().len(), std::sync::atomic::Ordering::SeqCst);
            })
            .expect("Failed to start async read");

        // Let it run for a bit
        std::thread::sleep(Duration::from_secs(period_secs));

        assert!(radio.set_xtal_freq(128_000_000).is_err());

        // Cancel reading
        radio.read_cancel().expect("Failed to cancel read");
        let _samplerate = ((count.load(std::sync::atomic::Ordering::SeqCst) as f64
            / period_secs as f64)
            / 1_000_000.0)
            .round() as u64;
        // assert!((60..=64).contains(&samplerate));

        assert!(radio.set_xtal_freq(128_000_000).is_ok());

        drop(radio);
    }
}
