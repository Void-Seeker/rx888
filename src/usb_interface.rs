use anyhow::{Context, Result};
use nusb::MaybeFuture;
use nusb::transfer as ntransfer;
use std::time::Duration;

/// Simplified USB API used only by the flasher.
pub trait UsbInterface {
    /// Control OUT (host -> device)
    fn control_write(
        &self,
        request: u8,
        value: u16,
        index: u16,
        data: &[u8],
        timeout: Duration,
    ) -> Result<()>;

    /// Control IN (device -> host)
    fn control_read(
        &self,
        request: u8,
        value: u16,
        index: u16,
        length: u16,
        timeout: Duration,
    ) -> Result<Vec<u8>>;
}

impl UsbInterface for nusb::Interface {
    fn control_write(
        &self,
        request: u8,
        value: u16,
        index: u16,
        data: &[u8],
        timeout: Duration,
    ) -> Result<()> {
        let out = ntransfer::ControlOut {
            control_type: ntransfer::ControlType::Vendor,
            recipient: ntransfer::Recipient::Device,
            request,
            value,
            index,
            data,
        };
        // Retried for the same reason as Radio::write_register: the FX3 stalls
        // the control endpoint when a register it forwards over I2C does not
        // answer, and the stall clears on the next SETUP packet. The gain and
        // tuner helpers reach the device through here rather than through
        // Radio::write_register, so without this they kept failing after the
        // writes there had been made reliable.
        let mut last = None;
        for attempt in 0..4 {
            if attempt != 0 {
                std::thread::sleep(Duration::from_millis(2));
            }
            match self.control_out(out.clone(), timeout).wait() {
                Ok(_) => return Ok(()),
                Err(e) => last = Some(e),
            }
        }
        Err(last.unwrap()).context("USB write failed after 4 attempts")
    }

    fn control_read(
        &self,
        request: u8,
        value: u16,
        index: u16,
        length: u16,
        timeout: Duration,
    ) -> Result<Vec<u8>> {
        let inp = ntransfer::ControlIn {
            control_type: ntransfer::ControlType::Vendor,
            recipient: ntransfer::Recipient::Device,
            request,
            value,
            index,
            length,
        };
        let mut last = None;
        for attempt in 0..4 {
            if attempt != 0 {
                std::thread::sleep(Duration::from_millis(2));
            }
            match self.control_in(inp.clone(), timeout).wait() {
                Ok(v) => return Ok(v),
                Err(e) => last = Some(e),
            }
        }
        Err(last.unwrap()).context("USB read failed after 4 attempts")
    }
}
