/// Bit-bang I2C driver using raw GPIO registers.
/// Works reliably on nRF52840 regardless of SoftDevice state.

const P0_OUTSET: *mut u32 = 0x5000_0508 as *mut u32;
const P0_OUTCLR: *mut u32 = 0x5000_050C as *mut u32;
const P0_IN: *const u32 = 0x5000_0510 as *const u32;
const P0_PIN_CNF: *mut u32 = 0x5000_0700 as *mut u32;

// Open-drain output with pull-up: DIR=1, INPUT=0(connect), PULL=3(pullup), DRIVE=6(S0D1)
const I2C_PIN_CNF: u32 = 1 | (0 << 1) | (3 << 2) | (6 << 8);

pub struct BitbangI2c {
    scl_bit: u32,
    sda_bit: u32,
}

impl BitbangI2c {
    /// Create a new bitbang I2C driver on P0 pins.
    /// Configures pins as open-drain with pull-up.
    pub fn new(scl_pin: u32, sda_pin: u32) -> Self {
        unsafe {
            P0_PIN_CNF.add(scl_pin as usize).write_volatile(I2C_PIN_CNF);
            P0_PIN_CNF.add(sda_pin as usize).write_volatile(I2C_PIN_CNF);
            // Start idle (both high)
            P0_OUTSET.write_volatile((1 << scl_pin) | (1 << sda_pin));
        }
        cortex_m::asm::delay(1000);
        Self {
            scl_bit: 1 << scl_pin,
            sda_bit: 1 << sda_pin,
        }
    }

    #[inline(always)]
    fn delay(&self) {
        cortex_m::asm::delay(320); // ~5µs at 64MHz → ~100kHz
    }

    #[inline(always)]
    fn scl_high(&self) {
        unsafe { P0_OUTSET.write_volatile(self.scl_bit); }
    }

    #[inline(always)]
    fn scl_low(&self) {
        unsafe { P0_OUTCLR.write_volatile(self.scl_bit); }
    }

    #[inline(always)]
    fn sda_high(&self) {
        unsafe { P0_OUTSET.write_volatile(self.sda_bit); }
    }

    #[inline(always)]
    fn sda_low(&self) {
        unsafe { P0_OUTCLR.write_volatile(self.sda_bit); }
    }

    #[inline(always)]
    fn read_sda(&self) -> bool {
        unsafe { (P0_IN.read_volatile() & self.sda_bit) != 0 }
    }

    fn start(&self) {
        self.sda_high(); self.delay();
        self.scl_high(); self.delay();
        self.sda_low();  self.delay();
        self.scl_low();  self.delay();
    }

    fn stop(&self) {
        self.sda_low();  self.delay();
        self.scl_high(); self.delay();
        self.sda_high(); self.delay();
    }

    /// Write a byte, return true if ACK received.
    fn write_byte(&self, byte: u8) -> bool {
        for bit in (0..8).rev() {
            if (byte >> bit) & 1 == 1 {
                self.sda_high();
            } else {
                self.sda_low();
            }
            self.delay();
            self.scl_high(); self.delay();
            self.scl_low();  self.delay();
        }
        // ACK bit
        self.sda_high(); self.delay();
        self.scl_high(); self.delay();
        let ack = !self.read_sda();
        self.scl_low();  self.delay();
        ack
    }

    /// Read a byte, send ACK if `ack` is true, NACK if false.
    fn read_byte(&self, ack: bool) -> u8 {
        let mut byte = 0u8;
        self.sda_high(); // Release SDA for reading
        for bit in (0..8).rev() {
            self.delay();
            self.scl_high(); self.delay();
            if self.read_sda() {
                byte |= 1 << bit;
            }
            self.scl_low(); self.delay();
        }
        // Send ACK or NACK
        if ack {
            self.sda_low();
        } else {
            self.sda_high();
        }
        self.delay();
        self.scl_high(); self.delay();
        self.scl_low();  self.delay();
        self.sda_high();
        byte
    }

    /// Write a register value.
    pub fn write_reg(&self, addr: u8, reg: u8, val: u8) -> bool {
        self.start();
        if !self.write_byte(addr << 1) { self.stop(); return false; }
        if !self.write_byte(reg)        { self.stop(); return false; }
        if !self.write_byte(val)        { self.stop(); return false; }
        self.stop();
        true
    }

    /// Read a single register.
    pub fn read_reg(&self, addr: u8, reg: u8) -> Option<u8> {
        self.start();
        if !self.write_byte(addr << 1) { self.stop(); return None; }
        if !self.write_byte(reg)        { self.stop(); return None; }
        // Repeated start
        self.start();
        if !self.write_byte((addr << 1) | 1) { self.stop(); return None; }
        let val = self.read_byte(false); // NACK last byte
        self.stop();
        Some(val)
    }

    /// Read multiple registers into a buffer.
    pub fn read_regs(&self, addr: u8, reg: u8, buf: &mut [u8]) -> bool {
        if buf.is_empty() { return true; }
        self.start();
        if !self.write_byte(addr << 1) { self.stop(); return false; }
        if !self.write_byte(reg)        { self.stop(); return false; }
        // Repeated start
        self.start();
        if !self.write_byte((addr << 1) | 1) { self.stop(); return false; }
        for i in 0..buf.len() {
            let ack = i < buf.len() - 1; // NACK last byte
            buf[i] = self.read_byte(ack);
        }
        self.stop();
        true
    }
}
