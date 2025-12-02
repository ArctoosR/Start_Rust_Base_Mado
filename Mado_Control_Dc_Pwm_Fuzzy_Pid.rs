#![no_std]
#![no_main]

use cortex_m::asm;
use cortex_m::peripheral::syst::SystClkSource;
use cortex_m_rt::entry;
use panic_halt as _;

use core::fmt::Write as FmtWrite;
use heapless::String;

use stm32f1xx_hal::{
    adc::Adc,
    gpio::GpioExt,
    i2c::I2c,
    pac::{self, Peripherals},
    prelude::*,
};
use embedded_hal_02::blocking::i2c::Write;

// ===============================
// System clock setup to 72MHz (PAC)
// ===============================
fn setup_clocks() {
    let rcc   = unsafe { &*pac::RCC::ptr() };
    let flash = unsafe { &*pac::FLASH::ptr() };

    flash.acr().modify(|_, w| unsafe {
        w.prftbe().set_bit()
         .latency().bits(2)
    });

    rcc.cr().modify(|_, w| w.hseon().set_bit());
    while rcc.cr().read().hserdy().bit_is_clear() {}

    rcc.cfgr().modify(|_, w| unsafe {
        w.hpre().bits(0b0000)
         .ppre1().bits(0b100)
         .ppre2().bits(0b000)
         .adcpre().bits(0b10)
    });

    rcc.cfgr().modify(|_, w| unsafe {
        w.pllsrc().set_bit()
         .pllxtpre().clear_bit()
         .pllmul().bits(7)
    });

    rcc.cr().modify(|_, w| w.pllon().set_bit());
    while rcc.cr().read().pllrdy().bit_is_clear() {}

    rcc.cfgr().modify(|_, w| unsafe { w.sw().bits(0b10) });
    while rcc.cfgr().read().sws().bits() != 0b10 {}
}

// ===============================
// Minimal LCD driver (PCF8574 → HD44780 4bit)
// ===============================
const PCF8574_ADDR: u8 = 0x27;

pub struct Lcd<I2C> {
    i2c: I2C,
    addr: u8,
    backlight: u8,
}

impl<I2C, E> Lcd<I2C>
where
    I2C: Write<Error = E>,
    E: core::fmt::Debug,
{
    pub fn new(i2c: I2C, addr: u8) -> Self {
        Self { i2c, addr, backlight: 0x08 }
    }

    fn pulse_enable(&mut self, data: u8) -> Result<(), E> {
        self.i2c.write(self.addr, &[data | self.backlight | 0x04])?;
        asm::delay(8_000);
        self.i2c.write(self.addr, &[data | self.backlight])?;
        asm::delay(8_000);
        Ok(())
    }

    fn write4(&mut self, data: u8) -> Result<(), E> {
        self.pulse_enable(data)
    }

    fn send(&mut self, data: u8, rs: bool) -> Result<(), E> {
        let rs_bit = if rs { 0x01 } else { 0x00 };
        let high = (data & 0xF0) | rs_bit;
        let low  = ((data << 4) & 0xF0) | rs_bit;
        self.write4(high)?;
        self.write4(low)?;
        Ok(())
    }

    pub fn command(&mut self, cmd: u8) -> Result<(), E> {
        self.send(cmd, false)
    }

    pub fn write_char(&mut self, ch: u8) -> Result<(), E> {
        self.send(ch, true)
    }

    pub fn init(&mut self) -> Result<(), E> {
        self.write4(0x30)?; asm::delay(72_000);
        self.write4(0x30)?; asm::delay(72_000);
        self.write4(0x30)?; asm::delay(72_000);
        self.write4(0x20)?;

        self.command(0x28)?; // 2 lines, 5x8
        self.command(0x0C)?; // display on, cursor off
        self.command(0x06)?; // entry mode
        self.command(0x01)?; // clear
        asm::delay(200_000);
        Ok(())
    }

    pub fn set_cursor(&mut self, row: u8, col: u8) -> Result<(), E> {
        let addr = match row {
            0 => 0x80 + col,
            1 => 0xC0 + col,
            _ => 0x80,
        };
        self.command(addr)
    }

    pub fn write_str(&mut self, s: &str) -> Result<(), E> {
        for b in s.bytes() { self.write_char(b)?; }
        Ok(())
    }
}

// ===============================
// Fuzzy controller helpers
// ===============================

// Membership function (triangular)
fn mf_tri(x: f32, a: f32, b: f32, c: f32) -> f32 {
    if x <= a || x >= c { 0.0 }
    else if (x - b).abs() < 1e-6 { 1.0 }
    else if x < b { (x - a) / (b - a) }
    else { (c - x) / (c - b) }
}

fn fuzzify_error(e: f32) -> [f32; 7] {
    [
        mf_tri(e, -4000.0, -3000.0, -2000.0), // NB
        mf_tri(e, -3000.0, -2000.0, -1000.0), // NM
        mf_tri(e, -2000.0, -1000.0, 0.0),     // NS
        mf_tri(e, -500.0, 0.0, 500.0),        // ZE
        mf_tri(e, 0.0, 1000.0, 2000.0),       // PS
        mf_tri(e, 1000.0, 2000.0, 3000.0),    // PM
        mf_tri(e, 2000.0, 3000.0, 4000.0),    // PB
    ]
}

fn fuzzify_delta(de: f32) -> [f32; 5] {
    [
        mf_tri(de, -2000.0, -1000.0, -500.0), // NB
        mf_tri(de, -1000.0, -500.0, 0.0),     // NS
        mf_tri(de, -200.0, 0.0, 200.0),       // ZE
        mf_tri(de, 0.0, 500.0, 1000.0),       // PS
        mf_tri(de, 500.0, 1000.0, 2000.0),    // PB
    ]
}

fn fuzzify_rpm(rpm: f32) -> [f32; 3] {
    [
        mf_tri(rpm, 0.0, 500.0, 1000.0),       // Low
        mf_tri(rpm, 800.0, 1500.0, 2200.0),    // Medium
        mf_tri(rpm, 2000.0, 3000.0, 4000.0),   // High
    ]
}

fn fuzzy_inference(mu_err: [f32;7], mu_de: [f32;5], mu_rpm: [f32;3]) -> [f32;5] {
    let mut out = [0.0; 5]; // NB, NS, ZE, PS, PB

    // Core rules
    out[4] = f32::max(out[4], mu_err[6].min(mu_rpm[0])); // e PB & rpm Low → PB
    out[2] = f32::max(out[2], mu_err[3].min(mu_rpm[1])); // e ZE & rpm Med → ZE
    out[0] = f32::max(out[0], mu_err[2].min(mu_de[0]));  // e NS & de NB → NB
    out[3] = f32::max(out[3], mu_err[4].min(mu_de[4]));  // e PS & de PB → PS
    // Smoothing rule: e NM at High rpm → NS
    out[1] = f32::max(out[1], mu_err[1].min(mu_rpm[2]));
    // Safety: e NB at High rpm → NB
    out[0] = f32::max(out[0], mu_err[0].min(mu_rpm[2]));

    out
}

fn defuzzify(mu_out: [f32;5]) -> f32 {
    let values = [-120.0, -60.0, 0.0, 60.0, 120.0];
    let mut num = 0.0;
    let mut den = 0.0;
    for (i,&m) in mu_out.iter().enumerate() {
        num += m * values[i];
        den += m;
    }
    if den == 0.0 { 0.0 } else { num / den }
}


// ===============================
// Main program
// ===============================
#[entry]
fn main() -> ! {
    let dp = Peripherals::take().unwrap();
    setup_clocks();

    let mut rcc   = dp.RCC.constrain();
    let mut gpioa = dp.GPIOA.split(&mut rcc);
    let     gpiob = dp.GPIOB.split(&mut rcc);

    // I2C1 (PB6=SCL, PB7=SDA)
    let scl = gpiob.pb6;
    let sda = gpiob.pb7;
    let i2c = I2c::new(
        dp.I2C1,
        (scl, sda),
        stm32f1xx_hal::i2c::Mode::standard(100.kHz()),
        &mut rcc,
    );
    let mut lcd = Lcd::new(i2c, PCF8574_ADDR);
    lcd.init().unwrap();

    // ADC on PA2 (setpoint)
    let mut adc = Adc::new(dp.ADC1, &mut rcc);
    let mut pa2 = gpioa.pa2.into_analog(&mut gpioa.crl);

    // TIM2 as encoder (PA0/PA1) via PAC
    let _pa0 = gpioa.pa0.into_pull_up_input(&mut gpioa.crl);
    let _pa1 = gpioa.pa1.into_pull_up_input(&mut gpioa.crl);
    let rcc_regs = unsafe { &*pac::RCC::ptr() };
    rcc_regs.apb1enr().modify(|_, w| w.tim2en().set_bit());
    let tim2 = dp.TIM2;
    tim2.smcr().modify(|_, w| unsafe { w.sms().bits(3) }); // encoder mode 3
    tim2.ccmr1_input().modify(|_, w| unsafe {
        w.cc1s().bits(1)
         .ic1f().bits(0b0111)
         .cc2s().bits(1)
         .ic2f().bits(0b0111)
    });
    tim2.ccer().modify(|_, w| {
        w.cc1e().set_bit().cc1p().clear_bit()
         .cc2e().set_bit().cc2p().clear_bit()
    });
    tim2.arr().write(|w| unsafe { w.arr().bits(0xFFFF) });
    tim2.cnt().write(|w| unsafe { w.cnt().bits(0) });
    tim2.cr1().modify(|_, w| w.cen().set_bit());

    // TIM1 PWM CH1 on PA8 via PAC
    let _pa8 = gpioa.pa8.into_alternate_push_pull(&mut gpioa.crh);
    rcc_regs.apb2enr().modify(|_, w| w.tim1en().set_bit());
    let tim1 = dp.TIM1;
    tim1.psc().write(|w| unsafe { w.psc().bits(71) });   // 1 MHz
    tim1.arr().write(|w| unsafe { w.arr().bits(999) });  // 1 kHz
    tim1.ccmr1_output().modify(|_, w| unsafe {
        w.cc1s().bits(0)
         .oc1pe().set_bit()
         .oc1m().bits(0b110)
    });
    tim1.ccer().modify(|_, w| w.cc1e().set_bit());
    tim1.bdtr().modify(|_, w| w.moe().set_bit());
    tim1.cr1().modify(|_, w| w.arpe().set_bit().cen().set_bit());
    tim1.ccr1().write(|w| unsafe { w.ccr().bits(0) });
    tim1.egr().write(|w| w.ug().set_bit());

    // SysTick → 50ms
    let mut cp = cortex_m::Peripherals::take().unwrap();
    cp.SYST.set_clock_source(SystClkSource::Core);
    cp.SYST.set_reload(72_000 * 50 - 1); // 72 MHz → 50ms
    cp.SYST.clear_current();
    cp.SYST.enable_counter();

    // Constants and state
    const ARR: i32 = 999;
    // اگر TIM2 ×4 است، PPR موثر = CPR×4
    const PPR: i32 = 2400; // نمونه: CPR=600 ×4 → 2400
    let mut last_cnt: i32 = tim2.cnt().read().cnt().bits() as i32;

    // ----------------------------
    // Calibration: duty floor determination
    // ----------------------------
    lcd.set_cursor(0, 0).unwrap();
    lcd.write_str("Calibrating...").unwrap();
    lcd.set_cursor(1, 0).unwrap();
    lcd.write_str("Scanning Duty ").unwrap();

    // Kick to break static friction (~15%)
    let duty_kick: i32 = (ARR * 15) / 100;
    tim1.ccr1().write(|w| unsafe { w.ccr().bits(duty_kick as u16) });
    asm::delay(72_000_000 / 3);
    tim1.ccr1().write(|w| unsafe { w.ccr().bits(0) });
    asm::delay(72_000_000 / 10);

    // Scan duty to find real floor; stop at first movement
    let mut duty_min: i32 = 0;
    for d in (0..=400).step_by(20) {
        tim1.ccr1().write(|w| unsafe { w.ccr().bits(d as u16) });
        asm::delay(72_000_000 / 2); // ~0.5s

        let now_cnt = tim2.cnt().read().cnt().bits() as i32;
        let moved = (now_cnt - last_cnt).abs() > 2;
        last_cnt = now_cnt;

        if moved {
            duty_min = d;
            break;
        }
    }

    // If no movement detected, choose a reasonable floor (~15%) and hold
    if duty_min == 0 {
        duty_min = (ARR * 15) / 100;
        tim1.ccr1().write(|w| unsafe { w.ccr().bits(duty_min as u16) });
        asm::delay(72_000_000 * 2);
    }

    // ----------------------------
    // Main control loop: FF + Fuzzy + EMA + Stall Mode + Asym Slew
    // ----------------------------
    let mut rpm_avg_f: f32 = 0.0;
    let mut duty_prev: i32 = duty_min;

    // Normal params
    let mut beta_duty: f32 = 0.30;
    let slew_up_norm: i32 = 150;
    let slew_dn_norm: i32 = 12;

    // Stall detection
    let stall_enter_frac: f32 = 0.08;
    let stall_exit_frac: f32 = 0.25;
    let mut stall_count: u8 = 0;
    let stall_enter_n: u8 = 5;
    let mut stall_mode: bool = false;
    let mut stall_boost_step: i32 = 25;

    // RPM EMA
    let alpha_rpm: f32 = 0.40;
    let sample_t_s: f32 = 0.050;

    // Fuzzy state
    let mut err_prev: f32 = 0.0;
    // PID state and parameters
let mut integral_f: f32 = 0.0;
let kp_f: f32 = 1.0;
let ki_f: f32 = 0.06;
let ki_div_f: f32 = 150.0;
let mut pi_limit: i32 = 50;


    loop {
        if cp.SYST.has_wrapped() {
            // 1) Setpoint
            let adc_raw: u16 = adc.read(&mut pa2).unwrap_or(0);
            let setp_rpm: i32 = (adc_raw as i32 * 3000) / 4095;

            // 2) Encoder → RPM
            let now_cnt: i32 = tim2.cnt().read().cnt().bits() as i32;
            let mut delta = now_cnt - last_cnt;
            if delta > 32767 { delta -= 65536; }
            if delta < -32768 { delta += 65536; }
            last_cnt = now_cnt;

            let rpm_inst_f = (delta as f32 / PPR as f32) * (60.0 / sample_t_s);
            rpm_avg_f = alpha_rpm * rpm_inst_f + (1.0 - alpha_rpm) * rpm_avg_f;
            if rpm_avg_f < 0.0 { rpm_avg_f = 0.0; }

            // 3) Linear feedforward
            let duty_ff: i32 = if setp_rpm > 0 {
                duty_min + (setp_rpm * (ARR - duty_min)) / 3000
            } else { 0 };

            // 4) Fuzzy controller
 // 4) Hybrid: PID + Fuzzy correction
// 4) Hybrid: PID + Fuzzy correction
let err_f = setp_rpm as f32 - rpm_avg_f;
let delta_err_f = err_f - err_prev;
err_prev = err_f;

// --- PID ---
integral_f = (integral_f + err_f).clamp(-7000.0, 7000.0);
let u_pid = (kp_f * err_f + (ki_f * integral_f) / ki_div_f) as i32;
let u_pid_limited = u_pid.clamp(-pi_limit, pi_limit);

// --- Fuzzy ---
let mu_err = fuzzify_error(err_f);
let mu_de  = fuzzify_delta(delta_err_f);
let mu_rpm = fuzzify_rpm(rpm_avg_f);
let mu_out = fuzzy_inference(mu_err, mu_de, mu_rpm);
let u_fuzzy = defuzzify(mu_out) as i32;

// --- Combine ---
let mut duty_target = (duty_ff + u_pid_limited + u_fuzzy).clamp(0, ARR);

// Anti-windup only on PID when saturating (disabled in stall)
if !stall_mode && (duty_target <= 0 || duty_target >= ARR) {
    integral_f *= 0.7;
}



            // 5) Stall mode
            let near_stall = (setp_rpm > 0) && (rpm_avg_f < stall_enter_frac * setp_rpm as f32);
            if near_stall { stall_count = stall_count.saturating_add(1); } else { stall_count = 0; }

            if !stall_mode && stall_count >= stall_enter_n {
                stall_mode = true;
                beta_duty = 0.60;
                stall_boost_step = 30;
            }

            let exit_stall = stall_mode && (rpm_avg_f > stall_exit_frac * setp_rpm as f32);
            if exit_stall || setp_rpm == 0 {
                stall_mode = false;
                beta_duty = 0.30;
                stall_boost_step = 25;
            }

            // boost in stall
            let mut duty_apply_target = duty_target;
            if stall_mode {
                duty_apply_target = (duty_prev + stall_boost_step).max(duty_target);
                duty_apply_target = duty_apply_target.clamp(0, ARR);
            }

            // 6) Asymmetric slew
            let diff = duty_apply_target - duty_prev;
            let step = if stall_mode {
                if diff >= 0 { diff } else { diff.max(-slew_dn_norm) }
            } else {
                if diff >= 0 { diff.min(slew_up_norm) } else { diff.max(-slew_dn_norm) }
            };
            let duty_step = duty_prev + step;

            // 7) EMA on duty and floor
            let blend = (duty_step as f32 - duty_prev as f32) * beta_duty;
            let mut duty_apply = (duty_prev as f32 + blend) as i32;
            if setp_rpm > 0 { duty_apply = duty_apply.max(duty_min); }
            duty_apply = duty_apply.clamp(0, ARR);

            duty_prev = duty_apply;
            tim1.ccr1().write(|w| unsafe { w.ccr().bits(duty_apply as u16) });

            // 8) LCD
            let mut l0: heapless::String<20> = heapless::String::new();
            let _ = core::fmt::write(&mut l0, format_args!("SET:{:4} RPM:{:4}", setp_rpm, rpm_avg_f as i32));
            lcd.set_cursor(0,0).unwrap(); lcd.write_str(&l0).unwrap();

            let mut l1: heapless::String<20> = heapless::String::new();
            let _ = core::fmt::write(&mut l1, format_args!("{:0}% {:0}%", (duty_target*100)/ARR, (duty_apply*100)/ARR));
            lcd.set_cursor(1,0).unwrap(); lcd.write_str(&l1).unwrap();
        }
    }
}
//######################################################################
Hybrid fuzzy-PID overview
ترکیب فازی–PID را می‌توان به دو شکل عملی اجرا کرد:

Hybrid A: فازی فقط خروجی اصلاحی می‌دهد و با خروجی PID جمع می‌شود. پایدار و ساده.

Hybrid B: فازی به صورت آنلاین ضرایب PID (Kp, Ki, Kd) را تیون می‌کند. انعطاف‌پذیرتر برای شرایط متغیر.

برای شروع امن و قابل‌اعتماد، پیشنهاد می‌کنم Hybrid A را اجرا کنیم و بعد، اگر لازم بود، Hybrid B را اضافه کنیم.

Design goals and signals
Inputs:

Error: e = setpoint − rpm

Delta Error: de = e − e_prev

RPM: برای ناحیه کاری موتور (Low/Med/High)

Outputs:

u_pid: خروجی PID محدودشده

u_fuzzy: اصلاح فازی

duty_target = feedforward + clamp(u_pid + u_fuzzy)

Constraints:

Anti-windup برای PID وقتی اشباع رخ می‌دهد

Asymmetric slew همان‌طور که در کد فعلی داری

Stall mode همچنان فعال بماند، ولی اثر آن روی محدودیت‌ها و beta انجام شود

Fuzzy correction design
Membership sets:

Error: NB, NM, NS, ZE, PS, PM, PB (7 مجموعه)

Delta Error: NB, NS, ZE, PS, PB (5 مجموعه)

RPM: Low, Medium, High (3 مجموعه)

Rule logic (خلاصه و عملی):

e بزرگ + rpm پایین → خروجی PB (Boost برای شکست اصطکاک)

e کوچک ~ صفر + de بزرگ → خروجی کوچک (جلوگیری از overshoot)

e منفی در rpm بالا → خروجی NB (کاهش برای جلوگیری از اشباع)

Defuzzify: Weighted average با نگاشت خروجی‌ها به «تیک‌های duty» (مثلاً ±120 با ARR=999)

Hybrid A: code integration (drop-in)
این تکه‌ها را به فایل اضافه کن. توابع فازی بالا، سپس جایگزینی در حلقه اصلی. ضرایب PID فعلی را نگه می‌داریم اما با فازی جمع می‌کنیم.
//#########################################
Hybrid B: fuzzy gain scheduling (optional)
وقتی Hybrid A پایدار شد، می‌توانی فازی را برای تیون دینامیک ضرایب استفاده کنی:

Inputs: e, de, rpm

Outputs: scale_kp, scale_ki (و اگر لازم شد kd)

Gains: Kp_eff = Kp_base * scale_kp, Ki_eff = Ki_base * scale_ki

نمونه ساده قوانین:

e بزرگ + rpm پایین → scale_kp = High, scale_ki = Low (شروع تهاجمی با انتگرال کم)

e کوچک + de بزرگ → scale_kp = Low, scale_ki = Low (جلوگیری از overshoot)

rpm بالا → هر دو scale کاهش برای جلوگیری از نوسان

پیاده‌سازی مشابه است ولی به جای تولید u_fuzzy، scale‌ها را تولید و روی PID اعمال می‌کنی.

Tuning checklist
Scaling:

Membership ranges را با داده واقعی‌ات تطبیق بده (حداکثر RPM، اثر یک tick روی duty).

Magnitudes:

values خروجی فازی را با ARR متناسب کن. اگر duty پله‌ای می‌شود، مقادیر را کاهش بده.

Sampling:

sample_t_s را دقیق نگه دار؛ هر تغییری در SysTick روی سرعت محاسبات اثر دارد.

Safety:

اگر سرعت نزدیک صفر و duty بالا است، یک soft-cut اعمال کن (کاهش beta_duty و محدودیت slew_up).
