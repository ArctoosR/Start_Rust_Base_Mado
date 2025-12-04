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

use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyle},
    pixelcolor::BinaryColor,
    prelude::*,
    text::Text,
};

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
    let mut gpiob = dp.GPIOB.split(&mut rcc);

    // I2C1 (PB6=SCL, PB7=SDA) برای SSD1306
    let scl = gpiob.pb6.into_alternate_open_drain(&mut gpiob.crl);
    let sda = gpiob.pb7.into_alternate_open_drain(&mut gpiob.crl);
    let i2c = I2c::new(
        dp.I2C1,
        (scl, sda),
        stm32f1xx_hal::i2c::Mode::standard(100.kHz()),
        &mut rcc,
    );

    // SSD1306 OLED init (128x64)
    let interface = I2CDisplayInterface::new(i2c);
    let mut disp = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    disp.init().unwrap();
    disp.clear();
    disp.set_display_on(true).ok();
    disp.flush().unwrap();

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
    let style = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);
    disp.clear();
    Text::new("Calibrating...", Point::new(0, 8), style).draw(&mut disp).ok();
    Text::new("Scanning Duty", Point::new(0, 24), style).draw(&mut disp).ok();
    disp.flush().ok();

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

        // نمایش پروگرس کالیبراسیون
        let mut l0: String<32> = String::new();
        let _ = core::fmt::write(&mut l0, format_args!("Scan d={:3}", d));
        disp.clear();
        Text::new(&l0, Point::new(0, 8), style).draw(&mut disp).ok();
        Text::new(if moved { "Move detected" } else { "No move" }, Point::new(0, 16), style)
            .draw(&mut disp).ok();
        disp.flush().ok();

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

    // Fuzzy/PID state and parameters
    let mut err_prev: f32 = 0.0;
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

            // 8) OLED نمایش پارامترها
            disp.clear();

            // خط 1: Setpoint و RPM
            let mut l0: String<32> = String::new();
            let _ = core::fmt::write(&mut l0, format_args!("SET:{:4} RPM:{:4}", setp_rpm, rpm_avg_f as i32));
            Text::new(&l0, Point::new(0, 8), style).draw(&mut disp).ok();

            // خط 2: Duty target/apply و حالت Stall
            let mut l1: String<32> = String::new();
            let _ = core::fmt::write(
                &mut l1,
                format_args!("DT:{:3}% DA:{:3}% {}", (duty_target*100)/ARR, (duty_apply*100)/ARR, if stall_mode {"STL"} else {""})
            );
            Text::new(&l1, Point::new(0, 24), style).draw(&mut disp).ok();

            // خط 3: PID/Fuzzy خروجی‌ها برای دیباگ
            let mut l2: String<32> = String::new();
            let _ = core::fmt::write(&mut l2, format_args!("PID:{:4} FZ:{:4}", u_pid_limited, u_fuzzy));
            Text::new(&l2, Point::new(0, 40), style).draw(&mut disp).ok();

            // خط 4: Min duty و PI limit (دیباگ)
            let mut l3: String<32> = String::new();
            let _ = core::fmt::write(&mut l3, format_args!("dmin:{:3} pilim:{:3}", duty_min, pi_limit));
            Text::new(&l3, Point::new(0, 56), style).draw(&mut disp).ok();

            disp.flush().ok();
        }
    }
}

//##############################################################
🖥️ اتصالات نمایشگر SSD1306 (I²C)
PB6 → SCL (خط کلاک I²C)

PB7 → SDA (خط داده I²C)

هر دو پین به حالت Alternate Open-Drain تنظیم شده‌اند.

نیاز به مقاومت‌های Pull-up (معمولاً 4.7kΩ به 3.3V) روی SDA و SCL.

تغذیه نمایشگر: VCC = 3.3V ، GND = زمین.

⚡️ ورودی تنظیم سرعت (Setpoint)
PA2 → ADC1_IN2

به حالت Analog تنظیم شده.

این پین ولتاژ مرجع (مثلاً از یک پتانسیومتر) را می‌خواند و به RPM هدف تبدیل می‌شود.

🔄 انکودر موتور (Feedback RPM)
PA0 → TIM2_CH1

PA1 → TIM2_CH2

هر دو پین به حالت Pull-up Input تنظیم شده‌اند.

تایمر TIM2 در حالت Encoder Mode 3 قرار دارد و شمارش پالس‌های انکودر را انجام می‌دهد.

🔊 خروجی PWM (کنترل موتور)
PA8 → TIM1_CH1

به حالت Alternate Push-Pull تنظیم شده.

تایمر TIM1 روی فرکانس 1 kHz تنظیم شده و Duty Cycle روی رجیستر CCR1 نوشته می‌شود.

این پین به درایور موتور (مثلاً MOSFET یا ماژول H-Bridge) وصل می‌شود.

🕒 تایمر سیستم (SysTick)
از کلاک 72 MHz استفاده می‌کند.

هر 50ms یک بار وقفه می‌دهد تا حلقه‌ی کنترل اجرا شود.

📋 جمع‌بندی اتصالات
ماژول / سیگنال	پین STM32	حالت پین	توضیح
SSD1306 SCL	PB6	Alternate Open-Drain	خط کلاک I²C
SSD1306 SDA	PB7	Alternate Open-Drain	خط داده I²C
SSD1306 VCC	3.3V	—	تغذیه نمایشگر
SSD1306 GND	GND	—	زمین
Setpoint (ADC)	PA2	Analog	ورودی ولتاژ هدف RPM
Encoder A	PA0	Input Pull-up	کانال 1 انکودر
Encoder B	PA1	Input Pull-up	کانال 2 انکودر
PWM Output	PA8	Alternate Push-Pull	خروجی PWM به درایور موتور

