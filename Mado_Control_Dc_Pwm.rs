//cargo run --release --example Mado_Control_Dc_Pwm_9 --features stm32f103 --target thumbv7m-none-eabi
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
use libm::sqrt;

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
// Simple statistics helpers
// ===============================
fn calc_stddev(data: &[i32]) -> f32 {
    let n = data.len() as f32;
    if n == 0.0 { return 0.0; }
    let mean: f32 = data.iter().map(|&x| x as f32).sum::<f32>() / n;
    let var: f32 = data.iter()
        .map(|&x| {
            let d = x as f32 - mean;
            d * d
        })
        .sum::<f32>() / n;
    sqrt(var as f64) as f32
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
    // توجه: اگر TIM2 در حالت ×4 است، PPR موثر = CPR×4. این مقدار را مطابق انکودر واقعی تنظیم کن.
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
    // Main control loop: FF + PI + EMA + Stall Mode + Asym Slew
    // ----------------------------
    let mut rpm_avg_f: f32 = 0.0;
    let mut integral_f: f32 = 0.0;

    let kp_f: f32 = 1.0;
    let ki_f: f32 = 0.06;
    let ki_div_f: f32 = 150.0;

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

    // PI limit
    let mut pi_limit: i32 = 50;

    // RPM EMA
    let alpha_rpm: f32 = 0.40;
    let sample_t_s: f32 = 0.050;

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

            // 4) PI limited
            let err_f = setp_rpm as f32 - rpm_avg_f;
            integral_f = (integral_f + err_f).clamp(-7000.0, 7000.0);
            let duty_corr = (kp_f * err_f + (ki_f * integral_f) / ki_div_f) as i32;
            let duty_corr_limited = duty_corr.clamp(-pi_limit, pi_limit);

            let mut duty_target = (duty_ff + duty_corr_limited).clamp(0, ARR);

            // anti-windup when saturating (disabled in stall)
            if !stall_mode && (duty_target <= 0 || duty_target >= ARR) {
                integral_f *= 0.7;
            }

            // 5) Stall mode
            let near_stall = (setp_rpm > 0) && (rpm_avg_f < stall_enter_frac * setp_rpm as f32);
            if near_stall { stall_count = stall_count.saturating_add(1); } else { stall_count = 0; }

            if !stall_mode && stall_count >= stall_enter_n {
                stall_mode = true;
                pi_limit = 200;
                beta_duty = 0.60;
                stall_boost_step = 30;
            }

            let exit_stall = stall_mode && (rpm_avg_f > stall_exit_frac * setp_rpm as f32);
            if exit_stall || setp_rpm == 0 {
                stall_mode = false;
                pi_limit = 50;
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

//##############################################################

[ENCODER] A ───── PA0 (TIM2_CH1)
          B ───── PA1 (TIM2_CH2)
          GND ──── GND مشترک
          Vcc ──── 12–24V جداگانه (سطح‌سازی خروجی به 3.3V/5V)

[LCD I2C] SCL ─── PB6 (I2C1)
          SDA ─── PB7 (I2C1)
          Vcc ─── 5V
          GND ─── GND مشترک

[PWM OUT] PA8 ─── Driver → Motor

[POT 10k] 3.3V ─ سمت 1
          GND  ─ سمت 2
          PA2  ─ وسط (Wiper)


//########################################################################

// جمع‌بندی مسئله
// رفتارهای گزارش‌شده—پرش شبیه استپر، جهش Duty و توقف ناگهانی، گیرکردن روی کف Duty، و سپس پایدار شدن با دقت بهتر—نشان می‌دهد ریشه‌ی مشکل ترکیبی از سه عامل بوده: مقیاس فیدبک نادرست (PPR/Ts)، محافظ‌های خروجی و فیلترها که اشتباه زمان‌بندی شده بودند، و فقدان کالیبراسیون حداقلی برای کف حرکت.

// ریشه‌های اصلی مشکل
// PPR مؤثر اشتباه یا ناهماهنگ با مد انکودر:

// در Encoder Mode 3 شمارش ×4 فعال است؛ اگر CPR واقعی در کد لحاظ نشود، RPM محاسبه می‌پرد یا نزدیک صفر می‌شود.

// نشانه‌ها: DELTA خیلی بزرگ (مثل 6000 در 100ms) یا RPM غیرواقعی، جهش Duty به بالا سپس سقوط.

// Ts واقعی با حلقه کنترل هم‌خوان نبود:

// با SysTick نادرست یا فرض Ts غلط، ضریب محاسبه RPM خطا را بزرگ/کوچک می‌کند و PI را ناپایدار می‌سازد.

// Guard کف و محدودیت‌های خروجی باعث قفل‌شدن Duty:

// نگه‌داشتن Duty روی dmin+margin به‌صورت دائمی اجازه نمی‌داد Duty با ست‌پوینت بالا برود؛ نتیجه: با تغییر ولوم سرعت تغییر نمی‌کرد.

// محدودیت‌های PI و EMA روی Duty در حالت گیرکردن بیش از حد محافظه‌کار بودند؛ فقط ~۸ واحد افزایش می‌دیدی.

// نبود حالت اختصاصی برای گیرکردن (Stall):

// در RPM بسیار پایین، کنترل نیاز دارد محدودیت‌ها را موقتاً شُل کند تا Duty بتواند تا سقف برود؛ نبود این منطق باعث ماندن در کف می‌شد.

// شواهد رفتاری که به تشخیص کمک کردند
// DELTA≈6000 در 100ms → PPR_eff بزرگ‌تر و Mode×4: نشان داد مقیاس RPM اشتباه بود و باید PPR اصلاح شود.

// SET تغییر می‌کرد اما Duty ثابت≈19٪: Guard کف به‌صورت دائمی اعمال شده بود؛ خروجی قفل شده بود.

// Duty فقط ~۸ واحد در مانع بلند می‌شد: PI محدود و EMA خروجی در Stall بیش از حد سخت بود؛ نیاز به Stall Mode برای برداشتن محدودیت‌ها.

// چه چیزها مشکل را حل کردند
// اصلاح مقیاس فیدبک: تعیین PPR_eff با یک دور واقعی و هم‌خوان کردن با Encoder Mode×4؛ تأیید Ts=50ms.

// بازطراحی محافظ‌ها: کف فقط حداقل، نه قفل؛ Slew نامتقارن (بالا سریع، پایین آهسته) و EMA روی Duty با بتا مناسب.

// Stall Mode هوشمند: تشخیص گیرکردن با آستانه درصدی از ست‌پوینت، افزایش pi_limit و beta_duty، و بوست پیشرونده تا رفع سکون.

// تیونینگ ملایم PI و EMA روی RPM: kp/ki کوچک اما کافی، alpha_rpm بالاتر برای پاسخ سریع بدون نوسان.

// چک‌لیست پیشگیری برای پروژه‌های بعدی
// کالیبراسیون سریع در بوت:

// dmin: رمپ کوتاه تا اولین حرکت پایدار؛ ذخیره و استفاده در FF.

// PPR_eff: یک دور واقعی، ذخیره بر اساس Mode×4.

// Ts: تأیید با GPIO/اسکوپ و به‌روزکردن ضریب RPM.

// اعتبارسنجی مسیر فیدبک:

// نمایش همزمان CNT/DELTA/RPM روی LCD در حالت تست. اگر RPM غیرمنطقی است، ابتدا PPR/Ts را اصلاح کن.

// محافظ‌های خروجی درست‌کار:

// کف فقط حداقل؛ بدون margin دائمی.

// Slew نامتقارن و قابل‌تعطیل در افزایش ست‌پوینت یا خطای بزرگ.

// EMA بر Duty با بتا 0.25–0.35؛ در Stall بتا را افزایش بده.

// Stall Mode استاندارد:

// آستانه ورود/خروج مبتنی بر درصد ست‌پوینت (مثلاً 8% و 25%).

// pi_limit و boost-step قابل تیون؛ خروجی تا سقف با مراقبت از جریان/حرارت.

// تیونینگ سریع و قابل مشاهده:

// جدول‌ها و لاگ‌های LCD: SET, RPM, DT/DA.

// گام‌به‌گام: اول FF، سپس EMA، بعد PI محدود، در نهایت Slew و Stall.

//###################################################################
// تاثیر کوپل مستقیم انکودر روی شفت موتور
// وقتی ENC-1-2-T-24 را بدون چرخ و مستقیم به شفت موتور کوپل می‌کنی، اندازه‌گیری از «سرعت خطی» به «سرعت زاویه‌ای (RPM)» تغییر می‌کند. بنابراین:

// پارامتر محیط چرخ و نسبت گیربکس داخلی دیگر معنی ندارد.

// باید فقط روی CPR/PPR انکودر و مد شمارش تایمر (×1/×2/×4) تمرکز کنی.

// خروجی DELTA مستقیماً پالس بر دور شفت را نشان می‌دهد و باید درست مقیاس شود.

// آنچه باید قطعی کنی
// مشخص کن انکودرت دقیقاً چند پالس خام در هر دور می‌دهد (CPR). در مدل‌های سری ENC-1-2 معمولاً CPR مشخص است (مثل 100، 200، 360، 600… بسته به نسخه). چون تو چرخ را حذف کرده‌ای، فقط CPR خود انکودر مهم است.

// مد شمارش تایمر را بررسی کن. در Encoder Mode 3 معمولاً ×4 می‌شمارد. پس:

// PPR_eff = CPR × 4 (اگر ×4 فعال است)

// PPR_eff = CPR × 2 (اگر ×2)

// PPR_eff = CPR × 1 (اگر ساده)

// اگر به CPR کاتالوگ دسترسی نداری، بهترین راه «اندازه‌گیری یک دور» است:

// CNT را صفر کن، شفت را دقیقاً یک دور بچرخان، مقدار CNT را بخوان → این می‌شود PPR_eff واقعی.

// تغییرات لازم در کد
// 1) جایگزین کردن PPR با مقدار مؤثر جدید
// در محاسبه RPM:

// rpm_inst = (delta / PPR_eff) × (60 / Ts)

// اگر قبلاً از مقدار 2400 به‌عنوان نمونه استفاده کرده‌ای، آن را با مقدار واقعی جایگزین کن. نمونه:

// rust
// // بعد از اندازه‌گیری یک دور
// const PPR: i32 = MEASURED_PPR_EFF; // مثلاً 400، 1440، 2400...
// let rpm_inst_f = (delta as f32 / PPR as f32) * (60.0 / sample_t_s);
// 2) حذف تبدیل‌های خطی مبتنی بر چرخ
// هرجا سرعت خطی (mm/s یا cm/s) حساب شده بود، حذف کن. دیگر تنها RPM کافی است.

// نگاشت فیدفوروارد را بر اساس ست‌پوینت RPM حفظ کن:

// rust
// let duty_ff = if setp_rpm > 0 {
//     duty_min + (setp_rpm * (ARR - duty_min)) / RPM_MAX  // RPM_MAX: سقف دلخواه
// } else { 0 };
// 3) بازکالیبراسیون dmin
// چون معیار «اولین حرکت پایدار» حالا بر اساس DELTA زاویه‌ای شفت است، آستانه حرکت کمی فرق می‌کند. یک‌بار quick-scan:

// Duty را از 10–30% بالا ببر؛ جایی که DELTA به‌صورت پایدار غیرصفر شد را dmin ذخیره کن.

// 4) بررسی آستانه‌های Stall
// چون رزولوشن تغییر کرده، «نسبت RPM به ست‌پوینت» همچنان معیار خوبی است. آستانه‌های ورود/خروج (۸٪ و ۲۵٪) را می‌توان ثابت نگه داشت.

// اگر DELTA نویزی شد، alpha_rpm را کمی پایین بیاور (0.35) یا Ts را 75ms کن.

// جدول تغییرات رایج
// وضعیت	CPR انکودر	مد شمارش تایمر	PPR_eff در کد	نوع سرعت
// چرخ‌دار (Wheel)	وابسته به مدل	×4	پالس بر دور چرخ	خطی (mm/s, cm/s)
// شفت مستقیم (Shaft)	CPR واقعی انکودر	×4	CPR × 4	زاویه‌ای (RPM)
// شفت مستقیم (Shaft)	CPR واقعی انکودر	×2	CPR × 2	زاویه‌ای (RPM)
// شفت مستقیم (Shaft)	CPR واقعی انکودر	×1	CPR × 1	زاویه‌ای (RPM)
// نکات عملی برای کاهش مراحل در پروژه‌های بعدی
// در بوت، یک روتین کوتاه اجرا کن:

// اندازه‌گیری یک دور → ثبت PPR_eff

// quick-scan dmin → ثبت کف واقعی

// تأیید Ts با GPIO toggle (فقط یک‌بار)

// سپس این مقادیر را در فلش ذخیره کن تا دیگر دست‌کاری دستی لازم نباشد.

// همیشه روی LCD سه کمیت را نشان بده: SET، RPM، DT/DA. اگر RPM غیرمنطقی شد، اول PPR_eff و Ts را بررسی کن، نه پارامترهای PI.
