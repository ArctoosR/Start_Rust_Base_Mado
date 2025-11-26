#![deny(unsafe_code)]
#![allow(clippy::empty_loop)]
#![no_main]
#![no_std]

use panic_halt as _;

use cortex_m::asm;
use cortex_m_rt::entry;
use stm32f1xx_hal::{
    pac,
    prelude::*,
    time::ms,
    timer::{Channel, Tim2NoRemap},
};

#[entry]
fn main() -> ! {
    let p = pac::Peripherals::take().unwrap();

    let mut rcc = p.RCC.constrain();
    let mut afio = p.AFIO.constrain(&mut rcc);

    let mut gpioa = p.GPIOA.split(&mut rcc);
    let mut gpioc = p.GPIOC.split(&mut rcc);

    // TIM2 PWM pins
    let c1 = gpioa.pa0.into_alternate_push_pull(&mut gpioa.crl);
    let c2 = gpioa.pa1.into_alternate_push_pull(&mut gpioa.crl);
    let c3 = gpioa.pa2.into_alternate_push_pull(&mut gpioa.crl);
    let pins = (c1, c2, c3);

    // ساخت PWM روی تایمر 2
    let mut pwm = p
        .TIM2
        .pwm_hz::<Tim2NoRemap, _, _>(pins, &mut afio.mapr, 1.kHz(), &mut rcc);

    pwm.enable(Channel::C1);
    pwm.enable(Channel::C2);
    pwm.enable(Channel::C3);

    let max = pwm.get_max_duty();
    pwm.set_duty(Channel::C3, max / 2); // 50% duty روی PA2

    // LED داخلی روی PC13
    let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);

    loop {
        // Blink LED
        led.set_low();  // روشن (روی Blue Pill LED active-low است)
        asm::delay(8_000_000);

        led.set_high(); // خاموش
        asm::delay(8_000_000);

        // می‌توانی همزمان duty PWM را تغییر بدهی
        // pwm.set_duty(Channel::C3, (max / 4));
    }
}
