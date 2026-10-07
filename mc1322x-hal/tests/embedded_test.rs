//! On-target `embedded-test` tests for `mc1322x-hal`, run through `probe-rs run`.
//!
//! The board is reset before every `#[test]`, so each test starts from a clean boot and can
//! construct drivers that may only be created once per boot (e.g. `Aes::new`).
//!
//! Only the blocking APIs are tested (no executor), and only peripherals that need no external
//! wiring (no UART loopback, SPI/I2C device or second radio board).

#![no_std]
#![no_main]

#[cfg(test)]
#[embedded_test::tests]
mod tests {
    use embedded_hal::delay::DelayNs;
    use embedded_hal::pwm::SetDutyCycle;
    use mc1322x_hal::adc::Adc;
    use mc1322x_hal::aes::Aes;
    use mc1322x_hal::delay::Delay;
    use mc1322x_hal::pwm::Pwm;
    use mc1322x_hal::rng::{Rng, ensure_maca_ready};
    use rand_core::TryRng;

    #[test]
    fn trivial_pass() {
        assert_eq!(1 + 1, 2);
    }

    #[test]
    #[should_panic]
    fn trivial_fail() {
        assert_eq!(1 + 1, 3);
    }

    #[test]
    #[ignore = "deliberately faults: branches just past the end of RAM (0x0041_7FFF per the MC1322x Reference Manual) to raise a Prefetch Abort at a known address - the reference manual documents Prefetch Abort generation as only supported for RAM access and UART modules, so out-of-bounds RAM (unlike a peripheral or a fully unmapped address) should reliably fault"]
    fn zz_deliberate_prefetch_abort() {
        let f: extern "C" fn() = unsafe { core::mem::transmute(0x0041_8000usize) };
        f();
    }

    #[test]
    fn rtc_delay_runs() {
        // Touches real hardware (the RTC-based blocking delay), which also pulls in
        // `libmc1322x`'s ROM-patch-dependent code that the trivial tests don't reference.
        let mut delay = Delay::new();
        delay.delay_ms(10);
    }

    #[test]
    fn aes_ecb_known_answer() {
        // FIPS-197 Appendix C.1 AES-128 known-answer test.
        const KEY: [u8; 16] = [
            0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d,
            0x0e, 0x0f,
        ];
        const PLAINTEXT: [u8; 16] = [
            0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd,
            0xee, 0xff,
        ];
        const EXPECTED_CIPHERTEXT: [u8; 16] = [
            0x69, 0xc4, 0xe0, 0xd8, 0x6a, 0x7b, 0x04, 0x30, 0xd8, 0xcd, 0xb7, 0x80, 0x70, 0xb4,
            0xc5, 0x5a,
        ];

        let mut aes = Aes::new();
        let ciphertext = aes.ecb_encrypt_block(KEY, PLAINTEXT);
        assert_eq!(ciphertext, EXPECTED_CIPHERTEXT);
    }

    #[test]
    fn rng_reads_are_not_degenerate() {
        // Catch a stuck MACA_RANDOM register (always 0, or always the same value).
        ensure_maca_ready();
        let mut rng = Rng::new();

        let mut values = [0u32; 16];
        for v in values.iter_mut() {
            *v = rng.try_next_u32().unwrap();
        }

        assert!(
            values.iter().any(|&v| v != 0),
            "MACA_RANDOM read back all zero across {} reads",
            values.len()
        );
        assert!(
            values.iter().any(|&v| v != values[0]),
            "MACA_RANDOM returned the same value for every read"
        );
    }

    #[test]
    fn rng_seed_and_read_is_deterministic() {
        // `seed_and_read` must be deterministic regardless of prior LFSR state; a different seed
        // in between rules out passing by coincidence.
        ensure_maca_ready();
        let mut rng = Rng::new();

        const SEED: u32 = 0xdead_beef;
        let first = rng.seed_and_read(SEED);
        rng.seed_and_read(0x1234_5678);
        let second = rng.seed_and_read(SEED);

        assert_eq!(
            first, second,
            "seed_and_read(0x{SEED:08x}) was not reproducible across an intervening different seed"
        );
    }

    #[test]
    fn adc_internal_reference_reads_in_range() {
        // Channel 8 is the internal 1.2 V reference, the only channel that needs no external
        // wiring.
        let mut adc = Adc::new();
        for _ in 0..5 {
            let value = adc.read(8);
            assert!(
                value <= 4095,
                "channel 8 (internal 1.2V reference) sample {value} exceeds the ADC's 12-bit range"
            );
        }
    }

    #[test]
    fn pwm_set_duty_cycle_runs_the_timer() {
        // `Pwm` has no getter, so read TMR1's CNTR register directly to check the timer counts.
        const TIMER: u8 = 1;

        let mut pwm = Pwm::new(TIMER, 1000);
        pwm.set_duty_cycle(32768).unwrap();

        let read_cntr = || unsafe {
            let base = (mc1322x_sys::TMR_BASE + mc1322x_sys::TMR_OFFSET * TIMER as u32) as usize;
            ((base + 0x0a) as *const u16).read_volatile()
        };

        let before = read_cntr();
        for _ in 0..200_000u32 {
            core::hint::black_box(0);
        }
        let after = read_cntr();

        assert_ne!(
            before, after,
            "TMR{TIMER} CNTR did not advance after set_duty_cycle - the PWM timer isn't running"
        );
    }
}
