//! [`Monotonic`](rtic_time::Monotonic) implementation for the Real Time
//! Clock (RTC).
//!
//! TODO: Somewhere make a note about u32 implementation limited to 36 hours and
//! 24 minutes
//!
//! TODO: Mention v1 and v2.
//! TODO: Have prelude for monotonic that includes extension traits like in
//! `rtic-monotonics`?
//!
//! # Example
//!
//! ```
//! use atsamd_hal::rtc::rtic::prelude::*;
//! rtc_monotonic!(Mono);
//!
//! fn init() {
//!     # // This is normally provided by the selected PAC
//!     # let rtc = unsafe { core::mem::transmute(()) };
//!     # let mut mclk = unsafe { core::mem::transmute(()) };
//!     // Start the monotonic
//!     Mono::start(rtc, &mut mclk);
//! }
//!
//! async fn usage() {
//!     loop {
//!          // Use the monotonic
//!          let timestamp = Mono::now();
//!          Mono::delay(100.millis()).await;
//!     }
//! }
//! ```

use super::{Count32Mode, Rtc};
use atsamd_hal_macros::hal_macro_helper;

pub use v2::*;

/// The RTC clock frequency in Hz.
pub const CLOCK_FREQ: u32 = 32_768;

mod v1 {
    use super::*;
    use rtic_monotonic::Monotonic;

    type Instant = fugit::Instant<u32, 1, CLOCK_FREQ>;
    type Duration = fugit::Duration<u32, 1, CLOCK_FREQ>;

    impl Monotonic for Rtc<Count32Mode> {
        type Instant = Instant;
        type Duration = Duration;
        unsafe fn reset(&mut self) {
            // Since reset is only called once, we use it to enable the interrupt generation
            // bit.
            self.mode0().intenset().write(|w| w.cmp0().set_bit());
        }

        fn now(&mut self) -> Self::Instant {
            Self::Instant::from_ticks(self.count32())
        }

        fn zero() -> Self::Instant {
            Self::Instant::from_ticks(0)
        }

        fn set_compare(&mut self, instant: Self::Instant) {
            unsafe {
                self.mode0()
                    .comp(0)
                    .write(|w| w.comp().bits(instant.ticks()))
            }
        }

        fn clear_compare_flag(&mut self) {
            self.mode0().intflag().write(|w| w.cmp0().set_bit());
        }
    }
}

mod v2 {
    use core::u32;

    use super::*;
    use crate::{pac, timer_traits::InterruptDrivenTimer};
    use atsamd51j::rtc::mode0;
    use fugit::HertzU32;
    use rtic_time::timer_queue::{TimerQueue, TimerQueueBackend};

    #[doc(hidden)]
    #[macro_export]
    macro_rules! __internal_create_rtc_interrupt {
        ($mono_backend:ident, $interrupt_rtc:ident) => {
            #[no_mangle]
            #[allow(non_snake_case)]
            unsafe extern "C" fn $interrupt_rtc() {
                use $crate::rtic_time::timer_queue::TimerQueueBackend;

                let rtc = unsafe { $crate::pac::Rtc::steal() };

                // For some strange reason that was unable to be determined, often the compare
                // interrupt will trigger, but the count will be less than the compare value,
                // even after syncing, which causes the TimerQueue to not register that the
                // timeout is up. There is nothing about this in the chip errata or
                // documentation.
                //
                // Testing showed that usually the count is only one less than
                // the compare. We correct for this here by waiting until the counter reaches
                // the compare value.
                if rtc.mode0().intflag().read().cmp0().bit_is_set() {
                    let compare = rtc.mode0().comp(0).read().bits();
                    loop {
                        let counter = $crate::rtc::rtic::$mono_backend::now();

                        if counter >= compare {
                            break;
                        }
                    }
                }

                $crate::rtc::rtic::$mono_backend::timer_queue().on_monotonic_interrupt();
            }
        };
    }

    #[doc(hidden)]
    #[macro_export]
    macro_rules! __internal_create_rtc_struct {
        ($name:ident, $mono_backend:ident, $pac_rtc:ty, $interrupt_rtc:ident) => {
            /// A `Monotonic` based on the RTC peripheral.
            pub struct $name;

            impl $name {
                /// Starts the `Monotonic`.
                ///
                /// This method must be called only once.
                pub fn start(
                    rtc: $pac_rtc,
                    mclk: &mut $crate::pac::Mclk,
                    osc32kctrl: &mut $crate::pac::Osc32kctrl,
                ) {
                    $crate::__internal_create_rtc_interrupt!($mono_backend, $interrupt_rtc);

                    $crate::rtc::rtic::$mono_backend::_start(rtc, mclk, osc32kctrl);
                }
            }

            impl $crate::rtic_time::monotonic::TimerQueueBasedMonotonic for $name {
                type Backend = $crate::rtc::rtic::$mono_backend;
                type Instant = $crate::fugit::Instant<
                    <Self::Backend as $crate::rtic_time::timer_queue::TimerQueueBackend>::Ticks,
                    1,
                    1_024,
                >;
                type Duration = $crate::fugit::Duration<
                    <Self::Backend as $crate::rtic_time::timer_queue::TimerQueueBackend>::Ticks,
                    1,
                    1_024,
                >;
                /* type Instant = $crate::fugit::Instant<
                    <Self::Backend as $crate::rtic_time::timer_queue::TimerQueueBackend>::Ticks,
                    1,
                    32_768,
                >;
                type Duration = $crate::fugit::Duration<
                    <Self::Backend as $crate::rtic_time::timer_queue::TimerQueueBackend>::Ticks,
                    1,
                    32_768,
                >; */
            }

            $crate::rtic_time::impl_embedded_hal_delay_fugit!($name);
            $crate::rtic_time::impl_embedded_hal_async_delay_fugit!($name);
        };
    }

    /// Create an RTC based monotonic for RTIC and register the RTC interrupt
    /// for it.
    ///
    /// See the [`rtc`](crate::rtc) module for more details.
    #[macro_export]
    macro_rules! rtc_monotonic {
        ($name:ident) => {
            $crate::__internal_create_rtc_struct!($name, RtcBackend, $crate::pac::Rtc, RTC);
        };
    }

    struct TimerValueU32(u32);
    impl rtic_time::half_period_counter::TimerValue for TimerValueU32 {
        const BITS: u32 = 32;
    }
    impl From<TimerValueU32> for u64 {
        fn from(value: TimerValueU32) -> Self {
            Self::from(value.0)
        }
    }

    /// RTC based [`TimerQueueBackend`].
    pub struct RtcBackend;

    static RTC_TQ: TimerQueue<RtcBackend> = TimerQueue::new();

    #[hal_macro_helper]
    impl RtcBackend {
        #[inline]
        fn sync(rtc: &pac::Rtc) {
            #[hal_cfg("rtc-d5x")]
            while rtc.mode2().syncbusy().read().bits() != 0 {}
            #[hal_cfg(any("rtc-d11", "rtc-d21"))]
            while rtc.mode2().status().read().syncbusy().bit_is_set() {}
        }

        /// Starts the clock.
        ///
        /// **Do not use this function directly.**
        ///
        /// Use the [`rtc_monotonic`] macro instead.
        pub fn _start(rtc: pac::Rtc, mclk: &mut pac::Mclk, osc32kctrl: &mut pac::Osc32kctrl) {
            // Enables the APBA clock for the RTC.
            mclk.apbamask().modify(|_, w| w.rtc_().set_bit());

            // It is necessary to disable the RTC before resetting it.
            // NOTE: This register and field are the same in all modes.
            rtc.mode0().ctrla().modify(|_, w| w.enable().clear_bit());
            Self::sync(&rtc);

            // Initialize the timer queue
            // TODO: When should we actually do this?
            RTC_TQ.initialize(Self {});

            // Reset RTC back to initial settings, which disables it and enters mode 0.
            rtc.mode0().ctrla().modify(|_, w| w.swrst().set_bit());
            // Wait for the reset to complete
            while rtc.mode0().ctrla().read().swrst().bit_is_set() {}

            // Set the RTC clock source.
            // TODO: How to we set this generically so that the user can choose?
            osc32kctrl.rtcctrl().write(|w| w.rtcsel().ulp1k());
            //osc32kctrl.rtcctrl().write(|w| w.rtcsel().ulp32k());

            // Set the compare for a long time in the future.
            // NOTE: Setting it to zero would immediately trigger an interrupt on the next
            // tick. TODO: Will eventually need half periods
            unsafe {
                rtc.mode0().comp(0).write(|w| w.comp().bits(u32::MAX));
            }
            Self::sync(&rtc);

            // Enable the compare interrupt.
            rtc.mode0().intenset().write(|w| w.cmp0().set_bit());

            // Start the RTC counter.
            rtc.mode0().ctrla().modify(|_, w| w.enable().set_bit());
            Self::sync(&rtc);

            // Enable counter sync on SAMx5x, the counter cannot be read otherwise.
            #[hal_cfg("rtc-d5x")]
            {
                rtc.mode0().ctrla().modify(|_, w| w.countsync().set_bit());

                // Errata: The first read of the count is incorrect so we need to read it then
                // wait for it to change.
                let count = Self::now();
                while Self::now() == count {}
            }

            // TODO: TEST CODE that checks whether setting an compare value lower than the
            // count will trigger an interrupt (does not).
            /* while Self::count(&rtc) < 2048 {}

            unsafe {
                rtc.mode0().comp(0).write(|w| w.comp().bits(2000));
            }
            Self::sync(&rtc);

            while rtc.mode0().intflag().read().cmp0().bit_is_clear() {}
            panic!(); */

            // Enable the RTC interrupt in the NVIC and set its priority.
            // TODO: If we keep this in the HAL is there a different way to do
            // it that does not involve copying the
            // `set_monotonic_prio` from `rtic-monotonics`?
            unsafe {
                set_monotonic_prio(pac::NVIC_PRIO_BITS, pac::Interrupt::RTC);
                pac::NVIC::unmask(pac::Interrupt::RTC);
            }
        }
    }

    impl TimerQueueBackend for RtcBackend {
        type Ticks = u32;

        #[hal_macro_helper]
        fn now() -> Self::Ticks {
            let rtc = unsafe { &pac::Rtc::steal() };

            #[hal_cfg(any("rtc-d11", "rtc-d21"))]
            {
                rtc.mode0().readreq().modify(|_, w| w.rcont().set_bit());
                Self::sync(rtc);
            }
            // NOTE: Sync is automatic on SAMD5x chips.
            rtc.mode0().count().read().bits()
        }

        fn enable_timer() {
            let rtc = unsafe { pac::Rtc::steal() };
            rtc.mode0().ctrla().modify(|_, w| w.enable().set_bit());
            Self::sync(&rtc);
        }

        fn disable_timer() {
            let rtc = unsafe { pac::Rtc::steal() };
            rtc.mode0().ctrla().modify(|_, w| w.enable().clear_bit());
            Self::sync(&rtc);
        }

        fn on_interrupt() {
            // TODO: We will eventually need period/overflow stuff handled here
        }

        fn set_compare(mut instant: Self::Ticks) {
            let rtc = unsafe { pac::Rtc::steal() };

            // Evidently the compare interrupt will not trigger if the instant is within a
            // couple of ticks, so delay it a bit if it is too close.
            // This is not mentioned in the documentation or errata, but is known to be an
            // issue for other microcontrollers as well (e.g. nRF family).
            if instant.saturating_sub(Self::now()) < 5 {
                instant = instant.wrapping_add(5)
            }

            unsafe { rtc.mode0().comp(0).write(|w| w.comp().bits(instant)) };
            Self::sync(&rtc);
        }

        fn clear_compare_flag() {
            let rtc = unsafe { pac::Rtc::steal() };
            rtc.mode0().intflag().modify(|_, w| w.cmp0().set_bit());
            // NOTE: Should not need to sync here
        }

        fn pend_interrupt() {
            pac::NVIC::pend(pac::Interrupt::RTC);
        }

        fn timer_queue() -> &'static TimerQueue<Self> {
            &RTC_TQ
        }
    }

    const fn cortex_logical2hw(logical: u8, nvic_prio_bits: u8) -> u8 {
        ((1 << nvic_prio_bits) - logical) << (8 - nvic_prio_bits)
    }

    unsafe fn set_monotonic_prio(
        prio_bits: u8,
        interrupt: impl cortex_m::interrupt::InterruptNumber,
    ) {
        extern "C" {
            static RTIC_ASYNC_MAX_LOGICAL_PRIO: u8;
        }

        let max_prio = RTIC_ASYNC_MAX_LOGICAL_PRIO.max(1).min(1 << prio_bits);
        let hw_prio = cortex_logical2hw(max_prio, prio_bits);

        // We take ownership of the entire IRQ and all settings to it, we only change
        // settings for the IRQ we control.
        // This will also compile-error in case the NVIC changes in size.
        let mut nvic: cortex_m::peripheral::NVIC = core::mem::transmute(());

        nvic.set_priority(interrupt, hw_prio);
    }
}
