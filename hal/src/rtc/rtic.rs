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

// TODO: can these be moved into v1 if not needed for v2?
type Instant = fugit::Instant<u32, 1, CLOCK_FREQ>;
type Duration = fugit::Duration<u32, 1, CLOCK_FREQ>;

mod v1 {
    use super::*;
    use rtic_monotonic::Monotonic;

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
    use super::*;
    use crate::{pac, timer_traits::InterruptDrivenTimer};
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
                    32_768,
                >;
                type Duration = $crate::fugit::Duration<
                    <Self::Backend as $crate::rtic_time::timer_queue::TimerQueueBackend>::Ticks,
                    1,
                    32_768,
                >;
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

        #[inline]
        #[hal_macro_helper]
        fn count(rtc: &pac::Rtc) -> u32 {
            // Synchronize this read on SAMD11/21. SAMx5x is automatically synchronized
            #[hal_cfg(any("rtc-d11", "rtc-d21"))]
            {
                rtc.mode0().readreq().modify(|_, w| w.rcont().set_bit());
                Self::sync(rtc);
            }
            rtc.mode0().count().read().bits()
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

            // Reset RTC back to initial settings, which disables it and enters mode 0 by
            // default
            rtc.mode0().ctrla().modify(|_, w| w.swrst().set_bit());
            // Wait for the reset to complete
            while rtc.mode0().ctrla().read().swrst().bit_is_set() {}

            // Set the RTC clock source
            // TODO: How to we set this generically so that the user can choose?
            osc32kctrl.rtcctrl().write(|w| w.rtcsel().ulp1k());

            // Enable counter sync on SAMx5x, the counter cannot be read otherwise.
            #[hal_cfg("rtc-d5x")]
            {
                rtc.mode0().ctrla().modify(|_, w| w.countsync().set_bit());
            }

            // Start the counter.
            rtc.mode0().ctrla().modify(|_, w| w.enable().set_bit());
            Self::sync(&rtc);

            while Self::count(&rtc) < 1024 {}
            panic!();

            // Initialize the RTC as a counter
            // TODO: This resets the RTC then enables the clock and waits for sync to
            // complete
            let mut rtc = Rtc::count32_mode(rtc, HertzU32::from_raw(CLOCK_FREQ), mclk);

            // TODO test code to just set a timer and see if it triggers
            //while rtc.count32() < 100 {}
            while rtc.count32() < 1 {}
            //while InterruptDrivenTimer::wait(&mut rtc).is_err() {}
            panic!();

            // Enable the compare interrupt
            rtc.enable_interrupt();

            // TODO: Need to enable the RTC interrupt and set its priority
            // SAFETY: We take full ownership of the peripheral and interrupt vector,
            // plus we are not using any external shared resources so we won't impact
            // basepri/source masking based critical sections.
            unsafe {
                set_monotonic_prio(pac::NVIC_PRIO_BITS, pac::Interrupt::RTC);
                pac::NVIC::unmask(pac::Interrupt::RTC);
            }

            // Initialize the timer queue
            // TODO: When should we actually do this?
            RTC_TQ.initialize(Self {});

            unsafe {
                let rtc = pac::Rtc::steal();

                rtc.mode0().comp(0).write(|w| w.comp().bits(100));
                loop {
                    if rtc.mode0().intflag().read().cmp0().bit_is_set() {
                        break;
                    }
                }

                panic!();
            }
        }
    }

    impl TimerQueueBackend for RtcBackend {
        type Ticks = u32;

        fn now() -> Self::Ticks {
            todo!()
        }

        fn set_compare(instant: Self::Ticks) {
            unsafe {
                pac::Rtc::steal().mode0()
                            .comp(0)
                            //.write(|w| w.comp().bits(instant))
                            .write(|w| w.comp().bits(32768))
            }
        }

        fn clear_compare_flag() {
            unsafe {
                pac::Rtc::steal()
                    .mode0()
                    .intflag()
                    .write(|w| w.cmp0().set_bit());
            }
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
