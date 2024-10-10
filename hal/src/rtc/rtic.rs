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
    use core::marker::PhantomData;
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
                pub fn start(rtc: $pac_rtc, mclk: &mut $crate::pac::Mclk) {
                    $crate::__internal_create_rtc_interrupt!($mono_backend, $interrupt_rtc);

                    $crate::rtc::rtic::$mono_backend::_start(rtc, mclk);
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

    macro_rules! make_rtc {
        ($backend_name:ident, $pac_rtc:ty, $interrupt_rtc:ident, $tq:ident$(, doc: ($($doc:tt)*))?) => {
            /// RTC based [`TimerQueueBackend`].
            $(
                #[cfg_attr(docsrs, doc(cfg($($doc)*)))]
            )?
            pub struct $backend_name;

            static $tq: TimerQueue<$backend_name> = TimerQueue::new();

            impl $backend_name {
                #[inline]
                fn steal_rtc() -> Rtc<Count32Mode> {
                    Rtc {
                        rtc: unsafe { <$pac_rtc>::steal() },
                        rtc_clock_freq: HertzU32::from_raw(CLOCK_FREQ),
                        _mode: PhantomData::<Count32Mode>::default(),
                    }
                }

                /// Starts the clock.
                ///
                /// **Do not use this function directly.**
                ///
                /// Use the [`rtc_monotonic`] macro instead.
                pub fn _start(rtc: $pac_rtc, mclk: &mut pac::Mclk) {
                    // Initialize the RTC as a counter
                    let mut rtc = Rtc::count32_mode(rtc, HertzU32::from_raw(CLOCK_FREQ), mclk);

                    // Initialize the timer queue
                    $tq.initialize(Self {});

                    // Enable the compare interrupt
                    rtc.enable_interrupt();

                    // Reset the count
                    rtc.reset();

                    // TODO: Need to enable the RTC interrupt and set its priority
                    // SAFETY: We take full ownership of the peripheral and interrupt vector,
                    // plus we are not using any external shared resources so we won't impact
                    // basepri/source masking based critical sections.
                    unsafe {
                        set_monotonic_prio(pac::NVIC_PRIO_BITS, pac::Interrupt::$interrupt_rtc);
                        pac::NVIC::unmask(pac::Interrupt::$interrupt_rtc);
                    }
                }
            }

            impl TimerQueueBackend for $backend_name {
                type Ticks = u32;

                fn now() -> Self::Ticks {
                Self::steal_rtc().count32()
                }

                fn set_compare(instant: Self::Ticks) {
                    unsafe {
                        <$pac_rtc>::steal().mode0()
                            .comp(0)
                            .write(|w| w.comp().bits(instant))
                    }
                }

                fn clear_compare_flag() {
                    unsafe { <$pac_rtc>::steal().mode0().intflag().write(|w| w.cmp0().set_bit()); }
                }

                fn pend_interrupt() {
                    pac::NVIC::pend(pac::Interrupt::$interrupt_rtc);
                }

                fn timer_queue() -> &'static TimerQueue<Self> {
                    &$tq
                }
            }
        };
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

    make_rtc!(RtcBackend, pac::Rtc, RTC, RTC_TQ);
}
