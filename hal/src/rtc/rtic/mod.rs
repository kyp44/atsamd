//! [`Monotonic`](rtic_time::Monotonic) implementations using the Real Time
//! Clock (RTC).
//!
//! Enabling the `rtic` feature is required to use this module.
//!
//! For RTIC v1, the old [`rtic_monotonic::Monotonic`] trait is implemented for
//! [`Rtc`](crate::rtc::Rtc) in [`Count32Mode`](crate::rtc::Count32Mode).
//!
//! The items here provide monotonics for RTIC v2. Two monotonics are provided:
//! one that uses the RTC in mode 0 (32-bit hardware counter), and another that
//! uses the RTC in mode 1 (16-bit hardware counter).
//!
//! Which mode is used can influence the total monotonic period before the its
//! time counter overflows and rolls back to zero time. This rollover violates
//! the contract of the monotonic, which is supposed to count time
//! _monotonically_. As such, the rollover can cause strange behavior for tasks
//! scheduled near the time of the rollover. Hence, the monotonic should be
//! chosen to match the use case such that the monotonic will never roll over
//! during the expected total duration of program execution.
//!
//! For all chip variants, the mode 1 monotonic uses [half-period
//! counting](rtic_time::half_period_counter) to extend the monotonic counter to
//! 64-bits. This is enabled by the fact that the RTC for every variant has
//! at least two compare registers in mode 1. This greatly extends the total
//! monotonic period. However, in this mode, half-period counting interrupts
//! occur more frequently (see below) due to the hardware counter being only 16
//! bits wide, so it is less efficient in that regard.
//!
//! For SAMD11/21 chips, the mode 0 monotonic only has a single compare register
//! so that half-period counting is not possible. This significantly reduces the
//! total monotonic period. The SAMx5x chips, however, feature two compare
//! registers in mode 0 so that half-period counting can be done. In the latter
//! case, the mode 0 monotonic has extremely infrequent half-period counting
//! interrupts and so is more efficient.
//!
//! NOTE: These monotonics currently live in the HAL for testing and refinement
//! purposes. The intention is to eventually move them to the
//! [`rtic-monotonics`](https://docs.rs/rtic-monotonics/latest/rtic_monotonics/) crate,
//! which is a central location for peripheral-based RTIC monotonics for various
//! microcontroller families. As such, be aware that this module could disappear
//! at any time in the future.
//!
//! # Choosing a mode
//!
//! The overall monotonic period (i.e. the time before the monotonic rolls over
//! back to zero time) is as follows:
//!
//! |                        | 1 kHz clock        | 32 kHz clock        |
//! | ---------------------- | ------------------ | ------------------- |
//! | **Mode 0 (SAMD11/21)** | ~48 days           | ~36 hours           |
//! | **Mode 0 (SAMx5x)**    | ~571 million years | ~17.8 million years |
//! | **Mode 1**             | ~571 million years | ~17.8 million years |
//!
//! The half-period counting interrupt periods for the monotonics are:
//!
//! |                        | 1 kHz clock | 32 kHz clock |
//! | ---------------------- | ----------- | ------------ |
//! | **Mode 0 (SAMD11/21)** | Never       | Never        |
//! | **Mode 0 (SAMx5x)**    | ~24 days    | ~18 hours    |
//! | **Mode 1**             | 32 seconds  | 1 second     |
//!
//! The time resolution (i.e. the RTC tick time and shortest delay time) is as
//! follows:
//!
//! |              | 1 kHz clock | 32 kHz clock |
//! | ------------ | ----------- | ------------ |
//! | **Any mode** | ~977 μs     | ~31 μs       |
//!
//! # RTC clock selection (TODO)
//!
//! A monotonic using the desired RTC mode should be created with the
//! appropriate [macro](crate::rtc::rtic::prelude). The RTC clock rate and
//! source must also be specified when calling the macro, using the types in
//! [`rtc_clock`]. The first macro argument is the name of the global structure
//! that will implement [`Monotonic`](rtic_time::Monotonic). The second argument
//! must be a clock rate type that implements
//! [`RtcClockRate`](rtc_clock::RtcClockRate), and the third argument must be a
//! type clock source implementing
//! [`RtcClockSource`](rtc_clock::RtcClockSource). See below for an example.
//!
//!
//!
//! TODO: Add note about sync delay and skipping ticks.
//!
//! TODO: Update example.
//! # Example
//!
//! ```
//! use atsamd_hal::prelude::*;
//! rtc_mode0_monotonic!(Mono, rtc_clock::Clock32k, rtc_clock::ClockInternal);
//!
//! fn init() {
//!     # // This is normally provided by the selected PAC
//!     # let rtc = unsafe { core::mem::transmute(()) };
//!     # let mut mclk = unsafe { core::mem::transmute(()) };
//!     # let mut osc32kctrl = unsafe { core::mem::transmute(()) };
//!     // Start the monotonic
//!     Mono::start(rtc, &mut mclk, &mut osc32kctrl);
//! }
//!
//! async fn usage() {
//!     loop {
//!          // Use the monotonic
//!          let timestamp = Mono::now();
//!          Mono::delay(100u32.millis()).await;
//!     }
//! }
//! ```

// TODO: Put this info somewhere in the documentation:
// - Default clock source is the internal 1k on the SAMD5x.
// - There is no default on SAMD11/21, a generic clock must be configured
// - Must ensure that the RTC clock mast is enabled in PM (SAMD11/21) or Mclk
//   (SAMx5x), which it is already on reset.

mod v1 {
    use crate::rtc::{Count32Mode, Rtc};
    use rtic_monotonic::Monotonic;

    /// The RTC clock frequency in Hz.
    pub const CLOCK_FREQ: u32 = 32_768;

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

mod backends;
mod modes;

pub use modes::mode0::RtcBackend as RtcMode0Backend;
pub use modes::mode1::RtcBackend as RtcMode1Backend;

pub mod prelude {
    pub use super::rtc_clock;
    pub use crate::{rtc_mode0_monotonic, rtc_mode1_monotonic};
    pub use rtic_time::Monotonic;

    pub use fugit::{self, ExtU32, ExtU32Ceil, ExtU64, ExtU64Ceil};
}

/// Types used to specify the RTC clock source at compile time when creating the
/// monotonics.
///
/// These types utilize [type-level programming](crate::typelevel)
/// techniques and are passed to the [monotonic creation
/// macros](crate::rtc::rtic::prelude).
/// The clock rate must be specified at compile time so that the `Instant` and
/// `Duration` types in
/// [`TimerQueueBasedMonotonic`](rtic_time::monotonic::TimerQueueBasedMonotonic)
/// can be specified.
pub mod rtc_clock {
    /// Type-level enum for available RTC clock rates.
    pub trait RtcClockRate {
        const RATE_HZ: u32;
    }

    /// Type level [`RtcClockRate`] variant for the 32.768 kHz clock rate.
    pub enum Clock32k {}
    impl RtcClockRate for Clock32k {
        const RATE_HZ: u32 = 32_768;
    }

    /// Type level [`RtcClockRate`] variant for the 1.024 kHz clock rate.
    pub enum Clock1k {}
    impl RtcClockRate for Clock1k {
        const RATE_HZ: u32 = 1_024;
    }

    /// Type level [`RtcClockRate`] variant for a custom clock rate
    pub struct ClockCustom<const RATE_HZ: u32>;
    impl<const RATE_HZ: u32> RtcClockRate for ClockCustom<RATE_HZ> {
        const RATE_HZ: u32 = RATE_HZ;
    }
}

#[doc(hidden)]
#[macro_export]
macro_rules! __internal_create_rtc_interrupt {
    ($backend:ident) => {
        #[no_mangle]
        #[allow(non_snake_case)]
        unsafe extern "C" fn RTC() {
            $crate::rtc::rtic::$backend::interrupt_handler();
        }
    };
}

#[doc(hidden)]
#[macro_export]
macro_rules! __internal_create_rtc_struct {
    ($name:ident, $backend:ident, $clock_rate:ty, $clock_source:ty) => {
        /// A `Monotonic` based on the RTC peripheral.
        pub struct $name;

        impl $name {
            /// This method must be called only once.
            pub fn start(rtc: $crate::pac::Rtc) {
                use $crate::rtc::rtic::rtc_clock::*;
                $crate::__internal_create_rtc_interrupt!($backend);

                $crate::rtc::rtic::$backend::_start(rtc);
            }
        }

        use $crate::rtc::rtic::rtc_clock::RtcClockRate;

        impl $crate::rtic_time::monotonic::TimerQueueBasedMonotonic for $name {
            type Backend = $crate::rtc::rtic::$backend;
            type Instant = $crate::fugit::Instant<
                <Self::Backend as $crate::rtic_time::timer_queue::TimerQueueBackend>::Ticks,
                1,
                { <$clock_rate>::RATE_HZ },
            >;
            type Duration = $crate::fugit::Duration<
                <Self::Backend as $crate::rtic_time::timer_queue::TimerQueueBackend>::Ticks,
                1,
                { <$clock_rate>::RATE_HZ },
            >;
        }

        $crate::rtic_time::impl_embedded_hal_delay_fugit!($name);
        $crate::rtic_time::impl_embedded_hal_async_delay_fugit!($name);
    };
}

/// Create an RTIC v2 monotonic using the RTC in mode 0.
///
/// See the [`rtic`](crate::rtc::rtic) module for details.
#[macro_export]
macro_rules! rtc_mode0_monotonic {
    ($name:ident, $clock_rate: ty, $clock_source: ty) => {
        $crate::__internal_create_rtc_struct!($name, RtcMode0Backend, $clock_rate, $clock_source);
    };
}

/// Create an RTIC v2 monotonic based on RTC in mode 1.
///
/// See the [`rtic`](crate::rtc::rtic) module for details.
#[macro_export]
macro_rules! rtc_mode1_monotonic {
    ($name:ident, $clock_rate: ty, $clock_source: ty) => {
        $crate::__internal_create_rtc_struct!($name, RtcMode1Backend, $clock_rate, $clock_source);
    };
}

/// This function was copied from the private function in `rtic-monotonics`,
/// so that should be used when the monotonics move there.
const fn cortex_logical2hw(logical: u8, nvic_prio_bits: u8) -> u8 {
    ((1 << nvic_prio_bits) - logical) << (8 - nvic_prio_bits)
}

/// This function was copied from the private function in `rtic-monotonics`,
/// so that should be used when the monotonics move there.
unsafe fn set_monotonic_prio(prio_bits: u8, interrupt: impl cortex_m::interrupt::InterruptNumber) {
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
