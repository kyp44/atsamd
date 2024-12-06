// NOTE: TODO: Move this elsewhere so it is not duplicated?
// As explained in the datasheet, reading a read-synced register may result in
// an old value, which we try to avoid by ensuring that SYNCBUSY is clear before
// reading. A write to a write-synced register will be discarded if syncing is
// happening during the write. As such we also ensure that SYNCBUSY is clear
// before writing to a synced register. Every register access should be prefaced
// by a SYNC comment indicating the required synchronization, which indicates
// that this access was checked and accounted for.

// TODO: Put this info somewhere in the documentation:
// - Default clock source is the internal 1k on the SAMD5x.
// - There is no default on SAMD11/21, a generic clock must be configured

#[doc(hidden)]
#[macro_export]
macro_rules! __internal_interrupt_handler {
    ($mode:ty, $rtic_int:ty) => {
        /// RTC interrupt handler called before control passes to the
        /// [`TimerQueue`
        /// handler`](rtic_time::timer_queue::TimerQueue::on_monotonic_interrupt).
        #[inline]
        pub unsafe fn interrupt_handler() {
            let rtc = pac::Rtc::steal();

            /// Returns whether a < b, taking wrapping into account and assuming
            /// that the difference is less than a half period.
            #[inline]
            fn less_than_with_wrap(
                a: <$mode as RtcMode>::Count,
                b: <$mode as RtcMode>::Count,
            ) -> bool {
                let d = a.wrapping_sub(b);

                d >= <$mode>::HALF_PERIOD
            }

            // Ensure that the COUNT is at least the compare value
            // Due to syncing delay this may not be the case initially
            // Note that this has to be done here because RTIC will clear the cmp0 flag
            // before `RtcBackend::on_interrupt` is called.
            if <$mode>::check_interrupt_flag::<$rtic_int>(&rtc) {
                let compare = <$mode>::get_compare(&rtc, 0);
                while less_than_with_wrap(<$mode>::count(&rtc), compare) {}
            }

            Self::timer_queue().on_monotonic_interrupt();
        }
    };
}

#[doc(hidden)]
#[macro_export]
macro_rules! __internal_basic_backend {
    ($name:ident, $mode:ty, $rtic_int:ty) => {
        use crate::pac;
        use crate::rtc::rtic::modes::RtcMode;
        use atsamd_hal_macros::hal_macro_helper;
        use rtic_time::timer_queue::{TimerQueue, TimerQueueBackend};

        /// Basic RTC-based [`TimerQueueBackend`] without period counting.
        pub struct $name;

        static RTC_TQ: TimerQueue<$name> = TimerQueue::new();

        #[hal_macro_helper]
        impl $name {
            crate::__internal_interrupt_handler!($mode, $rtic_int);

            /// Starts the clock.
            ///
            /// **Do not use this function directly.**
            ///
            /// Use the [`rtc_mode0_monotonic`](crate::rtc_mode0_monotonic) macro
            /// instead and then call `start` on the monotonic.
            // TODO: Do we end up setting the clock here? What about simply ensuring that
            // it's enabled?
            /* pub fn _start<S: crate::rtc::rtic::rtc_clock::RtcClockSetter>(
                rtc: pac::Rtc,
                mclk: &mut pac::Mclk,
                osc32kctrl: &mut pac::Osc32kctrl,
            ) { */
            // TODO: Can these be combined for both backend? There is a lot of code
            // duplication.
            pub fn _start(rtc: pac::Rtc) {
                // Disable the RTC.
                <$mode>::disable(&rtc);

                // Set the RTC clock source.
                // TODO: We may remove this, see:
                // https://github.com/atsamd-rs/atsamd/issues/765#issuecomment-2524171063
                // osc32kctrl.rtcctrl().write(S::set_source);

                // Enable the APBA clock for the RTC.
                // TODO: We may remove this, see:
                // https://github.com/atsamd-rs/atsamd/issues/765#issuecomment-2524171063
                // mclk.apbamask().modify(|_, w| w.rtc_().set_bit());

                // Reset RTC back to initial settings, which disables it and enters mode 0.
                <$mode>::reset(&rtc);

                unsafe {
                    // Set the RTC mode
                    <$mode>::set_mode(&rtc);

                    // Set the the initial compare
                    <$mode>::set_compare(&rtc, 0, 0);
                }

                // Timing critical, make sure we don't get interrupted.
                critical_section::with(|_| {
                    // Start the timer and initialize it
                    <$mode>::start_and_initialize(&rtc);

                    // Clear the triggered compare flag
                    <$mode>::clear_interrupt_flag::<$rtic_int>(&rtc);

                    // Enable the compare interrupt
                    <$mode>::enable_interrupt::<$rtic_int>(&rtc);

                    // Initialize the timer queue
                    RTC_TQ.initialize(Self);

                    // Enable the RTC interrupt in the NVIC and set its priority.
                    // SAFETY: We take full ownership of the peripheral and interrupt vector,
                    // plus we are not using any external shared resources so we won't impact
                    // basepri/source masking based critical sections.
                    unsafe {
                        crate::rtc::rtic::set_monotonic_prio(
                            pac::NVIC_PRIO_BITS,
                            pac::Interrupt::RTC,
                        );
                        pac::NVIC::unmask(pac::Interrupt::RTC);
                    }
                });
            }
        }

        impl TimerQueueBackend for $name {
            type Ticks = <$mode as RtcMode>::Count;

            #[hal_macro_helper]
            fn now() -> Self::Ticks {
                <$mode>::count(unsafe { &pac::Rtc::steal() })
            }

            fn enable_timer() {
                <$mode>::enable(unsafe { &pac::Rtc::steal() });
            }

            fn disable_timer() {
                <$mode>::disable(unsafe { &pac::Rtc::steal() });
            }

            fn on_interrupt() {
                // There is nothing we need to do here
            }

            fn set_compare(mut instant: Self::Ticks) {
                let rtc = unsafe { pac::Rtc::steal() };

                // Evidently the compare interrupt will not trigger if the instant is within a
                // couple of ticks, so delay it a bit if it is too close.
                // This is not mentioned in the documentation or errata, but is known to be an
                // issue for other microcontrollers as well (e.g. nRF family).
                if instant.saturating_sub(Self::now()) < <$mode>::MIN_COMPARE_TICKS {
                    instant = instant.wrapping_add(<$mode>::MIN_COMPARE_TICKS)
                }

                unsafe { <$mode>::set_compare(&rtc, 0, instant) };
            }

            fn clear_compare_flag() {
                <$mode>::clear_interrupt_flag::<$rtic_int>(unsafe { &pac::Rtc::steal() });
            }

            fn pend_interrupt() {
                pac::NVIC::pend(pac::Interrupt::RTC);
            }

            fn timer_queue() -> &'static TimerQueue<Self> {
                &RTC_TQ
            }
        }
    };
}

#[doc(hidden)]
#[macro_export]
macro_rules! __internal_half_period_counting_backend {
    ($name:ident, $mode:ty, $rtic_int:ty, $half_period_int:ty, $overflow_int:ty) => {
        use crate::pac;
        use crate::rtc::rtic::modes::RtcMode;
        use atsamd_hal_macros::hal_macro_helper;
        use core::sync::atomic::Ordering;
        use portable_atomic::AtomicU64;
        use rtic_time::{
            half_period_counter::calculate_now,
            timer_queue::{TimerQueue, TimerQueueBackend},
        };

        struct TimerValue(<$mode as RtcMode>::Count);
        impl rtic_time::half_period_counter::TimerValue for TimerValue {
            const BITS: u32 = <$mode as RtcMode>::Count::BITS;
        }
        impl From<TimerValue> for u64 {
            fn from(value: TimerValue) -> Self {
                Self::from(value.0)
            }
        }

        /// An RTC-based [`TimerQueueBackend`] that vastly extends the overall monotonic
        /// period using [half-period counting](rtic_time::half_period_counter).
        pub struct $name;

        static RTC_PERIOD_COUNT: AtomicU64 = AtomicU64::new(0);
        static RTC_TQ: TimerQueue<$name> = TimerQueue::new();

        #[hal_macro_helper]
        impl $name {
            /// Starts the clock.
            ///
            /// **Do not use this function directly.**
            ///
            /// Use the [`rtc_mode1_monotonic`](crate::rtc_mode1_monotonic) macro
            /// instead and then call `start` on the monotonic.
            // TODO: Do we end up setting the clock here? What about simply ensuring that
            // it's enabled?
            /* pub fn _start<S: crate::rtc::rtic::rtc_clock::RtcClockSetter>(
                rtc: pac::Rtc,
                // TODO: Evidently can conditional have an argument!
                #[hal_cfg("rtc-d5x")] mclk: &mut pac::Mclk,
                osc32kctrl: &mut pac::Osc32kctrl,
            ) { */
            // TODO: Can these be combined for both backend? There is a lot of code
            // duplication.
            pub fn _start(rtc: pac::Rtc) {
                // Disable the RTC.
                <$mode>::disable(&rtc);

                // Set the RTC clock source.
                // TODO: We may remove this, see:
                // https://github.com/atsamd-rs/atsamd/issues/765#issuecomment-2524171063
                // osc32kctrl.rtcctrl().write(S::set_source);

                // Enable the APBA clock for the RTC.
                // TODO: We may remove this, see:
                // https://github.com/atsamd-rs/atsamd/issues/765#issuecomment-2524171063
                // mclk.apbamask().modify(|_, w| w.rtc_().set_bit());

                // Reset RTC back to initial settings, which disables it and enters mode 0.
                <$mode>::reset(&rtc);

                unsafe {
                    // Set the RTC mode
                    <$mode>::set_mode(&rtc);

                    // Configure the compare registers
                    <$mode>::set_compare(&rtc, 0, 0);
                    <$mode>::set_compare(&rtc, 1, <$mode>::HALF_PERIOD);
                }

                // Timing critical, make sure we don't get interrupted.
                critical_section::with(|_| {
                    // Start the timer and initialize it
                    <$mode>::start_and_initialize(&rtc);

                    // Clear the triggered compare flag
                    <$mode>::clear_interrupt_flag::<$rtic_int>(&rtc);

                    // Make sure period counter is synced with the timer value
                    RTC_PERIOD_COUNT.store(0, Ordering::SeqCst);

                    // Initialize the timer queue
                    RTC_TQ.initialize(Self);

                    // Enable the compare and overflow interrupts.
                    <$mode>::enable_interrupt::<$rtic_int>(&rtc);
                    <$mode>::enable_interrupt::<$half_period_int>(&rtc);
                    <$mode>::enable_interrupt::<$overflow_int>(&rtc);

                    // Enable the RTC interrupt in the NVIC and set its priority.
                    // SAFETY: We take full ownership of the peripheral and interrupt vector,
                    // plus we are not using any external shared resources so we won't impact
                    // basepri/source masking based critical sections.
                    unsafe {
                        crate::rtc::rtic::set_monotonic_prio(
                            pac::NVIC_PRIO_BITS,
                            pac::Interrupt::RTC,
                        );
                        pac::NVIC::unmask(pac::Interrupt::RTC);
                    }
                });
            }
        }

        impl TimerQueueBackend for RtcBackend {
            type Ticks = u64;

            #[hal_macro_helper]
            fn now() -> Self::Ticks {
                calculate_now(
                    || RTC_PERIOD_COUNT.load(Ordering::Relaxed),
                    || TimerValue(<$mode>::count(unsafe { &pac::Rtc::steal() })),
                )
            }

            fn enable_timer() {
                <$mode>::enable(unsafe { &pac::Rtc::steal() });
            }

            fn disable_timer() {
                <$mode>::disable(unsafe { &pac::Rtc::steal() });
            }

            fn on_interrupt() {
                let rtc: pac::Rtc = unsafe { pac::Rtc::steal() };

                // NOTE: The cmp0 flag is cleared when RTIC calls `clear_compare_flag`.
                if <$mode>::check_interrupt_flag::<$half_period_int>(&rtc) {
                    <$mode>::clear_interrupt_flag::<$half_period_int>(&rtc);
                    let prev = RTC_PERIOD_COUNT.fetch_add(1, Ordering::Relaxed);
                    assert!(prev % 2 == 0, "Monotonic must have skipped an interrupt!");

                    // Ensure that the COUNT has crossed
                    // Due to syncing delay this may not be the case initially
                    while <$mode>::count(&rtc) < <$mode>::HALF_PERIOD {}
                }
                if <$mode>::check_interrupt_flag::<$overflow_int>(&rtc) {
                    <$mode>::clear_interrupt_flag::<$overflow_int>(&rtc);
                    let prev = RTC_PERIOD_COUNT.fetch_add(1, Ordering::Relaxed);
                    assert!(prev % 2 == 1, "Monotonic must have skipped an interrupt!");

                    // Ensure that the COUNT has wrapped
                    // Due to syncing delay this may not be the case initially
                    while <$mode>::HALF_PERIOD > 0x8000 {}
                }
            }

            fn set_compare(mut instant: Self::Ticks) {
                let rtc = unsafe { pac::Rtc::steal() };

                const MAX: u64 = 0xFFFF;

                // Disable interrupts because this section is timing critical.
                // We rely on the fact that this entire section runs within one
                // RTC clock tick. (which it will do easily if it doesn't get
                // interrupted)
                critical_section::with(|_| {
                    let now = Self::now();

                    // Wrapping_sub deals with the u64 overflow corner case
                    let diff = instant.wrapping_sub(now);
                    let val = if diff <= MAX {
                        // Now we know `instant` will happen within one `MAX` time duration.

                        // Evidently the compare interrupt will not trigger if the instant is within
                        // a couple of ticks, so delay it a bit if it is too
                        // close. This is not mentioned in the documentation
                        // or errata, but is known to be an issue for other
                        // microcontrollers as well (e.g. nRF family).
                        if diff < <$mode>::MIN_COMPARE_TICKS.into() {
                            instant = instant.wrapping_add(<$mode>::MIN_COMPARE_TICKS.into())
                        }

                        (instant & MAX) as u16
                    } else {
                        0
                    };

                    unsafe { <$mode>::set_compare(&rtc, 0, val) };
                });
            }

            fn clear_compare_flag() {
                <$mode>::clear_interrupt_flag::<$rtic_int>(unsafe { &pac::Rtc::steal() });
            }

            fn pend_interrupt() {
                pac::NVIC::pend(pac::Interrupt::RTC);
            }

            fn timer_queue() -> &'static TimerQueue<Self> {
                &RTC_TQ
            }
        }
    };
}
