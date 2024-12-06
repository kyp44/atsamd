// NOTE: TODO: Move this elsewhere so it is not duplicated?
// As explained in the datasheet, reading a read-synced register may result in
// an old value, which we try to avoid by ensuring that SYNCBUSY is clear before
// reading. A write to a write-synced register will be discarded if syncing is
// happening during the write. As such we also ensure that SYNCBUSY is clear
// before writing to a synced register. Every register access should be prefaced
// by a SYNC comment indicating the required synchronization, which indicates
// that this access was checked and accounted for.

use crate::pac;
use atsamd_hal_macros::hal_macro_helper;
use pac::Rtc;

// TODO: Test code
#[derive(Default)]
pub struct LoopChecker {
    count: usize,
}
impl LoopChecker {
    pub fn too_many(&mut self) -> bool {
        self.count += 1;

        self.count > 0x800000
    }
}

/// Type-level enum for RTC interrupts.
pub trait RtcInterrupt {
    fn enable(rtc: &Rtc);
    fn check_flag(rtc: &Rtc) -> bool;
    fn clear_flag(rtc: &Rtc);
}

macro_rules! create_rtc_interrupt {
    ($mode:ident, $name:ident, $bit:ident) => {
        /// Type-level variant for the $name interrupt in $mode.
        enum $name {}
        impl RtcInterrupt for $name {
            #[inline]
            fn enable(rtc: &Rtc) {
                // SYNC: None
                rtc.$mode().intenset().write(|w| w.$bit().set_bit());
            }

            #[inline]
            fn check_flag(rtc: &Rtc) -> bool {
                // SYNC: None
                rtc.$mode().intflag().read().$bit().bit_is_set()
            }

            #[inline]
            fn clear_flag(rtc: &Rtc) {
                // SYNC: None
                rtc.$mode().intflag().modify(|_, w| w.$bit().set_bit());
            }
        }
    };
}

// TODO: Document
// NOTE: All syncing should happen at this level.
#[hal_macro_helper]
pub trait RtcMode {
    type Count: Copy;
    const HALF_PERIOD: Self::Count;
    const MIN_COMPARE_TICKS: Self::Count;

    // TODO: Test code
    fn check_loop(rtc: &Rtc, checker: &mut LoopChecker, item: &str);
    fn check_for_clock_anomaly(rtc: &Rtc, last_count: Self::Count, count: Self::Count);

    unsafe fn set_mode(rtc: &Rtc);
    unsafe fn set_compare(rtc: &Rtc, number: usize, value: Self::Count);
    fn get_compare(rtc: &Rtc, number: usize) -> Self::Count;
    fn start_and_initialize(rtc: &Rtc);
    fn count(rtc: &Rtc) -> Self::Count;
    fn sync_busy(rtc: &Rtc) -> bool;

    #[inline]
    fn reset(rtc: &Rtc) {
        // Reset RTC back to initial settings, which disables it and enters mode 0.
        // NOTE: This register and field are the same in all modes.
        // SYNC: Write
        Self::sync(rtc);
        #[hal_cfg(any("rtc-d11", "rtc-d21"))]
        rtc.mode0().ctrl().modify(|_, w| w.swrst().set_bit());
        #[hal_cfg("rtc-d5x")]
        rtc.mode0().ctrla().modify(|_, w| w.swrst().set_bit());

        // Wait for the reset to complete
        let mut loop_checker = LoopChecker::default();
        // SYNC: Write (we just read though)
        #[hal_cfg(any("rtc-d11", "rtc-d21"))]
        while rtc.mode0().ctrl().read().swrst().bit_is_set() {
            Self::check_loop(rtc, &mut loop_checker, "reset");
        }
        #[hal_cfg("rtc-d5x")]
        while rtc.mode0().ctrla().read().swrst().bit_is_set() {
            Self::check_loop(rtc, &mut loop_checker, "reset");
        }
    }

    #[inline]
    fn enable_interrupt<I: RtcInterrupt>(rtc: &Rtc) {
        I::enable(rtc);
    }

    #[inline]
    fn check_interrupt_flag<I: RtcInterrupt>(rtc: &Rtc) -> bool {
        I::check_flag(rtc)
    }

    #[inline]
    fn clear_interrupt_flag<I: RtcInterrupt>(rtc: &Rtc) {
        I::clear_flag(rtc);
    }

    #[inline]
    fn sync(rtc: &Rtc) {
        let mut loop_checker = LoopChecker::default();

        while Self::sync_busy(rtc) {
            Self::check_loop(rtc, &mut loop_checker, "sync");
        }
    }

    #[inline]
    fn disable(rtc: &Rtc) {
        // SYNC: Write
        Self::sync(rtc);
        // NOTE: This register and field are the same in all modes.
        #[hal_cfg(any("rtc-d11", "rtc-d21"))]
        rtc.mode0().ctrl().modify(|_, w| w.enable().clear_bit());
        #[hal_cfg("rtc-d5x")]
        rtc.mode0().ctrla().modify(|_, w| w.enable().clear_bit());
    }

    #[inline]
    fn enable(rtc: &Rtc) {
        // SYNC: Write
        Self::sync(rtc);
        // NOTE: This register and field are the same in all modes.
        #[hal_cfg(any("rtc-d11", "rtc-d21"))]
        rtc.mode0().ctrl().modify(|_, w| w.enable().set_bit());
        #[hal_cfg("rtc-d5x")]
        rtc.mode0().ctrla().modify(|_, w| w.enable().set_bit());
    }

    #[inline]
    fn is_enabled(rtc: &Rtc) -> bool {
        // SYNC: Write (we just read though)
        // NOTE: This register and field are the same in all modes.
        #[hal_cfg(any("rtc-d11", "rtc-d21"))]
        return rtc.mode0().ctrl().read().enable().bit_is_set();
        #[hal_cfg("rtc-d5x")]
        return rtc.mode0().ctrla().read().enable().bit_is_set();
    }

    #[inline]
    fn wait_for_count_change(rtc: &Rtc) -> Self::Count {
        let mut last_count = Self::count(rtc);

        // If the clock is disabled then just continue.
        // This can happen if a new task pends the interrupt while the queue is empty so
        // that the timer is disabled, which can otherwise result in waiting
        // forever.
        if !Self::is_enabled(rtc) {
            return last_count;
        }

        let mut loop_checker = LoopChecker::default();
        loop {
            let count = Self::count(rtc);

            Self::check_for_clock_anomaly(rtc, last_count, count);
            Self::check_loop(rtc, &mut loop_checker, "wait_for_count_change");

            last_count = count;
        }
    }
}

#[hal_macro_helper]
pub mod mode0 {
    use super::*;

    create_rtc_interrupt!(mode0, Compare0, cmp0);
    #[hal_cfg("rtc-d5x")]
    create_rtc_interrupt!(mode0, Compare1, cmp1);
    #[hal_cfg("rtc-d5x")]
    create_rtc_interrupt!(mode0, Overflow, ovf);

    pub struct RtcMode0;

    impl RtcMode for RtcMode0 {
        type Count = u32;
        const HALF_PERIOD: Self::Count = 0x8000_0000;
        const MIN_COMPARE_TICKS: Self::Count = 5;

        // TODO: Test code
        fn check_loop(rtc: &Rtc, checker: &mut LoopChecker, item: &str) {
            if checker.too_many() {
                panic!(
                "Took too long in '{item}' with count 0x{:X} and cmp 0x{:X}. Int flags: 0x{:X}, Enabled: {:?}",
                Self::count(rtc),
                Self::get_compare(rtc, 0),
                rtc.mode0().intflag().read().bits(),
                Self::is_enabled(rtc),
            );
            }
        }

        // TODO: Test code
        fn check_for_clock_anomaly(rtc: &Rtc, last_count: Self::Count, count: Self::Count) {
            if last_count > 0 && count != last_count.wrapping_add(4) {
                panic!("Clock anomaly at 0x{count:X} with previous step at 0x{last_count:X} with cmp 0x{:X}", Self::get_compare(rtc, 0));
            }
        }

        #[inline]
        unsafe fn set_mode(rtc: &Rtc) {
            // SYNC: Write
            Self::sync(rtc);
            // NOTE: This register and field are the same in all modes.
            #[hal_cfg(any("rtc-d11", "rtc-d21"))]
            rtc.mode0().ctrl().modify(|_, w| w.mode().count32());
            #[hal_cfg("rtc-d5x")]
            rtc.mode0().ctrla().modify(|_, w| w.mode().count32());
        }

        #[inline]
        unsafe fn set_compare(rtc: &Rtc, number: usize, value: Self::Count) {
            // SYNC: Write
            Self::sync(rtc);
            rtc.mode0().comp(number).write(|w| w.comp().bits(value));
        }

        #[inline]
        fn get_compare(rtc: &Rtc, number: usize) -> Self::Count {
            // SYNC: Write (we just read though)
            rtc.mode0().comp(number).read().bits()
        }

        #[inline]
        fn start_and_initialize(rtc: &Rtc) {
            Self::enable(rtc);

            // TODO: Can we just set the sync request once here on d21 and d11?

            // Enable counter sync on SAMx5x, the counter cannot be read otherwise.
            #[hal_cfg("rtc-d5x")]
            {
                // Enable counter synchronization
                // SYNC: Write
                Self::sync(rtc);
                #[hal_cfg("rtc-d5x")]
                rtc.mode0().ctrla().modify(|_, w| w.countsync().set_bit());

                // Errata: The first read of the count is incorrect so we need to read it
                // then wait for it to change.
                Self::wait_for_count_change(rtc);
            }
        }

        #[inline]
        fn count(rtc: &Rtc) -> Self::Count {
            #[hal_cfg(any("rtc-d11", "rtc-d21"))]
            {
                // Request syncing of the COUNT register.
                // SYNC: None
                rtc.mode0().readreq().modify(|_, w| w.rreq().set_bit());
            }

            // SYNC: Read/Write
            Self::sync(rtc);
            rtc.mode0().count().read().bits()
        }

        #[inline]
        fn sync_busy(rtc: &Rtc) -> bool {
            // SYNC: None
            #[hal_cfg(any("rtc-d11", "rtc-d21"))]
            return rtc.mode0().status().read().syncbusy().bit_is_set();

            // SYNC: None
            #[hal_cfg("rtc-d5x")]
            return rtc.mode0().syncbusy().read().bits() != 0;
        }
    }

    // The SAMD11/21 chips do not feature a second compare in MODE0.
    #[hal_cfg(any("rtc-d11", "rtc-d21"))]
    crate::__internal_basic_backend!(RtcBackend, RtcMode0, Compare0);

    #[hal_cfg("rtc-d5x")]
    crate::__internal_basic_backend!(RtcBackend, RtcMode0, Compare0);
}

#[hal_macro_helper]
pub mod mode1 {
    use super::*;

    create_rtc_interrupt!(mode1, Compare0, cmp0);
    create_rtc_interrupt!(mode1, Compare1, cmp1);
    create_rtc_interrupt!(mode1, Overflow, ovf);

    pub struct RtcMode1;

    impl RtcMode for RtcMode1 {
        type Count = u16;
        const HALF_PERIOD: Self::Count = 0x8000;
        const MIN_COMPARE_TICKS: Self::Count = 5;

        // TODO: Test code
        fn check_loop(rtc: &Rtc, checker: &mut LoopChecker, item: &str) {
            if checker.too_many() {
                panic!(
                "Took too long in '{item}' with count 0x{:X} and cmp 0x{:X}. Int flags: 0x{:X}, Enabled: {:?}",
                Self::count(rtc),
                Self::get_compare(rtc, 0),
                rtc.mode1().intflag().read().bits(),
                Self::is_enabled(rtc),
            );
            }
        }

        // TODO: Test code
        fn check_for_clock_anomaly(rtc: &Rtc, last_count: Self::Count, count: Self::Count) {
            if last_count > 0 && count != last_count.wrapping_add(4) {
                panic!("Clock anomaly at 0x{count:X} with previous step at 0x{last_count:X} with cmp 0x{:X}", Self::get_compare(rtc, 0));
            }
        }

        #[inline]
        unsafe fn set_mode(rtc: &Rtc) {
            // SYNC: Write
            Self::sync(rtc);
            // NOTE: This register and field are the same in all modes.
            #[hal_cfg(any("rtc-d11", "rtc-d21"))]
            rtc.mode0().ctrl().modify(|_, w| w.mode().count16());
            #[hal_cfg("rtc-d5x")]
            rtc.mode0().ctrla().modify(|_, w| w.mode().count16());

            // Set the mode 1 period
            // SYNC: Write
            Self::sync(rtc);
            rtc.mode1().per().write(|w| w.bits(0xFFFF));
        }

        #[inline]
        unsafe fn set_compare(rtc: &Rtc, number: usize, value: Self::Count) {
            // SYNC: Write
            Self::sync(rtc);
            rtc.mode1().comp(number).write(|w| w.comp().bits(value));
        }

        #[inline]
        fn get_compare(rtc: &Rtc, number: usize) -> Self::Count {
            // SYNC: Write (we just read though)
            rtc.mode1().comp(number).read().bits()
        }

        #[inline]
        fn start_and_initialize(rtc: &Rtc) {
            Self::enable(rtc);

            // TODO: Can we just set the sync request once here on d21 and d11?

            // Enable counter sync on SAMx5x, the counter cannot be read otherwise.
            #[hal_cfg("rtc-d5x")]
            {
                // Enable counter synchronization
                // SYNC: Write
                Self::sync(rtc);
                rtc.mode1().ctrla().modify(|_, w| w.countsync().set_bit());

                // Errata: The first read of the count is incorrect so we need to read it
                // then wait for it to change.
                Self::wait_for_count_change(rtc);
            }
        }

        #[inline]
        fn count(rtc: &Rtc) -> Self::Count {
            #[hal_cfg(any("rtc-d11", "rtc-d21"))]
            {
                // Request syncing of the COUNT register.
                // SYNC: None
                rtc.mode1().readreq().modify(|_, w| w.rreq().set_bit());
            }

            // SYNC: Read/Write
            Self::sync(rtc);
            rtc.mode1().count().read().bits()
        }

        #[inline]
        fn sync_busy(rtc: &Rtc) -> bool {
            // SYNC: None
            #[hal_cfg(any("rtc-d11", "rtc-d21"))]
            return rtc.mode1().status().read().syncbusy().bit_is_set();

            // SYNC: None
            #[hal_cfg("rtc-d5x")]
            return rtc.mode1().syncbusy().read().bits() != 0;
        }
    }

    crate::__internal_half_period_counting_backend!(
        RtcBackend, RtcMode1, Compare0, Compare1, Overflow
    );
}
