#![no_std]
#![no_main]

use core::cell::OnceCell;
use core::cell::UnsafeCell;

use cortex_m::interrupt::CriticalSection;
use panic_halt as _;

use cortex_m::asm::delay as cycle_delay;
use cortex_m::interrupt::{free, Mutex};
use cortex_m::peripheral::NVIC;
use usb_device::bus::UsbBusAllocator;
use usb_device::prelude::*;
use usbd_serial::{SerialPort, USB_CLASS_CDC};

use bsp::hal;
use bsp::pac;
use metro_m0 as bsp;

use bsp::entry;
use hal::clock::GenericClockController;
use hal::prelude::*;
use hal::usb::UsbBus;
use pac::{interrupt, CorePeripherals, Peripherals};

#[entry]
fn main() -> ! {
    let mut peripherals = Peripherals::take().unwrap();
    let mut core = CorePeripherals::take().unwrap();
    let mut clocks = GenericClockController::with_internal_32kosc(
        peripherals.gclk,
        &mut peripherals.pm,
        &mut peripherals.sysctrl,
        &mut peripherals.nvmctrl,
    );
    let pins = bsp::Pins::new(peripherals.port);
    let mut red_led: bsp::RedLed = pins.d13.into();

    free(|cs| {
        // These are safe because the data is static and this is a critical section.
        let allocator_cell = unsafe { &mut *USB_ALLOCATOR.borrow(cs).get() };
        let package_cell = unsafe { &mut *USB_PACKAGE.borrow(cs).get() };

        let _ = allocator_cell.set(bsp::usb_allocator(
            peripherals.usb,
            &mut clocks,
            &mut peripherals.pm,
            pins.usb_dm,
            pins.usb_dp,
        ));
        let bus_allocator = allocator_cell.get().unwrap();

        // It seems that this has to be created before the `UsbDevice` for some reason.
        let serial = SerialPort::new(bus_allocator);
        let _ = package_cell.set(UsbPackage {
            device: UsbDeviceBuilder::new(bus_allocator, UsbVidPid(0x16c0, 0x27dd))
                .strings(&[StringDescriptors::new(LangID::EN)
                    .manufacturer("Fake company")
                    .product("Serial port")
                    .serial_number("TEST")])
                .expect("Failed to set strings")
                .device_class(USB_CLASS_CDC)
                .build(),
            serial,
        });
    });

    unsafe {
        core.NVIC.set_priority(interrupt::USB, 1);
        NVIC::unmask(interrupt::USB);
    }

    // Flash the LED in a spin loop to demonstrate that USB is
    // entirely interrupt driven.
    loop {
        cycle_delay(15 * 1024 * 1024);
        red_led.toggle().ok();
    }
}

struct UsbPackage {
    pub device: UsbDevice<'static, UsbBus>,
    pub serial: SerialPort<'static, UsbBus>,
}

static USB_ALLOCATOR: Mutex<UnsafeCell<OnceCell<UsbBusAllocator<UsbBus>>>> =
    Mutex::new(UnsafeCell::new(OnceCell::new()));
static USB_PACKAGE: Mutex<UnsafeCell<OnceCell<UsbPackage>>> =
    Mutex::new(UnsafeCell::new(OnceCell::new()));

fn poll_usb() {
    // This is safe because the data is static, and only this ISR
    // accesses the package after initialization.
    let usb_package = unsafe {
        (*USB_PACKAGE.borrow(&CriticalSection::new()).get())
            .get_mut()
            .unwrap()
    };

    usb_package.device.poll(&mut [&mut usb_package.serial]);
    let mut buf = [0u8; 64];

    if let Ok(count) = usb_package.serial.read(&mut buf) {
        for (i, c) in buf.iter().enumerate() {
            if i >= count {
                break;
            }
            usb_package.serial.write(&[*c]).ok();
        }
    };
}

#[interrupt]
fn USB() {
    poll_usb();
}
