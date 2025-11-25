#![no_std]
#![no_main]

use bsp::hal;
use circuit_playground_express::{self as bsp};
use hal::gpio::DynPin;
use hal::rtc::rtic::rtc_clock;
use ws2812_timer_delay::Ws2812;

#[cfg(not(feature = "use_semihosting"))]
use panic_halt as _;
#[cfg(feature = "use_semihosting")]
use panic_semihosting as _;

//hal::rtc_monotonic!(Mono, rtc_clock::ClockCustom<8_192>);
hal::rtc_monotonic!(Mono, rtc_clock::Clock32k);

const NUM_PIXELS: usize = 20;
const PIXEL_BRIGHTNESS: u8 = 124;
const DEFAULT_NUM_ACTIVE_PIXELS: usize = 10;
const MIN_SPIN_LEN: usize = 5;
const MAX_SPIN_LEN: usize = 20;

type NeoPixelTimer = hal::timer::TimerCounter3;

#[rtic::app(device = bsp::pac, dispatchers = [EVSYS])]
mod app {
    use super::*;
    use core::mem::MaybeUninit;
    use hal::timer::TimerCounter;
    use rand::Rng;
    use rand::SeedableRng;
    use smart_leds::SmartLedsWrite;
    use usb_device::bus::UsbBusAllocator;
    use usb_device::prelude::*;
    use usbd_serial::{SerialPort, USB_CLASS_CDC};

    use bsp::pin_alias;
    use hal::clock::{ClockGenId, ClockSource, GenericClockController};
    use hal::pac::Peripherals;
    use hal::prelude::*;
    use hal::timer_traits::InterruptDrivenTimer;
    use hal::usb::UsbBus;

    #[local]
    struct Local {
        // usb_allocator: UsbBusAllocator<UsbBus>,
        big_button_pin: DynPin,
        button_a_pin: DynPin,
        red_led: bsp::RedLed,
        switch_pin: DynPin,
    }

    #[shared]
    struct Shared {
        button_a: debouncr::DebouncerStateful<u8, debouncr::Repeat2>,
        neo_pixel: Ws2812<NeoPixelTimer, DynPin>,
        rng: rand::rngs::SmallRng,
        usb_bus: UsbDevice<'static, UsbBus>,
        usb_serial: SerialPort<'static, UsbBus>,
    }

    #[init(local=[usb_allocator: MaybeUninit<UsbBusAllocator<UsbBus>> = MaybeUninit::uninit()])]
    fn init(cx: init::Context) -> (Shared, Local) {
        let mut peripherals: Peripherals = cx.device;
        let mut core: rtic::export::Peripherals = cx.core;
        let mut clocks = GenericClockController::with_internal_32kosc(
            peripherals.gclk,
            &mut peripherals.pm,
            &mut peripherals.sysctrl,
            &mut peripherals.nvmctrl,
        );
        let pins = bsp::Pins::new(peripherals.port);

        let red_led: bsp::RedLed = pin_alias!(pins.red_led).into();
        let button_a_pin = pin_alias!(pins.d4).into_pull_down_input().into();
        let button_a_debouncr = debouncr::debounce_stateful_2(false);

        let big_button_pin = pin_alias!(pins.a1).into_pull_down_input().into();

        let switch_pin = pin_alias!(pins.d7).into_pull_up_input().into();

        *cx.local.usb_allocator = MaybeUninit::new(bsp::usb_allocator(
            peripherals.usb,
            &mut clocks,
            &mut peripherals.pm,
            pins.usb_dm,
            pins.usb_dp,
        ));
        let usb_allocator = unsafe { cx.local.usb_allocator.assume_init_ref() };
        let usb_serial = SerialPort::new(usb_allocator);
        let usb_bus = UsbDeviceBuilder::new(usb_allocator, UsbVidPid(0x6666, 0x6666))
            .strings(&[StringDescriptors::new(LangID::EN)
                .manufacturer("YMATYT Holdings, LLC")
                .product("Debug serial port")
                .serial_number("YMATYT")])
            .expect("Failed to set strings")
            .device_class(USB_CLASS_CDC)
            .build();

        // Set the RTC clock to use clock derived from the internal 32 kHz oscillator.
        // Needs to match the Mono global monotonic clock rate.
        let rtc_clock_src = clocks
            .configure_gclk_divider_and_source(ClockGenId::Gclk2, 1, ClockSource::Osc32k, true)
            .unwrap();
        clocks.configure_standby(ClockGenId::Gclk2, true);
        let _ = clocks.rtc(&rtc_clock_src).unwrap();

        Mono::start(peripherals.rtc);

        let gclk0 = clocks.gclk0();
        let timer_clock = clocks.tcc2_tc3(&gclk0).unwrap();
        let mut timer = TimerCounter::tc3_(&timer_clock, peripherals.tc3, &mut peripherals.pm);
        InterruptDrivenTimer::start(&mut timer, hal::fugit::ExtU32::micros(1));

        // Can't reuse the timer afterward???
        // let builtin_neo_pixel_pin: bsp::NeoPixel = pin_alias!(pins.neo_pixel).into();
        // let mut builtin_neo_pixel = ws2812_timer_delay::Ws2812::new(timer, builtin_neo_pixel_pin);
        // let pixel_colors = [smart_leds::colors::BLACK; NUM_PIXELS];
        // builtin_neo_pixel
        //     .write(pixel_colors.iter().cloned())
        //     .unwrap();

        let neo_pixel_pin: DynPin = pin_alias!(pins.a3).into();
        let mut neo_pixel = Ws2812::new(timer, neo_pixel_pin);

        let pixel_colors = [smart_leds::colors::BLACK; NUM_PIXELS];
        neo_pixel.write(pixel_colors.iter().cloned()).unwrap();

        // Should seed from the microphone/accelerometer
        let rng = rand::rngs::SmallRng::seed_from_u64(666);

        core.SCB.set_sleepdeep();

        blink::spawn().unwrap();
        clicker_picker::spawn().unwrap();

        (
            Shared {
                button_a: button_a_debouncr,
                rng,
                neo_pixel,
                usb_bus,
                usb_serial,
            },
            Local {
                big_button_pin,
                button_a_pin,
                red_led,
                switch_pin,
            },
        )
    }

    #[task(local = [big_button_pin, switch_pin], shared = [button_a, rng, usb_serial])]
    async fn clicker_picker(
        clicker_picker::Context {
            local:
                clicker_picker::LocalResources {
                    big_button_pin,
                    switch_pin,
                    ..
                },
            shared:
                clicker_picker::SharedResources {
                    mut button_a,
                    mut rng,
                    mut usb_serial,
                    ..
                },
            ..
        }: clicker_picker::Context,
    ) -> ! {
        // let mut new_buf = [0u8; 64];
        // use hal::embedded_io::Write;
        // let _ = core::write!(new_buf.as_mut_slice(), "{}", x.random::<i32>());
        // let _ = serial.write(&new_buf);
        let mut old_active_pixels = 0;
        let mut active_pixels = DEFAULT_NUM_ACTIVE_PIXELS;

        let _ = light_active_pixels_neo_pixel::spawn(active_pixels);

        let mut state = ClickerAppState::WaitingForInput;
        loop {
            let now = Mono::now();
            match state {
                ClickerAppState::WaitingForInput => {
                    if big_button_pin.is_high().unwrap() {
                        let _ = usb_serial.lock(|s| s.write(b"big high\r\n"));
                    }
                    if switch_pin.is_low().unwrap() {
                        //|| big_button_pin.is_high().unwrap() {
                        state = ClickerAppState::Configure;
                        continue;
                    }
                    let _ = update_button_state::spawn();
                    if button_a.lock(|but| but.is_high()) || big_button_pin.is_high().unwrap() {
                        state = ClickerAppState::Spinning;
                    }
                }
                ClickerAppState::Spinning => {
                    rng.lock(|rng| {
                        let _ = spin_neo_pixel::spawn(rng.random_range(0..active_pixels));
                    });
                    state = ClickerAppState::WaitingForInput;
                }
                ClickerAppState::Configure => {
                    if switch_pin.is_high().unwrap() {
                        state = ClickerAppState::WaitingForInput;
                        old_active_pixels = 0;
                        continue;
                    }
                    let _ = update_button_state::spawn();
                    if button_a.lock(|but| but.is_high()) {
                        active_pixels = active_pixels % NUM_PIXELS + 1;
                    }
                    if old_active_pixels != active_pixels {
                        let _ = light_active_pixels_neo_pixel::spawn(active_pixels);
                        old_active_pixels = active_pixels;
                    }
                    Mono::delay(100u64.millis()).await;
                }
            }
            Mono::delay_until(now + 50u64.millis()).await;
        }
    }

    #[task(shared = [neo_pixel], priority = 1)]
    // /// A task to report button state
    async fn light_active_pixels_neo_pixel(
        light_active_pixels_neo_pixel::Context {
            shared: light_active_pixels_neo_pixel::SharedResources { mut neo_pixel, .. },
            ..
        }: light_active_pixels_neo_pixel::Context,
        active_pixels: usize,
    ) {
        let pixel_colors: [smart_leds::RGB8; NUM_PIXELS] = core::array::from_fn(|i| {
            if i < active_pixels {
                smart_leds::colors::RED
            } else {
                smart_leds::colors::BLACK
            }
        });
        neo_pixel.lock(|neo_pixel| {
            neo_pixel
                .write(smart_leds::brightness(
                    pixel_colors.iter().rev().cloned(),
                    PIXEL_BRIGHTNESS,
                ))
                .unwrap();
        });
    }

    #[task(shared = [neo_pixel, rng], priority = 1)]
    async fn spin_neo_pixel(
        spin_neo_pixel::Context {
            shared:
                spin_neo_pixel::SharedResources {
                    mut neo_pixel,
                    mut rng,
                    ..
                },
            ..
        }: spin_neo_pixel::Context,
        rand_val: usize,
    ) -> ! {
        let mut pixel_colors = [smart_leds::colors::BLACK; NUM_PIXELS];
        neo_pixel.lock(|neo_pixel| {
            neo_pixel.write(pixel_colors.iter().rev().cloned()).unwrap();
        });

        let (spin_length, stops) = rng.lock(|rng| {
            let spin_length = rng.random_range(MIN_SPIN_LEN..=MAX_SPIN_LEN);
            let stops: [_; MAX_SPIN_LEN] =
                core::array::from_fn(|_| rng.random_range(0..NUM_PIXELS));
            (spin_length, stops)
        });
        for i in 0..spin_length {
            pixel_colors[stops[i]] = smart_leds::colors::PURPLE;
            neo_pixel.lock(|neo_pixel| {
                neo_pixel
                    .write(smart_leds::brightness(
                        pixel_colors.iter().rev().cloned(),
                        PIXEL_BRIGHTNESS,
                    ))
                    .unwrap();
            });
            Mono::delay(100u64.millis()).await;
            pixel_colors[stops[i]] = smart_leds::colors::BLACK;
        }
        // for j in (0..255u8).step_by(20) {
        pixel_colors[rand_val] = smart_leds::colors::RED;
        // pixel_colors[rand_val] = smart_leds::hsv::hsv2rgb(smart_leds::hsv::Hsv {
        //     hue: j,
        //     sat: 255,
        //     val: 2,
        // });
        neo_pixel.lock(|neo_pixel| {
            neo_pixel
                .write(smart_leds::brightness(
                    pixel_colors.iter().rev().cloned(),
                    PIXEL_BRIGHTNESS,
                ))
                .unwrap();
        });
        // Mono::delay(5u64.millis()).await;
        // }
        // let colors = [smart_leds::hsv::hsv2rgb(smart_leds::hsv::Hsv {
        //     hue: 0,
        //     sat: 0,
        //     val: 0,
        // }); NUM_PIXELS];
        // neo_pixel.write(colors.iter().cloned()).unwrap();
    }

    /// A task to report button state
    #[task(local = [button_a_pin], shared = [button_a, usb_serial], priority = 1)]
    async fn update_button_state(
        update_button_state::Context {
            local: update_button_state::LocalResources { button_a_pin, .. },
            shared:
                update_button_state::SharedResources {
                    mut button_a,
                    mut usb_serial,
                    ..
                },
            ..
        }: update_button_state::Context,
    ) {
        usb_serial.lock(|s| {
            button_a.lock(|but| {
                // if button_a_pin.is_high().unwrap() {
                //     let _ = s.write(b"button high\r\n");
                // } else {
                //     let _ = s.write(b"button low\r\n");
                // }
                match but.update(button_a_pin.is_high().unwrap()) {
                    Some(e) => {
                        let _ = s.write(match e {
                            debouncr::Edge::Rising => b"rising",
                            debouncr::Edge::Falling => b"falling",
                        });
                        let _ = s.write(b" edge\r\n");
                    }
                    None => {
                        // let _ = s.write(b"no edge\r\n");
                    }
                }
                // if but.is_high() {
                //     let _ = s.write(b"button debouncer high\r\n");
                // } else {
                //     let _ = s.write(b"button debouncer low\r\n");
                // }
            });
        });
    }

    /// A task to blink the LED to show that there is life on the board.
    #[task(local = [red_led])]
    //async fn blink(cx: blink::Context) {
    async fn blink(
        blink::Context {
            local: blink::LocalResources { red_led, .. },
            ..
        }: blink::Context,
    ) -> ! {
        loop {
            // If the LED were a local resource, the lock would not be necessary
            red_led.toggle().unwrap();
            Mono::delay(1u64.secs()).await;
        }
    }

    #[task(binds = USB, shared = [usb_bus, usb_serial], priority = 1)]
    fn poll_usb(
        poll_usb::Context {
            shared:
                poll_usb::SharedResources {
                    usb_bus,
                    usb_serial,
                    ..
                },
            ..
        }: poll_usb::Context,
    ) {
        (usb_serial, usb_bus).lock(|serial, b| {
            if !b.poll(&mut [serial]) {
                return;
            }

            #[cfg(debug_assertions)]
            {
                let lc = serial.line_coding();
                if lc.data_rate() == 1200 && !serial.dtr() {
                    cortex_m::peripheral::SCB::sys_reset();
                }
            }

            let mut buf = [0u8; 64];
            if let Ok(count) = serial.read(&mut buf) {
                for (i, c) in buf.iter().enumerate() {
                    if i >= count {
                        break;
                    }
                    // s.write(b"hi").ok();
                    serial.write(&[*c]).ok();
                    // s.write(b"bye").ok();
                    // show_button_state::spawn().unwrap();
                    // if c == &b'\r' {
                    //     //     let _ = s.write(b"OMG");
                    //     //     // let _ = s.write(b"afterOMG");
                    //     let _ = spin_neo_pixel::spawn(1);
                    // }
                }
            };
        });
    }
}

enum ClickerAppState {
    Configure,
    WaitingForInput,
    Spinning,
}
