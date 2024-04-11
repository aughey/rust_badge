//! This example test the RP Pico on board LED.
//!
//! It does not work with the RP Pico W board. See wifi_blinky.rs.

#![no_std]
#![no_main]

use cortex_m::prelude::_embedded_hal_blocking_delay_DelayUs;
use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_time::{Duration, Timer};
use embedded_hal::{delay::DelayNs as _, digital::OutputPin as _};
use uc8151::UpdateRegion;
use {defmt_rtt as _, panic_probe as _};

use embedded_graphics::primitives::PrimitiveStyleBuilder;
use embedded_graphics::primitives::StrokeAlignment;
//use hal::halt;
// The macro for our start-up function

// GPIO traits

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
// use pimoroni_badger2040::entry;
// use pimoroni_badger2040::hal::pac;
// use pimoroni_badger2040::hal::Clock;
// use pimoroni_badger2040::hal;
// A shorter alias for the Hardware Abstraction Layer, which provides
// higher-level drivers.
use fugit::RateExtU32;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embedded_graphics::{
    image::Image,
    mono_font::{ascii::*, MonoTextStyle},
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::{PrimitiveStyle, Rectangle},
};
use embedded_text::{
    alignment::HorizontalAlignment,
    style::{HeightMode, TextBoxStyleBuilder},
    TextBox,
};
use pimoroni_badger2040::hal;
use pimoroni_badger2040::hal::pac;
use pimoroni_badger2040::hal::Clock;

use embassy_executor::Executor;
use embassy_rp::multicore::{spawn_core1, Stack};
use static_cell::StaticCell;

use tinybmp::Bmp;

static FERRIS_IMG: &[u8; 2622] = include_bytes!("../ferris_1bpp.bmp");

static mut CORE1_STACK: Stack<4096> = Stack::new();
static EXECUTOR0: StaticCell<Executor> = StaticCell::new();
static EXECUTOR1: StaticCell<Executor> = StaticCell::new();
static CHANNEL: Channel<CriticalSectionRawMutex, &'static str, 1> = Channel::new();

enum LedState {
    On,
    Off,
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();
    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins up according to their function on this particular board
    let pins = pimoroni_badger2040::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    // Set the LED to be an output
    //let mut led_pin = pins.led.into_push_pull_output();

    // Configure the timer peripheral for our blinky delay
    //let mut timer = HalTimer::new(pac.TIMER, &mut pac.RESETS, &clocks);
    let mut display = {
        // Set up the watchdog driver - needed by the clock setup code
        let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

        // The default is to generate a 125 MHz system clock
        let clocks = hal::clocks::init_clocks_and_plls(
            pimoroni_badger2040::XOSC_CRYSTAL_FREQ,
            pac.XOSC,
            pac.CLOCKS,
            pac.PLL_SYS,
            pac.PLL_USB,
            &mut pac.RESETS,
            &mut watchdog,
        )
        .ok()
        .unwrap();

        // We use the embassy time Delay so the hal doesn't take control of the timer
        let mut timer = embassy_time::Delay;

        // Set up the pins for the e-ink display
        let spi_sclk = pins.sclk.into_function::<hal::gpio::FunctionSpi>();
        let spi_mosi = pins.mosi.into_function::<hal::gpio::FunctionSpi>();
        let spi = hal::Spi::<_, _, _>::new(pac.SPI0, (spi_mosi, spi_sclk));
        let mut dc = pins.inky_dc.into_push_pull_output();
        let mut cs = pins.inky_cs_gpio.into_push_pull_output();
        let busy = pins.inky_busy.into_pull_up_input();
        let reset = pins.inky_res.into_push_pull_output();

        // Enable 3.3V power or you won't see anything
        let mut power = pins.p3v3_en.into_push_pull_output();
        power.set_high().unwrap();

        let spi = spi.init(
            &mut pac.RESETS,
            clocks.peripheral_clock.freq(),
            RateExtU32::MHz(1),
            embedded_hal::spi::MODE_0,
        );

        dc.set_high().unwrap();
        cs.set_high().unwrap();

        let mut display = uc8151::Uc8151::new(spi, cs, dc, busy, reset);

        // Reset display
        display.reset(&mut timer);

        let border_stroke = PrimitiveStyleBuilder::new()
            .stroke_color(BinaryColor::Off)
            .stroke_width(3)
            .stroke_alignment(StrokeAlignment::Outside)
            .build();

        // Draw a 3px wide outline around the display.
        // _ = display
        //     .bounding_box()
        //     .into_styled(border_stroke)
        //     .draw(&mut display);

        // Initialise display. Using the default LUT speed setting
        let _ = display.setup(&mut timer, uc8151::LUT::Internal);

        // Note we're setting the Text color to `Off`. The driver is set up to treat Off as Black so that BMPs work as expected.
        let character_style = MonoTextStyle::new(
            &FONT_10X20,
            // FONT_9X18_BOLD,
            BinaryColor::Off,
        );
        let textbox_style = TextBoxStyleBuilder::new()
            .height_mode(HeightMode::FitToText)
            .alignment(HorizontalAlignment::Center)
            .paragraph_spacing(6)
            .build();

        // Bounding box for our text. Fill it with the opposite color so we can read the text.
        let bounds = Rectangle::new(Point::new(157, 10), Size::new(uc8151::WIDTH - 157, 0));
        bounds
            .into_styled(PrimitiveStyle::with_fill(BinaryColor::On))
            .draw(&mut display)
            .unwrap();

        // Create the text box and apply styling options.
        let text = "Embassy\nMy name is\nJohn Aughey";
        let text_box = TextBox::with_textbox_style(text, bounds, character_style, textbox_style);

        // Draw the text box.
        text_box.draw(&mut display).unwrap();
        text_box
            .bounding_box()
            .into_styled(border_stroke)
            .draw(&mut display)
            .unwrap();

        // Draw ferris
        let tga: Bmp<BinaryColor> = Bmp::from_slice(FERRIS_IMG).unwrap();
        let image = Image::new(&tga, Point::zero());
        let _ = image.draw(&mut display);
        let _ = display.update();
        display
    };

    let up_button = Input::new(p.PIN_15, Pull::Up);

    let led = Output::new(p.PIN_25, Level::Low);

    spawn_core1(
        p.CORE1,
        unsafe { &mut *core::ptr::addr_of_mut!(CORE1_STACK) },
        move || {
            let executor1 = EXECUTOR1.init(Executor::new());
            executor1.run(|spawner| {
                unwrap!(spawner.spawn(core1_task(led)));
                unwrap!(spawner.spawn(text_sender()));
            });
        },
    );

    //return;

    info!("Hello from core 0");
    loop {
        // Note we're setting the Text color to `Off`. The driver is set up to treat Off as Black so that BMPs work as expected.
        let character_style = MonoTextStyle::new(
            &FONT_10X20,
            // FONT_9X18_BOLD,
            BinaryColor::Off,
        );
        let textbox_style = TextBoxStyleBuilder::new()
            .height_mode(HeightMode::FitToText)
            .alignment(HorizontalAlignment::Center)
            .paragraph_spacing(6)
            .build();
        let bounds = Rectangle::new(
            Point::new(157, 0),
            Size::new(uc8151::WIDTH - 157, uc8151::HEIGHT),
        );
        _ = bounds
            .into_styled(PrimitiveStyle::with_fill(BinaryColor::On))
            .draw(&mut display);

        // Create the text box and apply styling options.
        let text = CHANNEL.receive().await;
        let text_box = TextBox::with_textbox_style(text, bounds, character_style, textbox_style);

        // Draw the text box.
        _ = text_box.draw(&mut display);
        let _region: UpdateRegion = bounds.try_into().unwrap();
        display.partial_update(bounds.try_into().unwrap()).unwrap();
        //display.update().unwrap();
    }
}

#[embassy_executor::task]
async fn core1_task(mut led: Output<'static, embassy_rp::peripherals::PIN_25>) {
    info!("Hello from core 1");
    loop {
        led.set_high();
        Timer::after_millis(100).await;

        led.set_low();
        Timer::after_millis(400).await;
    }
}

#[embassy_executor::task]
async fn text_sender() {
    loop {
        CHANNEL.send("Hello from\ncore 0").await;
        Timer::after_millis(5000).await;
        CHANNEL.send("Hello again\ncore 0").await;
        Timer::after_millis(5000).await;
    }
}
