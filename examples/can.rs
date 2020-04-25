#![deny(unsafe_code)]
#![no_std]
#![no_main]

use core::fmt::Write;
use cortex_m_rt::entry;
use cortex_m_semihosting::hio;
use panic_halt as _;
use stm32f1xx_hal::{
    can::{self, ReceiveFifo, TransmitMailbox},
    prelude::*,
};

#[entry]
fn main() -> ! {
    let config = can::Configuration {
        time_triggered_communication_mode: false,
        automatic_bus_off_management: true,
        automatic_wake_up_mode: true,
        no_automatic_retransmission: false,
        receive_fifo_locked_mode: false,
        transmit_fifo_priority: false,
        silent_mode: false,
        loopback_mode: true,
        synchronisation_jump_width: 1,
        bit_segment_1: 3,
        bit_segment_2: 2,
        time_quantum_length: 6,
    };

    let dp = stm32f1xx_hal::stm32::Peripherals::take().unwrap();
    let mut rcc = dp.RCC.constrain();

    let mut gpioa = dp.GPIOA.split(&mut rcc.apb2);
    let canrx = gpioa.pa11.into_floating_input(&mut gpioa.crh);
    let cantx = gpioa.pa12.into_alternate_push_pull(&mut gpioa.crh);

    //remapped version:
    //let mut gpiob = dp.GPIOB.split(&mut rcc.apb2);
    //let canrx = gpiob.pb8.into_floating_input(&mut gpiob.crh);
    //let cantx = gpiob.pb9.into_alternate_push_pull(&mut gpiob.crh);

    let mut afio = dp.AFIO.constrain(&mut rcc.apb2);
    //USB is needed here because it can not be used at the same time as CAN since they share memory:
    let mut can = can::Can::can1(
        dp.CAN1,
        (cantx, canrx),
        &mut afio.mapr,
        &mut rcc.apb1,
        dp.USB,
    );

    can.configure(&config);
    nb::block!(can.to_normal()).unwrap(); //just to be sure

    let id10: can::Id = can::Id::new_standard(10);
    let id11: can::Id = can::Id::new_standard(11);
    let id12: can::Id = can::Id::new_standard(12);
    let id13: can::Id = can::Id::new_standard(13);
    let id14: can::Id = can::Id::new_standard(14);
    let id15: can::Id = can::Id::new_standard(15);

    let filterbank0_config = can::FilterBankConfiguration {
        mode: can::FilterMode::List,
        info: can::FilterInfo::Whole(can::FilterData {
            id: id10.clone(),
            mask_or_id2: id11.clone(),
        }),
        fifo_assignment: 0,
        active: true,
    };
    let filterbank1_config = can::FilterBankConfiguration {
        mode: can::FilterMode::List,
        info: can::FilterInfo::Whole(can::FilterData {
            id: id12.clone(),
            mask_or_id2: id13.clone(),
        }),
        fifo_assignment: 1,
        active: true,
    };
    let filterbank2_config = can::FilterBankConfiguration {
        mode: can::FilterMode::List,
        info: can::FilterInfo::Whole(can::FilterData {
            id: id14.with_rtr(),
            mask_or_id2: id14.clone(),
        }),
        fifo_assignment: 0,
        active: true,
    };
    let filterbank3_config = can::FilterBankConfiguration {
        mode: can::FilterMode::List,
        info: can::FilterInfo::Whole(can::FilterData {
            id: id15.with_rtr(),
            mask_or_id2: id15.clone(),
        }),
        fifo_assignment: 1,
        active: true,
    };
    can.configure_filter_bank(0, &filterbank0_config);
    can.configure_filter_bank(1, &filterbank1_config);
    can.configure_filter_bank(2, &filterbank2_config);
    can.configure_filter_bank(3, &filterbank3_config);

    let mut hstdout = hio::hstdout().unwrap();

    let (tx, rx) = can.split();

    let (mut tx0, mut tx1, mut tx2) = tx.split();

    let txresult0 = tx0.request_transmit(&can::Frame::new(id10, b"0"));
    let txresult1 = tx1.request_transmit(&can::Frame::new(id11, b"1"));
    let txresult2 = tx2.request_transmit(&can::Frame::new(id12, b"2"));
    writeln!(
        hstdout,
        "tx: {:?} {:?} {:?}",
        &txresult0, &txresult1, &txresult2
    )
    .unwrap(); //while this printing slowly, all 3 messages are transfered
    let txresult0 = tx0.request_transmit(&can::Frame::new(id13, b"3"));
    let txresult1 = tx1.request_transmit(&can::Frame::new(id14, b"4"));
    let txresult2 = tx2.request_transmit(&can::Frame::new(id15, b"5"));
    writeln!(
        hstdout,
        "tx: {:?} {:?} {:?}",
        &txresult0, &txresult1, &txresult2
    )
    .unwrap(); //while this printing slowly, all 3 messages are transfered

    let (mut rx0, mut rx1) = rx.split();
    loop {
        if let Ok((filter_match_index, time, frame)) = rx0.read() {
            writeln!(
                hstdout,
                "rx0: {} {} {} {} {:?}",
                filter_match_index,
                frame.id().standard(),
                time,
                frame.data().len(),
                frame.data()
            )
            .unwrap();
        };

        if let Ok((filter_match_index, time, frame)) = rx1.read() {
            writeln!(
                hstdout,
                "rx1: {} {} {} {} {:?}",
                filter_match_index,
                frame.id().standard(),
                time,
                frame.data().len(),
                frame.data()
            )
            .unwrap();
        };
    }

    //writeln!(hstdout, "done.").unwrap();
}
