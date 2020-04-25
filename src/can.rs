//! CAN interface
//!
//! The `Can` interface can be used with the following CAN instances:
//!
//! # CAN1
//!
//! - TX = PA12 | PB9
//! - RX = PA11 | PB8
//! - Interrupt = CAN1
//!
//! # CAN2
//!
//! - TX = PB6 | PB13
//! - RX = PB5 | PB12
//! - Interrupt = CAN2

use crate::afio::MAPR;
#[cfg(feature = "connectivity")]
use crate::device::CAN2;
#[cfg(not(feature = "connectivity"))]
use crate::device::USB;
use crate::device::{can1, CAN1};
#[cfg(feature = "connectivity")]
use crate::gpio::gpiob::{PB12, PB13, PB5, PB6};
use crate::gpio::{
    gpioa::{PA11, PA12},
    gpiob::{PB8, PB9},
    Alternate, Floating, Input, PushPull,
};
use crate::rcc::{Enable, APB1};
use core::{convert::Infallible, convert::TryInto, marker::PhantomData};

#[derive(Clone, PartialEq, PartialOrd, Eq, Ord)]
pub struct Id {
    // standard part: 0x_FFE0_0000 //11 valid bits
    // extended part: 0x_FFFF_FFF8 //11+18 vaild bits
    // is_extended:   0x_0000_0004
    // is_rtr:        0x_0000_0002
    raw: u32,
}

//TODO put a part of this in the HAL as Id trait?
impl Id {
    pub fn new_standard(standard: u32) -> Id {
        // this could become a const function, when it becomes stable in Rust
        Id {
            raw: (standard & 0b_111_1111_1111) << 21,
        }
    }

    pub fn new_extended(extended: u32) -> Id {
        // this could become a const function, when it becomes stable in Rust
        Id {
            raw: 0b100 | ((extended & 0b_1_1111_1111_1111_1111_1111_1111_1111) << 3),
        }
    }

    pub fn with_rtr(&self) -> Id {
        Id {
            raw: self.raw | 0b10,
        }
    }

    fn from_received_register(register: u32) -> Id {
        Id { raw: register }
    }

    pub fn is_extended(&self) -> bool {
        0 != (self.raw & 0b100)
    }

    pub fn standard(&self) -> u16 {
        ((self.raw & 0b_1_111_1111_1110_0000_0000_0000_0000_0000) >> 21) as u16
    }

    pub fn extended(&self) -> u32 {
        (self.raw & 0b_1111_1111_1111_1111_1111_1111_1111_1000) >> 3
    }

    pub fn extended_part(&self) -> u32 {
        (self.raw & 0b_0_0000_0000_0011_1111_1111_1111_1111_1000) >> 3
    }

    pub fn is_rtr(&self) -> bool {
        0 != (self.raw & 0b10)
    }

    // these are private:
    fn as_32bit_filter(&self) -> u32 {
        // [31:0] = stdid[10:0], extid[17:0], IDE, RTR, 0
        self.raw
    }

    fn as_16bit_filter(&self) -> u32 {
        // [31:0] = stdid[10:0], IDE, RTR, extid[17:15]
        ((self.raw & 0b_1111_1111_1110_0000_0000_0000_0000_0000) >> 16)
            | ((self.raw & 0b1100) << 1)
            | ((self.raw & 0b_1_1100_0000_0000_0000_0000) >> 18)
    }
}

pub struct Frame {
    id: Id,
    dlc: usize,
    data: [u8; 8],
}

impl Frame {
    pub fn new(id: Id, data: &[u8]) -> Self {
        let mut frame = Self {
            id,
            dlc: data.len(),
            data: [0; 8],
        };
        frame.data[0..data.len()].copy_from_slice(data);
        frame
    }
    pub fn id(&self) -> &Id {
        &self.id
    }
    pub fn data(&self) -> &[u8] {
        &self.data[0..self.dlc]
    }
}

pub struct Configuration {
    /// In this mode, the internal counter of the CAN hardware is
    /// activated and used to generate the Time Stamp value stored
    /// in the CAN_RDTxR/CAN_TDTxR registers, respectively (for Rx
    /// and Tx mailboxes). The internal counter is captured on the
    /// sample point of the Start Of Frame bit in both reception and
    /// transmission.
    /// In this mode, if DLC == 8, the last two data bytes of the
    /// 8-byte message is replaced with a 16 bit timestamp: TIME[7:0]
    /// in data byte 6 and TIME[15:8] in data byte 7
    pub time_triggered_communication_mode: bool,

    /// Depending on the ABOM bit in the CAN_MCR register bxCAN will
    /// recover from Bus-Off (become error active again) either
    /// automatically or on software request. But in both cases the
    /// bxCAN has to wait at least for the recovery sequence specified
    /// in the CAN standard (128 occurrences of 11 consecutive recessive
    /// bits monitored on CANRX).
    /// If ABOM is set, the bxCAN will start the recovering sequence
    /// automatically after it has entered Bus-Off state.
    /// If ABOM is cleared, the software must initiate the recovering
    /// sequence by requesting bxCAN to enter and to leave initialization
    /// mode.
    pub automatic_bus_off_management: bool,

    /// On CAN bus activity detection, hardware automatically performs
    /// the wakeup sequence by clearing the SLEEP bit if the AWUM bit
    /// in the CAN_MCR register is set. If the AWUM bit is cleared,
    /// software has to clear the SLEEP bit when a wakeup interrupt
    /// occurs, in order to exit from Sleep mode.
    pub automatic_wake_up_mode: bool,

    /// This mode has been implemented in order to fulfil the requirement
    /// of the Time Triggered Communication option of the CAN standard.
    /// In this mode, each transmission is started only once. If the
    /// first attempt fails, due to an arbitration loss or an error, the
    /// hardware will not automatically restart the message transmission.
    /// At the end of the first transmission attempt, the hardware
    /// considers the request as completed and sets the RQCP bit in the
    /// CAN_TSR register. The result of the transmission is indicated in
    /// the CAN_TSR register by the TXOK, ALST and TERR bits.
    pub no_automatic_retransmission: bool,

    /// false: Once a receive FIFO is full the next incoming message will
    ///        overwrite the previous one.
    /// true:  Once a receive FIFO is full the next incoming message will
    ///        be discarded.
    pub receive_fifo_locked_mode: bool,

    /// The transmit mailboxes can be configured as a transmit FIFO by
    /// setting the TXFP bit in the CAN_MCR register. In this mode the
    /// priority order is given by the transmit request order.
    pub transmit_fifo_priority: bool,

    /// In Silent mode, the bxCAN is able to receive valid data frames
    /// and valid remote frames, but it sends only recessive bits on the
    /// CAN bus and it cannot start a transmission. If the bxCAN has to
    /// send a dominant bit (ACK bit, overload flag, active error flag),
    /// the bit is rerouted internally so that the CAN Core monitors
    /// this dominant bit, although the CAN bus may remain in recessive
    /// state. Silent mode can be used to analyze the traffic on a CAN
    /// bus without affecting it by the transmission of dominant bits
    /// (Acknowledge Bits, Error Frames).
    /// It is also possible to combine Loop Back mode and Silent mode.
    pub silent_mode: bool,

    /// In Loop Back Mode, the bxCAN treats its own transmitted messages
    /// as received messages and stores them (if they pass acceptance
    /// filtering) in a Receive mailbox.
    /// It is also possible to combine Loop Back mode and Silent mode
    pub loopback_mode: bool,

    /// Specifies the maximum number of time quanta the CAN hardware is allowed to
    /// lengthen or shorten a bit to perform resynchronization: 1TQ..4TQ  
    pub synchronisation_jump_width: u8,

    /// Specifies the number of time quanta in Bit Segment 1: 1TQ..16TQ
    /// defines the location of the sample point. It includes the PROP_SEG
    /// and PHASE_SEG1 of the CAN standard. Its duration is programmable
    /// between 1 and 16 time quanta but may be automatically lengthened
    /// to compensate for positive phase drifts due to differences in the
    /// frequency of the various nodes of the network.
    pub bit_segment_1: u8,

    /// Specifies the number of time quanta in Bit Segment 2: 1TQ..8TQ
    /// defines the location of the transmit point. It represents the
    /// PHASE_SEG2 of the CAN standard. Its duration is programmable
    /// between 1 and 8 time quanta but may also be automatically
    /// shortened to compensate for negative phase drifts.
    pub bit_segment_2: u8,

    /// Prescaling for time quantum: 1..1024
    /// Length of a time quanta: tq = (BRP[9:0]+1) x tPCLK
    pub time_quantum_length: u16,
}

#[cfg(not(feature = "connectivity"))]
pub const FILTERBANKCOUNT: FilterBankIndex = 14;
#[cfg(feature = "connectivity")]
pub const FILTERBANKCOUNT: FilterBankIndex = 28;
pub type FilterBankIndex = u8; //[0..FILTERBANKCOUNT-1] is valid

pub enum FilterMode {
    /// Identifier mask mode
    Mask,
    /// Identifier list mode
    List,
}

pub struct FilterData {
    /// Specifies the filter identification number
    pub id: Id,
    /// Specifies the filter mask number or a second identification number
    pub mask_or_id2: Id,
}

pub enum FilterInfo {
    /// One 32-bit filter for the STDID[10:0], EXTID[17:0], IDE and RTR bits.
    Whole(FilterData),
    /// Two 16-bit filters for the STDID[10:0], RTR, IDE and EXTID[17:15] bits (the rest of extid bits are ignored).
    Halves((FilterData, FilterData)),
}

pub struct FilterBankConfiguration {
    /// Specifies the filter mode to be InitializationMode.
    ///  This parameter can be a value of @ref CAN_filter_mode
    pub mode: FilterMode,

    pub info: FilterInfo,

    /// Specifies the FIFO (0 or 1) which will be assigned to the filter.
    /// This parameter can be a value of @ref CAN_filter_FIFO
    pub fifo_assignment: RxFifoIndex,

    /// Enable or disable the filter.
    pub active: bool,
}

#[derive(Debug)]
pub enum Error {
    Unexpected,     //we should never get this
    TooLongPayload, //tx mailbox can send up to 8 bytes at a time
    InvalidArgument,
    NotEmptyTxMailBox,
}

pub trait CanPeripheral {
    fn ptr() -> *const can1::RegisterBlock;
}

impl CanPeripheral for CAN1 {
    fn ptr() -> *const can1::RegisterBlock {
        CAN1::ptr()
    }
}

#[cfg(feature = "connectivity")]
impl CanPeripheral for CAN2 {
    fn ptr() -> *const can1::RegisterBlock {
        // Register block definitions are the same.
        CAN2::ptr() as *const can1::RegisterBlock
    }
}

pub trait Pins<CAN> {
    const REMAP: u8;
}

impl Pins<CAN1> for (PA12<Alternate<PushPull>>, PA11<Input<Floating>>) {
    const REMAP: u8 = 0b00;
}

impl Pins<CAN1> for (PB9<Alternate<PushPull>>, PB8<Input<Floating>>) {
    const REMAP: u8 = 0b10;
}

#[cfg(feature = "connectivity")]
impl Pins<CAN2> for (PB13<Alternate<PushPull>>, PB12<Input<Floating>>) {
    const REMAP: u8 = 0;
}

#[cfg(feature = "connectivity")]
impl Pins<CAN2> for (PB6<Alternate<PushPull>>, PB5<Input<Floating>>) {
    const REMAP: u8 = 1;
}

/// Connectivity line devcies (stm32105/stm32f107) share the filter banks
/// between the two CAN peripheral instances. The filter bank is clocked by
/// CAN1 and this struct represents an enabled CAN1/filter clock.
/// CAN2 takes ownership of the filter clock to make sure it is enabled.
/// The filter clock token can be obtained by
/// 1. `Can::can1()` which returns it in addition to its instance API struct
/// 2. `FilterClock::enable()` when only the CAN1 clock is required to
///     construct CAN2.
// Cannot be constructed outside of this module.
pub struct FilterClock(());

impl FilterClock {
    pub fn enable(_can: CAN1, apb1: &mut APB1) -> FilterClock {
        FilterClock::enable_priv(apb1)
    }

    fn enable_priv(apb1: &mut APB1) -> FilterClock {
        // power up CAN peripheral
        CAN1::enable(apb1);

        // delay after an RCC peripheral clock enabling
        apb1.enr().read();

        FilterClock(())
    }

    fn release(&self, apb1: &mut APB1) {
        CAN1::disable(apb1);
    }
}

/// CAN abstraction
pub struct Can<CAN, PINS>
where
    CAN: CanPeripheral,
{
    can: CAN,
    pins: PINS,
    clk: FilterClock,

    /// The USB and CAN share a dedicated 512-byte SRAM memory for data
    /// transmission and reception, and so they cannot be used concurrently
    /// (the shared SRAM is accessed through CAN and USB exclusively).
    /// The USB and CAN can be used in the same application but not
    /// at the same time.
    #[cfg(not(feature = "connectivity"))]
    _usb: USB,
}

/// To reduce power consumption, bxCAN has a low-power mode called Sleep mode.
/// In this mode, the bxCAN clock is stopped, however software can still access
/// the bxCAN mailboxes.
impl<PINS> Can<CAN1, PINS> {
    #[cfg(not(feature = "connectivity"))]
    pub fn can1(
        can: CAN1,
        pins: PINS,
        mapr: &mut MAPR,
        apb1: &mut APB1,
        usb: USB,
    ) -> Can<CAN1, PINS>
    where
        PINS: Pins<CAN1>,
    {
        // choose pin mapping
        #[allow(unused_unsafe)]
        mapr.modify_mapr(|_, w| unsafe { w.can_remap().bits(PINS::REMAP) });

        Can {
            can,
            pins,
            clk: FilterClock::enable_priv(apb1),
            _usb: usb,
        }
    }

    #[cfg(feature = "connectivity")]
    pub fn can1(
        can: CAN1,
        pins: PINS,
        mapr: &mut MAPR,
        apb1: &mut APB1,
    ) -> (Can<CAN1, PINS>, FilterClock)
    where
        PINS: Pins<CAN1>,
    {
        // choose pin mapping
        mapr.modify_mapr(|_, w| unsafe { w.can1_remap().bits(PINS::REMAP) });

        (
            Can {
                can,
                pins,
                clk: FilterClock::enable_priv(apb1),
            },
            FilterClock(()),
        )
    }

    /// releasing the resources
    /// (required for example to use USB instead of CAN)
    #[cfg(not(feature = "connectivity"))]
    pub fn release(self, apb1: &mut APB1) -> (CAN1, PINS, USB) {
        self.clk.release(apb1);
        (self.can, self.pins, self._usb)
    }

    #[cfg(feature = "connectivity")]
    pub fn release(self, _clk: FilterClock, apb1: &mut APB1) -> (CAN1, PINS) {
        self.clk.release(apb1);
        (self.can, self.pins)
    }
}

#[cfg(feature = "connectivity")]
impl<PINS> Can<CAN2, PINS> {
    pub fn can2(
        can: CAN2,
        clk: FilterClock,
        pins: PINS,
        mapr: &mut MAPR,
        apb1: &mut APB1,
    ) -> Can<CAN2, PINS>
    where
        PINS: Pins<CAN2>,
    {
        // power up CAN peripheral
        CAN1::enable(apb1);
        CAN2::enable(apb1);

        // delay after an RCC peripheral clock enabling
        apb1.enr().read();

        // choose pin mapping
        mapr.modify_mapr(|_, w| w.can2_remap().bit(PINS::REMAP == 1));

        Can { can, pins, clk }
    }

    /// releasing the resources
    /// (required for example to use USB instead of CAN)
    pub fn release(self, apb1: &mut APB1) -> (CAN2, PINS, FilterClock) {
        CAN2::disable(apb1);
        (self.can, self.pins, self.clk)
    }
}

impl<PINS, CAN> Can<CAN, PINS>
where
    CAN: CanPeripheral,
{
    /// moves from Sleep or Normal to Initialization mode
    fn to_initialization(&mut self) -> nb::Result<(), Infallible> {
        let can = unsafe { &*CAN::ptr() };
        let msr = can.msr.read();
        if msr.slak().bit_is_set() || msr.inak().bit_is_clear() {
            // request exit from sleep and enter initialization modes
            can.mcr.write(|w| w.sleep().clear_bit().inrq().set_bit());
            Err(nb::Error::WouldBlock)
        } else {
            Ok(())
        }
    }

    /// While in Initialization Mode, all message transfers to and from the
    /// CAN bus are stopped and the status of the CAN bus output CANTX is
    /// recessive (high).
    pub fn configure(&mut self, config: &Configuration) {
        let can = unsafe { &*CAN::ptr() };

        let slept = self.is_sleeping();

        // exit from sleep mode, request initialization
        while let Err(nb::Error::WouldBlock) = self.to_initialization() {}

        // Update register MCR
        can.mcr.modify(|_, w| {
            w.ttcm()
                .bit(config.time_triggered_communication_mode)
                .abom()
                .bit(config.automatic_bus_off_management)
                .awum()
                .bit(config.automatic_wake_up_mode)
                .nart()
                .bit(config.no_automatic_retransmission)
                .rflm()
                .bit(config.receive_fifo_locked_mode)
                .txfp()
                .bit(config.transmit_fifo_priority)
        });

        //Set the bit timing register
        can.btr.modify(|_, w| unsafe {
            w.silm()
                .bit(config.silent_mode)
                .lbkm()
                .bit(config.loopback_mode)
                .sjw()
                .bits(config.synchronisation_jump_width)
                .ts1()
                .bits(config.bit_segment_1 - 1)
                .ts2()
                .bits(config.bit_segment_2 - 1)
                .brp()
                .bits(config.time_quantum_length - 1)
        });

        //exit from initialization mode and return to the previous mode
        if slept {
            while let Err(nb::Error::WouldBlock) = self.to_sleep() {}
        } else {
            while let Err(nb::Error::WouldBlock) = self.to_normal() {}
        }
    }

    //TODO this below can not be called if the can if is splitted. - what to do?
    /// If automatic bus off management is disabled, the software must initiate
    /// the recovering sequence by requesting bxCAN to enter and to leave initialization
    /// mode. Note: In initialization mode, bxCAN does not monitor the CANRX signal,
    /// therefore it cannot complete the recovery sequence. To recover, bxCAN must be
    /// in normal mode.
    pub fn recover_from_bus_off(&mut self) {
        // exit from sleep mode, request initialization
        while let Err(nb::Error::WouldBlock) = self.to_initialization() {}
        while let Err(nb::Error::WouldBlock) = self.to_normal() {}
    }

    pub fn is_sleeping(&self) -> bool {
        let can = unsafe { &*CAN::ptr() };
        can.msr.read().slak().bit_is_set()
    }

    /// moves from Sleep to Normal mode
    pub fn to_normal(&mut self) -> nb::Result<(), Infallible> {
        let can = unsafe { &*CAN::ptr() };
        let msr = can.msr.read();
        if msr.slak().bit_is_set() || msr.inak().bit_is_set() {
            // request exit from both sleep and initialization modes
            can.mcr.write(|w| w.sleep().clear_bit().inrq().clear_bit());
            Err(nb::Error::WouldBlock)
        } else {
            Ok(())
        }
    }

    /// moves from Normal to Sleep mode
    pub fn to_sleep(&mut self) -> nb::Result<(), Infallible> {
        let can = unsafe { &*CAN::ptr() };
        let msr = can.msr.read();
        if msr.slak().bit_is_clear() || msr.inak().bit_is_set() {
            // request exit from both sleep and initialization modes
            can.mcr.write(|w| w.sleep().set_bit().inrq().clear_bit());
            Err(nb::Error::WouldBlock)
        } else {
            Ok(())
        }
    }

    pub fn activate_filter_bank(&mut self, index: FilterBankIndex, activate: bool) {
        let can = unsafe { &*CAN::ptr() };

        if index >= FILTERBANKCOUNT {
            return; // Err(Error::InvalidArgument);
        }

        let bit = 1u32 << index;

        if activate {
            can.fa1r.modify(|r, w| unsafe { w.bits(r.bits() | bit) });
        } else {
            can.fa1r.modify(|r, w| unsafe { w.bits(r.bits() & (!bit)) });
        }
    }

    pub fn configure_filter_bank(
        &mut self,
        index: FilterBankIndex,
        config: &FilterBankConfiguration,
    ) {
        let can = unsafe { &*CAN::ptr() };

        if index >= FILTERBANKCOUNT {
            return; // Err(Error::InvalidArgument);
        }

        // The filters values can be modified either deactivating the associated
        // filter banks or by setting the FINIT bit. Moreover, the modification
        // of the filter configuration (scale, mode and FIFO assignment) in
        // CAN_FMxR, CAN_FSxR and CAN_FFAR registers can only be done when the
        // filter initialization mode is set (FINIT=1) in the CAN_FMR register.

        let bit = 1u32 << index;
        let nbit = !bit;

        // Initialization mode for the filter
        can.fmr.write(|w| w.finit().bit(true));

        //Filter deactivation
        can.fa1r.modify(|r, w| unsafe { w.bits(r.bits() & nbit) });

        //Filter Mode
        match config.mode {
            FilterMode::Mask => {
                //Id/Mask mode for the filter
                can.fm1r.modify(|r, w| unsafe { w.bits(r.bits() & nbit) });
            }
            FilterMode::List => {
                //Identifier list mode for the filter
                can.fm1r.modify(|r, w| unsafe { w.bits(r.bits() | bit) });
            }
        };

        // Filter FIFO assignment
        match config.fifo_assignment {
            0 => {
                can.ffa1r.modify(|r, w| unsafe { w.bits(r.bits() & nbit) });
            }
            1 => {
                can.ffa1r.modify(|r, w| unsafe { w.bits(r.bits() | bit) });
            }
            _ => {
                //return Err(Error::InvalidArgument);
            }
        }

        //Depending on filter scale
        match config.info {
            FilterInfo::Halves((ref low, ref high)) => {
                // 16-bit scale for the filter
                can.fs1r.modify(|r, w| unsafe { w.bits(r.bits() & nbit) });

                // First 16-bit identifier and First 16-bit mask
                // Or First 16-bit identifier and Second 16-bit identifier
                // Second 16-bit identifier and Second 16-bit mask
                // Or Third 16-bit identifier and Fourth 16-bit identifier
                self.fill_filter_registers(
                    index,
                    low.id.as_16bit_filter() | (low.mask_or_id2.as_16bit_filter() << 16),
                    high.id.as_16bit_filter() | (high.mask_or_id2.as_16bit_filter() << 16),
                );
            }
            FilterInfo::Whole(ref whole) => {
                // 32-bit scale for the filter
                can.fs1r.modify(|r, w| unsafe { w.bits(r.bits() | bit) });

                //32-bit identifier or First 32-bit identifier,
                //32-bit mask or Second 32-bit identifier
                self.fill_filter_registers(
                    index,
                    whole.id.as_32bit_filter(),
                    whole.mask_or_id2.as_32bit_filter(),
                );
            }
        }

        // Filter activation
        if config.active {
            can.fa1r.modify(|r, w| unsafe { w.bits(r.bits() | bit) });
        }

        // Leave the initialisation mode for the filter
        can.fmr.write(|w| w.finit().bit(false));
    }

    //private helper function to get indexed access to the filter registers
    fn fill_filter_registers(&self, index: FilterBankIndex, r1: u32, r2: u32) {
        let can = unsafe { &*CAN::ptr() };
        can.fb[index as usize].fr1.write(|w| unsafe { w.bits(r1) });
        can.fb[index as usize].fr1.write(|w| unsafe { w.bits(r2) });
    }

    //TODO a join function may be needed also (which is the reverse of this one)...
    pub fn split(self) -> (Tx<CAN>, Rx<CAN>) {
        (Tx { _can: PhantomData }, Rx { _can: PhantomData })
    }

    /// interrupts for transmission:
    /// CAN_IT_TME - Transmit mailbox empty Interrupt
    ///
    /// interrupts for reception:
    /// CAN_IT_FMP0 - FIFO 0 message pending interrupt
    /// CAN_IT_FF0  - FIFO 0 full interrupt
    /// CAN_IT_FOV0 - FIFO 0 overrun interrupt
    /// CAN_IT_FMP1 - FIFO 1 message pending interrupt
    /// CAN_IT_FF1  - FIFO 1 full interrupt
    /// CAN_IT_FOV1 - FIFO 1 overrun interrupt
    ///
    /// Operating Mode Interrupts:
    /// CAN_IT_WKU - Wake-up interrupt
    /// CAN_IT_SLK - Sleep acknowledge interrupt
    ///
    /// Error Interrupts:
    /// CAN_IT_EWG - Error warning Interrupt
    /// CAN_IT_EPV - Error passive Interrupt
    /// CAN_IT_BOF - Bus-off Interrupt
    /// CAN_IT_LEC - Last error code Interrupt
    /// CAN_IT_ERR - Error Interrupt
    pub fn listen() {
        unimplemented!()
    }

    pub fn unlisten() {
        unimplemented!()
    }
}

//TODO put a part of this in the HAL as TransmitMailbox trait?
pub trait TransmitMailbox {
    const INDEX: TxMailBoxIndex;
    fn is_empty(&self) -> bool;
    fn has_the_lowest_priority(&self) -> bool;
    fn was_transmission_error(&self) -> bool;
    fn was_arbitration_lost(&self) -> bool;
    fn has_transmission_succeeded(&self) -> bool;
    fn has_last_request_completed(&self) -> bool;
    fn request_abort_transmit(&mut self);

    /// In order to transmit a message, the application must select
    /// one empty transmit mailbox, set up the identifier, the data
    /// length code (DLC) and the data before requesting the transmission
    /// Once the mailbox has left empty state, the software no longer
    /// has write access to the mailbox registers.
    /// The hardware indicates a successful transmission by setting
    /// the RQCP and TXOK bits in the CAN_TSR register.
    fn request_transmit(&mut self, frame: &Frame) -> Result<(), Error>;
}

/// Can transmitter mailboxes
pub struct Tx<CAN> {
    _can: PhantomData<CAN>,
}

impl<CAN> Tx<CAN> {
    pub fn split(
        self,
    ) -> (
        TxMailBox<CAN, TxMailBox0>,
        TxMailBox<CAN, TxMailBox1>,
        TxMailBox<CAN, TxMailBox2>,
    ) {
        (
            TxMailBox {
                _can: PhantomData,
                _index: PhantomData,
            },
            TxMailBox {
                _can: PhantomData,
                _index: PhantomData,
            },
            TxMailBox {
                _can: PhantomData,
                _index: PhantomData,
            },
        )
    }
}

/// Can receiver fifos
pub struct Rx<CAN> {
    _can: PhantomData<CAN>,
}

impl<CAN> Rx<CAN> {
    pub fn split(self) -> (RxFifo<CAN, RxFifo0>, RxFifo<CAN, RxFifo1>) {
        (
            RxFifo {
                _can: PhantomData,
                _index: PhantomData,
            },
            RxFifo {
                _can: PhantomData,
                _index: PhantomData,
            },
        )
    }
}

pub struct TxMailBox<CAN, IDX> {
    _can: PhantomData<CAN>,
    _index: PhantomData<IDX>,
}

pub type TxMailBoxIndex = u8; //[0..2] is valid

//type state
pub struct TxMailBox0;
pub struct TxMailBox1;
pub struct TxMailBox2;

macro_rules! TxMailBox {
    ($CANX:ident, [
        $($TxMailBoxi:ident: ($i:expr, $tmei:ident, $lowi:ident, $terri:ident,
        $alsti:ident, $txoki:ident, $rqcpi:ident, $abrqi:ident),)+
    ]) => {
        $(
            impl TransmitMailbox for TxMailBox<$CANX, $TxMailBoxi>
            {
                const INDEX: TxMailBoxIndex = $i;

                fn is_empty(&self) -> bool {
                    // NOTE(unsafe) atomic read with no side effects
                    let tsr = unsafe { &*$CANX::ptr() }.tsr.read();
                    tsr.$tmei().bit_is_set()
                }

                fn has_the_lowest_priority(&self) -> bool {
                    // NOTE(unsafe) atomic read with no side effects
                    let tsr = unsafe { &*$CANX::ptr() }.tsr.read();
                    tsr.$lowi().bit_is_set()
                }

                fn was_transmission_error(&self) -> bool {
                    // NOTE(unsafe) atomic read with no side effects
                    let tsr = unsafe { &*$CANX::ptr() }.tsr.read();
                    tsr.$terri().bit_is_set()
                }
                fn was_arbitration_lost(&self) -> bool {
                    // NOTE(unsafe) atomic read with no side effects
                    let tsr = unsafe { &*$CANX::ptr() }.tsr.read();
                    tsr.$alsti().bit_is_set()
                }

                fn has_transmission_succeeded(&self) -> bool {
                    // NOTE(unsafe) atomic read with no side effects
                    let tsr = unsafe { &*$CANX::ptr() }.tsr.read();
                    tsr.$txoki().bit_is_set()
                }

                // //16-bit timer value captured at the SOF transmission.
                // fn get_transmission_time(&mut self) -> u16 {
                //     unsafe { (*$CANX::ptr()).$can_tdtir.read().time().bits() }
                // }

                fn has_last_request_completed(&self) -> bool {
                    // NOTE(unsafe) atomic read with no side effects
                    let tsr = unsafe { &*$CANX::ptr() }.tsr.read();
                    tsr.$rqcpi().bit_is_set()
                }

                fn request_abort_transmit(&mut self) {
                    unsafe { &*$CANX::ptr() }.tsr.write(|w| w.$abrqi().set_bit());
                }

                //TODO non blocking on the top of this:
                //TODO use a message struct as input
                fn request_transmit(&mut self, frame: &Frame) -> Result<(), Error> {
                    if self.is_empty() {
                        if frame.dlc > 8 {
                            return Err(Error::TooLongPayload);
                        }

                        let tx = &(unsafe { &*$CANX::ptr() }.tx[$i]);

                        //fill message length [0..8]
                        tx.tdtr.write(|w| unsafe { w.dlc().bits(frame.dlc as u8) });

                        tx.tdlr.write(|w| unsafe { w.bits(u32::from_ne_bytes(frame.data[0..4].try_into().unwrap())) });
                        tx.tdhr.write(|w| unsafe { w.bits(u32::from_ne_bytes(frame.data[4..8].try_into().unwrap())) });

                        // Bits 31:21 STID[10:0]: Standard Identifier
                        //             The standard part of the identifier.
                        // Bit 20:3 EXID[17:0]: Extended Identifier
                        //             The extended part of the identifier.
                        // Bit 2 IDE: Identifier Extension
                        //             This bit defines the identifier type of message in the mailbox.
                        //             0: Standard identifier.
                        //             1: Extended identifier.
                        // Bit 1 RTR: Remote Transmission Request
                        //             0: Data frame
                        //             1: Remote frame
                        // Bit 0 TXRQ: Transmit Mailbox Request
                        //             Set by software to request the transmission for the corresponding mailbox.
                        //             Cleared by hardware when the mailbox becomes empty.
                        let id = &frame.id;
                        tx.tir.write(|w| unsafe { w
                            .stid().bits(id.standard())
                            .exid().bits(id.extended_part())
                            .ide().bit(id.is_extended())
                            .rtr().bit(id.is_rtr())
                            .txrq().set_bit() //request transmit
                        });
                        Ok(())
                    } else {
                        //this mailbox is not empty, so return the index of the less
                        //important mailbox as candidate for request_abort_transmit
                        Err(Error::NotEmptyTxMailBox)
                    }
                }
            }
        )+
    }
}

TxMailBox!(CAN1, [
    TxMailBox0: (0, tme0, low0, terr0, alst0, txok0, rqcp0, abrq0),
    TxMailBox1: (1, tme1, low1, terr1, alst1, txok1, rqcp1, abrq1),
    TxMailBox2: (2, tme2, low2, terr2, alst2, txok2, rqcp2, abrq2),
]);

#[cfg(feature = "connectivity")]
TxMailBox!(CAN2, [
    TxMailBox0: (0, tme0, low0, terr0, alst0, txok0, rqcp0, abrq0),
    TxMailBox1: (1, tme1, low1, terr1, alst1, txok1, rqcp1, abrq1),
    TxMailBox2: (2, tme2, low2, terr2, alst2, txok2, rqcp2, abrq2),
]);

pub type RxFifoIndex = u8; //[0..1] is valid
pub type FilterMatchIndex = u8;
pub type TimeStamp = u16;

//TODO put a part of this in the HAL as ReceiveFifo trait?
pub trait ReceiveFifo {
    const INDEX: RxFifoIndex;
    fn has_overun(&self) -> bool;
    fn is_full(&self) -> bool;
    fn pending_count(&self) -> u8;
    fn read(&mut self) -> nb::Result<(FilterMatchIndex, TimeStamp, Frame), Infallible>;
}

//type state
pub struct RxFifo0;
pub struct RxFifo1;

pub struct RxFifo<CAN, IDX> {
    _can: PhantomData<CAN>,
    _index: PhantomData<IDX>,
}

macro_rules! RxFifo {
    ($CANX:ident, [
        $($RxFifoi:ident: ($i:expr, ),)+
    ]) => {
        $(
            impl ReceiveFifo for RxFifo<$CANX, $RxFifoi> {
                const INDEX: RxFifoIndex = $i;

                fn has_overun(&self) -> bool {
                    unsafe { &*$CANX::ptr() }.rfr[$i].read().fovr().bit()
                }

                fn is_full(&self) -> bool {
                    unsafe { &*$CANX::ptr() }.rfr[$i].read().full().bit()
                }

                fn pending_count(&self) -> u8 {
                     unsafe { &*$CANX::ptr() }.rfr[$i].read().fmp().bits()
                }

                fn read(&mut self) -> nb::Result<(FilterMatchIndex, TimeStamp, Frame), Infallible> {
                    let n = self.pending_count();
                    if n < 1 {
                        //there are no messages in the fifo
                        Err(nb::Error::WouldBlock)
                    } else {
                        let rx = &(unsafe { &*$CANX::ptr() }.rx[$i]);
                        let rdtir = rx.rdtr.read();
                        let filter_match_index = rdtir.fmi().bits();
                        let time = rdtir.time().bits();

                        let mut frame = Frame {
                            id: Id::from_received_register(rx.rir.read().bits()),
                            dlc: rdtir.dlc().bits() as usize,
                            data: [0; 8],
                        };
                        frame.data[0..4].copy_from_slice(&rx.rdlr.read().bits().to_ne_bytes());
                        frame.data[4..8].copy_from_slice(&rx.rdhr.read().bits().to_ne_bytes());

                        //after every info captured release fifo output mailbox:
                        unsafe { &*$CANX::ptr() }.rfr[$i].write(|w| w.rfom().set_bit());

                        Ok((filter_match_index, time, frame))
                    }
                }
            }

        )+
    }
}

RxFifo!(CAN1, [
    RxFifo0: (0,),
    RxFifo1: (1,),
]);

#[cfg(feature = "connectivity")]
RxFifo!(CAN2, [
    RxFifo0: (0,),
    RxFifo1: (1,),
]);
