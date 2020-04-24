pub use crate::adc::ChannelTimeSequence as _stm32_hal_adc_ChannelTimeSequence;
pub use crate::afio::AfioExt as _stm32_hal_afio_AfioExt;
#[cfg(any(feature = "stm32f103", feature = "connectivity"))]
pub use crate::can::ReceiveFifo as _stm32_hal_can_ReceiveFifo;
#[cfg(any(feature = "stm32f103", feature = "connectivity"))]
pub use crate::can::TransmitMailbox as _stm32_hal_can_TransmitMailbox;
pub use crate::dma::CircReadDma as _stm32_hal_dma_CircReadDma;
pub use crate::dma::DmaExt as _stm32_hal_dma_DmaExt;
pub use crate::dma::ReadDma as _stm32_hal_dma_ReadDma;
pub use crate::dma::WriteDma as _stm32_hal_dma_WriteDma;
pub use crate::flash::FlashExt as _stm32_hal_flash_FlashExt;
pub use crate::gpio::GpioExt as _stm32_hal_gpio_GpioExt;
pub use crate::hal::adc::OneShot as _embedded_hal_adc_OneShot;
pub use crate::hal::digital::v2::StatefulOutputPin as _embedded_hal_digital_StatefulOutputPin;
pub use crate::hal::digital::v2::ToggleableOutputPin as _embedded_hal_digital_ToggleableOutputPin;
pub use crate::hal::prelude::*;
pub use crate::rcc::RccExt as _stm32_hal_rcc_RccExt;
pub use crate::time::U32Ext as _stm32_hal_time_U32Ext;
