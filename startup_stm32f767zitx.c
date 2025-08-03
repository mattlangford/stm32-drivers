// This is converted from this assemlby file (I had trouble getting it building with bazel)
// https://github.com/Plunne/STM32_LED/blob/main/Core/Startup/startup_stm32f767zitx.s
// These weak references can be overriden by:
// ```c++
// extern "C" void SysTick_Handler() { ... }
// ```

#include <stdint.h>

/* Symbols defined in the linker script */
extern uint32_t _sidata;   /* start of init data in flash */
extern uint32_t _sdata;    /* start of .data in RAM */
extern uint32_t _edata;    /* end of .data in RAM */
extern uint32_t _sbss;     /* start of .bss in RAM */
extern uint32_t _ebss;     /* end of .bss in RAM */
extern uint32_t _estack;   /* end of stack (initial SP) */

/* Forward declarations of the default handlers */
void Reset_Handler(void);
void Default_Handler(void);

#define WEAK_DEFAULT_HANDLER(name) \
    void name(void) __attribute__ ((weak, alias("Default_Handler")))

/* Cortex-M exceptions */
WEAK_DEFAULT_HANDLER(NMI_Handler);
WEAK_DEFAULT_HANDLER(HardFault_Handler);
WEAK_DEFAULT_HANDLER(MemManage_Handler);
WEAK_DEFAULT_HANDLER(BusFault_Handler);
WEAK_DEFAULT_HANDLER(UsageFault_Handler);
WEAK_DEFAULT_HANDLER(SVC_Handler);
WEAK_DEFAULT_HANDLER(DebugMon_Handler);
WEAK_DEFAULT_HANDLER(PendSV_Handler);
WEAK_DEFAULT_HANDLER(SysTick_Handler);

/* STM32F7 peripheral IRQ handlers */
WEAK_DEFAULT_HANDLER(WWDG_IRQHandler);
WEAK_DEFAULT_HANDLER(PVD_IRQHandler);
WEAK_DEFAULT_HANDLER(TAMP_STAMP_IRQHandler);
WEAK_DEFAULT_HANDLER(RTC_WKUP_IRQHandler);
WEAK_DEFAULT_HANDLER(FLASH_IRQHandler);
WEAK_DEFAULT_HANDLER(RCC_IRQHandler);
WEAK_DEFAULT_HANDLER(EXTI0_IRQHandler);
WEAK_DEFAULT_HANDLER(EXTI1_IRQHandler);
WEAK_DEFAULT_HANDLER(EXTI2_IRQHandler);
WEAK_DEFAULT_HANDLER(EXTI3_IRQHandler);
WEAK_DEFAULT_HANDLER(EXTI4_IRQHandler);
WEAK_DEFAULT_HANDLER(DMA1_Stream0_IRQHandler);
WEAK_DEFAULT_HANDLER(DMA1_Stream1_IRQHandler);
WEAK_DEFAULT_HANDLER(DMA1_Stream2_IRQHandler);
WEAK_DEFAULT_HANDLER(DMA1_Stream3_IRQHandler);
WEAK_DEFAULT_HANDLER(DMA1_Stream4_IRQHandler);
WEAK_DEFAULT_HANDLER(DMA1_Stream5_IRQHandler);
WEAK_DEFAULT_HANDLER(DMA1_Stream6_IRQHandler);
WEAK_DEFAULT_HANDLER(ADC_IRQHandler);
WEAK_DEFAULT_HANDLER(CAN1_TX_IRQHandler);
WEAK_DEFAULT_HANDLER(CAN1_RX0_IRQHandler);
WEAK_DEFAULT_HANDLER(CAN1_RX1_IRQHandler);
WEAK_DEFAULT_HANDLER(CAN1_SCE_IRQHandler);
WEAK_DEFAULT_HANDLER(EXTI9_5_IRQHandler);
WEAK_DEFAULT_HANDLER(TIM1_BRK_TIM9_IRQHandler);
WEAK_DEFAULT_HANDLER(TIM1_UP_TIM10_IRQHandler);
WEAK_DEFAULT_HANDLER(TIM1_TRG_COM_TIM11_IRQHandler);
WEAK_DEFAULT_HANDLER(TIM1_CC_IRQHandler);
WEAK_DEFAULT_HANDLER(TIM2_IRQHandler);
WEAK_DEFAULT_HANDLER(TIM3_IRQHandler);
WEAK_DEFAULT_HANDLER(TIM4_IRQHandler);
WEAK_DEFAULT_HANDLER(I2C1_EV_IRQHandler);
WEAK_DEFAULT_HANDLER(I2C1_ER_IRQHandler);
WEAK_DEFAULT_HANDLER(I2C2_EV_IRQHandler);
WEAK_DEFAULT_HANDLER(I2C2_ER_IRQHandler);
WEAK_DEFAULT_HANDLER(SPI1_IRQHandler);
WEAK_DEFAULT_HANDLER(SPI2_IRQHandler);
WEAK_DEFAULT_HANDLER(USART1_IRQHandler);
WEAK_DEFAULT_HANDLER(USART2_IRQHandler);
WEAK_DEFAULT_HANDLER(USART3_IRQHandler);
WEAK_DEFAULT_HANDLER(EXTI15_10_IRQHandler);
WEAK_DEFAULT_HANDLER(RTC_Alarm_IRQHandler);
WEAK_DEFAULT_HANDLER(OTG_FS_WKUP_IRQHandler);
WEAK_DEFAULT_HANDLER(TIM8_BRK_TIM12_IRQHandler);
WEAK_DEFAULT_HANDLER(TIM8_UP_TIM13_IRQHandler);
WEAK_DEFAULT_HANDLER(TIM8_TRG_COM_TIM14_IRQHandler);
WEAK_DEFAULT_HANDLER(TIM8_CC_IRQHandler);
WEAK_DEFAULT_HANDLER(DMA1_Stream7_IRQHandler);
WEAK_DEFAULT_HANDLER(FMC_IRQHandler);
WEAK_DEFAULT_HANDLER(SDMMC1_IRQHandler);
WEAK_DEFAULT_HANDLER(TIM5_IRQHandler);
WEAK_DEFAULT_HANDLER(SPI3_IRQHandler);
WEAK_DEFAULT_HANDLER(UART4_IRQHandler);
WEAK_DEFAULT_HANDLER(UART5_IRQHandler);
WEAK_DEFAULT_HANDLER(TIM6_DAC_IRQHandler);
WEAK_DEFAULT_HANDLER(TIM7_IRQHandler);
WEAK_DEFAULT_HANDLER(DMA2_Stream0_IRQHandler);
WEAK_DEFAULT_HANDLER(DMA2_Stream1_IRQHandler);
WEAK_DEFAULT_HANDLER(DMA2_Stream2_IRQHandler);
WEAK_DEFAULT_HANDLER(DMA2_Stream3_IRQHandler);
WEAK_DEFAULT_HANDLER(DMA2_Stream4_IRQHandler);
WEAK_DEFAULT_HANDLER(ETH_IRQHandler);
WEAK_DEFAULT_HANDLER(ETH_WKUP_IRQHandler);
WEAK_DEFAULT_HANDLER(CAN2_TX_IRQHandler);
WEAK_DEFAULT_HANDLER(CAN2_RX0_IRQHandler);
WEAK_DEFAULT_HANDLER(CAN2_RX1_IRQHandler);
WEAK_DEFAULT_HANDLER(CAN2_SCE_IRQHandler);
WEAK_DEFAULT_HANDLER(OTG_FS_IRQHandler);
WEAK_DEFAULT_HANDLER(DMA2_Stream5_IRQHandler);
WEAK_DEFAULT_HANDLER(DMA2_Stream6_IRQHandler);
WEAK_DEFAULT_HANDLER(DMA2_Stream7_IRQHandler);
WEAK_DEFAULT_HANDLER(USART6_IRQHandler);
WEAK_DEFAULT_HANDLER(I2C3_EV_IRQHandler);
WEAK_DEFAULT_HANDLER(I2C3_ER_IRQHandler);
WEAK_DEFAULT_HANDLER(OTG_HS_EP1_OUT_IRQHandler);
WEAK_DEFAULT_HANDLER(OTG_HS_EP1_IN_IRQHandler);
WEAK_DEFAULT_HANDLER(OTG_HS_WKUP_IRQHandler);
WEAK_DEFAULT_HANDLER(OTG_HS_IRQHandler);
WEAK_DEFAULT_HANDLER(DCMI_IRQHandler);
/* Reserved slot */
/* next IRQs */
WEAK_DEFAULT_HANDLER(RNG_IRQHandler);
WEAK_DEFAULT_HANDLER(FPU_IRQHandler);
WEAK_DEFAULT_HANDLER(UART7_IRQHandler);
WEAK_DEFAULT_HANDLER(UART8_IRQHandler);
WEAK_DEFAULT_HANDLER(SPI4_IRQHandler);
WEAK_DEFAULT_HANDLER(SPI5_IRQHandler);
WEAK_DEFAULT_HANDLER(SPI6_IRQHandler);
WEAK_DEFAULT_HANDLER(SAI1_IRQHandler);
WEAK_DEFAULT_HANDLER(LTDC_IRQHandler);
WEAK_DEFAULT_HANDLER(LTDC_ER_IRQHandler);
WEAK_DEFAULT_HANDLER(DMA2D_IRQHandler);
WEAK_DEFAULT_HANDLER(SAI2_IRQHandler);
WEAK_DEFAULT_HANDLER(QUADSPI_IRQHandler);
WEAK_DEFAULT_HANDLER(LPTIM1_IRQHandler);
WEAK_DEFAULT_HANDLER(CEC_IRQHandler);
WEAK_DEFAULT_HANDLER(I2C4_EV_IRQHandler);
WEAK_DEFAULT_HANDLER(I2C4_ER_IRQHandler);
WEAK_DEFAULT_HANDLER(SPDIF_RX_IRQHandler);
/* Reserved */
WEAK_DEFAULT_HANDLER(DFSDM1_FLT0_IRQHandler);
WEAK_DEFAULT_HANDLER(DFSDM1_FLT1_IRQHandler);
WEAK_DEFAULT_HANDLER(DFSDM1_FLT2_IRQHandler);
WEAK_DEFAULT_HANDLER(DFSDM1_FLT3_IRQHandler);
WEAK_DEFAULT_HANDLER(SDMMC2_IRQHandler);
WEAK_DEFAULT_HANDLER(CAN3_TX_IRQHandler);
WEAK_DEFAULT_HANDLER(CAN3_RX0_IRQHandler);
WEAK_DEFAULT_HANDLER(CAN3_RX1_IRQHandler);
WEAK_DEFAULT_HANDLER(CAN3_SCE_IRQHandler);
WEAK_DEFAULT_HANDLER(JPEG_IRQHandler);
WEAK_DEFAULT_HANDLER(MDIOS_IRQHandler);

/* External references (usually in system_stm32f7xx.c or CMSIS) */
extern void SystemInit(void);
extern void __libc_init_array(void);
extern int main(void);

/* Vector table placed at start of flash */
__attribute__ ((section(".isr_vector")))
void (* const g_pfnVectors[])(void) = {
    /* Stack pointer */
    (void (*)(void))(&_estack),
    /* Cortex-M handlers */
    Reset_Handler,
    NMI_Handler,
    HardFault_Handler,
    MemManage_Handler,
    BusFault_Handler,
    UsageFault_Handler,
    (void (*)(void))0,
    (void (*)(void))0,
    (void (*)(void))0,
    (void (*)(void))0,
    SVC_Handler,
    DebugMon_Handler,
    (void (*)(void))0,
    PendSV_Handler,
    SysTick_Handler,
    /* STM32F7 interrupts */
    WWDG_IRQHandler, PVD_IRQHandler, TAMP_STAMP_IRQHandler, RTC_WKUP_IRQHandler,
    FLASH_IRQHandler, RCC_IRQHandler, EXTI0_IRQHandler, EXTI1_IRQHandler,
    EXTI2_IRQHandler, EXTI3_IRQHandler, EXTI4_IRQHandler, DMA1_Stream0_IRQHandler,
    DMA1_Stream1_IRQHandler, DMA1_Stream2_IRQHandler, DMA1_Stream3_IRQHandler,
    DMA1_Stream4_IRQHandler, DMA1_Stream5_IRQHandler, DMA1_Stream6_IRQHandler,
    ADC_IRQHandler, CAN1_TX_IRQHandler, CAN1_RX0_IRQHandler, CAN1_RX1_IRQHandler,
    CAN1_SCE_IRQHandler, EXTI9_5_IRQHandler, TIM1_BRK_TIM9_IRQHandler,
    TIM1_UP_TIM10_IRQHandler, TIM1_TRG_COM_TIM11_IRQHandler, TIM1_CC_IRQHandler,
    TIM2_IRQHandler, TIM3_IRQHandler, TIM4_IRQHandler, I2C1_EV_IRQHandler,
    I2C1_ER_IRQHandler, I2C2_EV_IRQHandler, I2C2_ER_IRQHandler, SPI1_IRQHandler,
    SPI2_IRQHandler, USART1_IRQHandler, USART2_IRQHandler, USART3_IRQHandler,
    EXTI15_10_IRQHandler, RTC_Alarm_IRQHandler, OTG_FS_WKUP_IRQHandler,
    TIM8_BRK_TIM12_IRQHandler, TIM8_UP_TIM13_IRQHandler,
    TIM8_TRG_COM_TIM14_IRQHandler, TIM8_CC_IRQHandler, DMA1_Stream7_IRQHandler,
    FMC_IRQHandler, SDMMC1_IRQHandler, TIM5_IRQHandler, SPI3_IRQHandler,
    UART4_IRQHandler, UART5_IRQHandler, TIM6_DAC_IRQHandler, TIM7_IRQHandler,
    DMA2_Stream0_IRQHandler, DMA2_Stream1_IRQHandler, DMA2_Stream2_IRQHandler,
    DMA2_Stream3_IRQHandler, DMA2_Stream4_IRQHandler, ETH_IRQHandler,
    ETH_WKUP_IRQHandler, CAN2_TX_IRQHandler, CAN2_RX0_IRQHandler,
    CAN2_RX1_IRQHandler, CAN2_SCE_IRQHandler, OTG_FS_IRQHandler,
    DMA2_Stream5_IRQHandler, DMA2_Stream6_IRQHandler, DMA2_Stream7_IRQHandler,
    USART6_IRQHandler, I2C3_EV_IRQHandler, I2C3_ER_IRQHandler,
    OTG_HS_EP1_OUT_IRQHandler, OTG_HS_EP1_IN_IRQHandler,
    OTG_HS_WKUP_IRQHandler, OTG_HS_IRQHandler, DCMI_IRQHandler,
    (void (*)(void))0,  /* Reserved */
    RNG_IRQHandler, FPU_IRQHandler, UART7_IRQHandler, UART8_IRQHandler,
    SPI4_IRQHandler, SPI5_IRQHandler, SPI6_IRQHandler, SAI1_IRQHandler,
    LTDC_IRQHandler, LTDC_ER_IRQHandler, DMA2D_IRQHandler, SAI2_IRQHandler,
    QUADSPI_IRQHandler, LPTIM1_IRQHandler, CEC_IRQHandler, I2C4_EV_IRQHandler,
    I2C4_ER_IRQHandler, SPDIF_RX_IRQHandler, (void (*)(void))0, /* Reserved */
    DFSDM1_FLT0_IRQHandler, DFSDM1_FLT1_IRQHandler, DFSDM1_FLT2_IRQHandler,
    DFSDM1_FLT3_IRQHandler, SDMMC2_IRQHandler, CAN3_TX_IRQHandler,
    CAN3_RX0_IRQHandler, CAN3_RX1_IRQHandler, CAN3_SCE_IRQHandler,
    JPEG_IRQHandler, MDIOS_IRQHandler
};

/* Reset handler in C */
void Reset_Handler(void)
{
    uint32_t *src  = &_sidata;
    uint32_t *dest = &_sdata;

    /* Copy .data from flash to SRAM */
    while (dest < &_edata) {
        *dest++ = *src++;
    }
    /* Zero-fill .bss */
    for (uint32_t *b = &_sbss; b < &_ebss; ) {
        *b++ = 0;
    }
    /* Initialize system (clock setup) */
    SystemInit();
    /* Run C++ static constructors, etc. */
    __libc_init_array();
    /* Enter main */
    main();
    /* If main returns, loop forever */
    while (1) { }
}

/* Default interrupt handler */
void Default_Handler(void)
{
    while (1) { }
}
