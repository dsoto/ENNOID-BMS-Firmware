#include <stdint.h>

// base address and size of SRAM program location
#define SRAM_BASE       0x20000000U
#define SRAM_SIZE       40*1024

// base address and size of SRAM program location
#define SRAM_BASE       0x20000000U
#define SRAM_SIZE       40*1024
#define SRAM_END        (SRAM_BASE + SRAM_SIZE)

void main();
void __libc_init_array(void);

void Reset_Handler(void);
// put in prototypes
// void Reset_Handler               (void) __attribute__ ((weak, alias("Default_Handler")));
void NMI_Handler                 (void) __attribute__ ((weak, alias("Default_Handler")));
void HardFault_Handler           (void) __attribute__ ((weak, alias("Default_Handler")));
void MemManage_Handler           (void) __attribute__ ((weak, alias("Default_Handler")));
void BusFault_Handler            (void) __attribute__ ((weak, alias("Default_Handler")));
void UsageFault_Handler          (void) __attribute__ ((weak, alias("Default_Handler")));
void SVCall_Handler              (void) __attribute__ ((weak, alias("Default_Handler")));
void PendSV_Handler              (void) __attribute__ ((weak, alias("Default_Handler")));
void SysTick_Handler             (void) __attribute__ ((weak, alias("Default_Handler")));
void WWDG_Handler                (void) __attribute__ ((weak, alias("Default_Handler")));
void PVD_Handler                 (void) __attribute__ ((weak, alias("Default_Handler")));
void TAMPER_STAMP_Handler        (void) __attribute__ ((weak, alias("Default_Handler")));
void RTC_WKUP_Handler            (void) __attribute__ ((weak, alias("Default_Handler")));
void FLASH_Handler               (void) __attribute__ ((weak, alias("Default_Handler")));
void RCC_Handler                 (void) __attribute__ ((weak, alias("Default_Handler")));
void EXTI0_Handler               (void) __attribute__ ((weak, alias("Default_Handler")));
void EXTI1_Handler               (void) __attribute__ ((weak, alias("Default_Handler")));
void EXTI2_TS_Handler            (void) __attribute__ ((weak, alias("Default_Handler")));
void EXTI3_Handler               (void) __attribute__ ((weak, alias("Default_Handler")));
void EXTI4_Handler               (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA1_Channel1_Handler       (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA1_Channel2_Handler       (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA1_Channel3_Handler       (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA1_Channel4_Handler       (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA1_Channel5_Handler       (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA1_Channel6_Handler       (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA1_Channel7_Handler       (void) __attribute__ ((weak, alias("Default_Handler")));
void ADC1_2_Handler              (void) __attribute__ ((weak, alias("Default_Handler")));
void USB_HP_CAN_TX_Handler       (void) __attribute__ ((weak, alias("Default_Handler")));
void USB_LP_CAN_RX0_Handler      (void) __attribute__ ((weak, alias("Default_Handler")));
void CAN_RX1_Handler             (void) __attribute__ ((weak, alias("Default_Handler")));
void CAN_SCE_Handler             (void) __attribute__ ((weak, alias("Default_Handler")));
void EXTI9_5_Handler             (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM1_BRK_TIM15_Handler      (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM1_UP_TIM16_Handler       (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM1_TRG_COM_TIM17_Handler  (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM1_CC_Handler             (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM2_Handler                (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM3_Handler                (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM4_Handler                (void) __attribute__ ((weak, alias("Default_Handler")));
void I2C1_EV_Handler             (void) __attribute__ ((weak, alias("Default_Handler")));
void I2C1_ER_Handler             (void) __attribute__ ((weak, alias("Default_Handler")));
void I2C2_EV_Handler             (void) __attribute__ ((weak, alias("Default_Handler")));
void I2C2_ER_Handler             (void) __attribute__ ((weak, alias("Default_Handler")));
void SPI1_Handler                (void) __attribute__ ((weak, alias("Default_Handler")));
void SPI2_Handler                (void) __attribute__ ((weak, alias("Default_Handler")));
void USART1_Handler              (void) __attribute__ ((weak, alias("Default_Handler")));
void USART2_Handler              (void) __attribute__ ((weak, alias("Default_Handler")));
void USART3_Handler              (void) __attribute__ ((weak, alias("Default_Handler")));
void EXTI15_10_Handler           (void) __attribute__ ((weak, alias("Default_Handler")));
void RTC_Alarm_Handler           (void) __attribute__ ((weak, alias("Default_Handler")));
void USBWakeUP_Handler           (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM8_BRK_Handler            (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM8_UP_Handler             (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM8_TRG_COM_Handler        (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM8_CC_Handler             (void) __attribute__ ((weak, alias("Default_Handler")));
void ADC3_Handler                (void) __attribute__ ((weak, alias("Default_Handler")));
void FMC_Handler                 (void) __attribute__ ((weak, alias("Default_Handler")));
void SPI3_Handler                (void) __attribute__ ((weak, alias("Default_Handler")));
void UART4_Handler               (void) __attribute__ ((weak, alias("Default_Handler")));
void UART5_Handler               (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM6_DAC_Handler            (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM7_Handler                (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA2_Channel1_Handler       (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA2_Channel2_Handler       (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA2_Channel3_Handler       (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA2_Channel4_Handler       (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA2_Channel5_Handler       (void) __attribute__ ((weak, alias("Default_Handler")));
void ADC4_Handler                (void) __attribute__ ((weak, alias("Default_Handler")));
void COMP_1_2_3_Handler          (void) __attribute__ ((weak, alias("Default_Handler")));
void COMP_4_5_6_Handler          (void) __attribute__ ((weak, alias("Default_Handler")));
void COMP7_Handler               (void) __attribute__ ((weak, alias("Default_Handler")));
void I2C3_EV_Handler             (void) __attribute__ ((weak, alias("Default_Handler")));
void I2C3_ER_Handler             (void) __attribute__ ((weak, alias("Default_Handler")));
void USB_HP_Handler              (void) __attribute__ ((weak, alias("Default_Handler")));
void USB_LP_Handler              (void) __attribute__ ((weak, alias("Default_Handler")));
void USB_WakeUp_RMP_Handler      (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM20_BRK_Handler           (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM20_UP_Handler            (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM20_TRG_COM_Handler       (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM20_CC_Handler            (void) __attribute__ ((weak, alias("Default_Handler")));
void FPU_Handler                 (void) __attribute__ ((weak, alias("Default_Handler")));
void SPI4                        (void) __attribute__ ((weak, alias("Default_Handler")));

// this is visible to the linker script
uint32_t * vectors[]
__attribute__ ((section(".isr_vector")))= {
    (uint32_t *) SRAM_END,                    //  0x00
    (uint32_t *) Reset_Handler,               //  0x04
    (uint32_t *) NMI_Handler,                 //  0x08
    (uint32_t *) HardFault_Handler,           //  0x0C
    (uint32_t *) MemManage_Handler,           //  0x10
    (uint32_t *) BusFault_Handler,            //  0x14
    (uint32_t *) UsageFault_Handler,          //  0x18
    0,                                            //  0x1C
    0,                                            //  0x20
    0,                                            //  0x24
    0,                                            //  0x28
    (uint32_t *) SVCall_Handler,              //  0x2C
    0,                                            //  0x30
    0,                                            //  0x34
    (uint32_t *) PendSV_Handler,              //  0x38
    (uint32_t *) SysTick_Handler,             //  0x3C
    (uint32_t *) WWDG_Handler,                //  0x40
    (uint32_t *) PVD_Handler,                 //  0x44
    (uint32_t *) TAMPER_STAMP_Handler,        //  0x48
    (uint32_t *) RTC_WKUP_Handler,            //  0x4C
    (uint32_t *) FLASH_Handler,               //  0x50
    (uint32_t *) RCC_Handler,                 //  0x54
    (uint32_t *) EXTI0_Handler,               //  0x58
    (uint32_t *) EXTI1_Handler,               //  0x5C
    (uint32_t *) EXTI2_TS_Handler,            //  0x60
    (uint32_t *) EXTI3_Handler,               //  0x64
    (uint32_t *) EXTI4_Handler,               //  0x68
    (uint32_t *) DMA1_Channel1_Handler,       //  0x6C
    (uint32_t *) DMA1_Channel2_Handler,       //  0x70
    (uint32_t *) DMA1_Channel3_Handler,       //  0x74
    (uint32_t *) DMA1_Channel4_Handler,       //  0x78
    (uint32_t *) DMA1_Channel5_Handler,       //  0x7C
    (uint32_t *) DMA1_Channel6_Handler,       //  0x80
    (uint32_t *) DMA1_Channel7_Handler,       //  0x84
    (uint32_t *) ADC1_2_Handler,              //  0x88
    (uint32_t *) USB_HP_CAN_TX_Handler,       //  0x8C
    (uint32_t *) USB_LP_CAN_RX0_Handler,      //  0x90
    (uint32_t *) CAN_RX1_Handler,             //  0x94
    (uint32_t *) CAN_SCE_Handler,             //  0x98
    (uint32_t *) EXTI9_5_Handler,             //  0x9C
    (uint32_t *) TIM1_BRK_TIM15_Handler,      //  0xA0
    (uint32_t *) TIM1_UP_TIM16_Handler,       //  0xA4
    (uint32_t *) TIM1_TRG_COM_TIM17_Handler,  //  0xA8
    (uint32_t *) TIM1_CC_Handler,             //  0xAC
    (uint32_t *) TIM2_Handler,                //  0xB0
    (uint32_t *) TIM3_Handler,                //  0xB4
    (uint32_t *) TIM4_Handler,                //  0xB8
    (uint32_t *) I2C1_EV_Handler,             //  0xBC
    (uint32_t *) I2C1_ER_Handler,             //  0xC0
    (uint32_t *) I2C2_EV_Handler,             //  0xC4
    (uint32_t *) I2C2_ER_Handler,             //  0xC8
    (uint32_t *) SPI1_Handler,                //  0xCC
    (uint32_t *) SPI2_Handler,                //  0xD0
    (uint32_t *) USART1_Handler,              //  0xD4
    (uint32_t *) USART2_Handler,              //  0xD8
    (uint32_t *) USART3_Handler,              //  0xDC
    (uint32_t *) EXTI15_10_Handler,           //  0xE0
    (uint32_t *) RTC_Alarm_Handler,           //  0xE4
    (uint32_t *) USBWakeUP_Handler,           //  0xE8
    (uint32_t *) TIM8_BRK_Handler,            //  0xEC
    (uint32_t *) TIM8_UP_Handler,             //  0xF0
    (uint32_t *) TIM8_TRG_COM_Handler,        //  0xF4
    (uint32_t *) TIM8_CC_Handler,             //  0xF8
    (uint32_t *) ADC3_Handler,                //  0xFC
    (uint32_t *) FMC_Handler,                 // 0x100
    0,                                            // 0x104
    0,                                            // 0x108
    (uint32_t *) SPI3_Handler,                // 0x10C
    (uint32_t *) UART4_Handler,               // 0x110
    (uint32_t *) UART5_Handler,               // 0x114
    (uint32_t *) TIM6_DAC_Handler,            // 0x118
    (uint32_t *) TIM7_Handler,                // 0x11C
    (uint32_t *) DMA2_Channel1_Handler,       // 0x120
    (uint32_t *) DMA2_Channel2_Handler,       // 0x124
    (uint32_t *) DMA2_Channel3_Handler,       // 0x128
    (uint32_t *) DMA2_Channel4_Handler,       // 0x12C
    (uint32_t *) DMA2_Channel5_Handler,       // 0x130
    (uint32_t *) ADC4_Handler,                // 0x134
    0,                                            // 0x138
    0,                                            // 0x13C
    (uint32_t *) COMP_1_2_3_Handler,          // 0x140
    (uint32_t *) COMP_4_5_6_Handler,          // 0x144
    (uint32_t *) COMP7_Handler,               // 0x148
    0,                                            // 0x14C
    0,                                            // 0x150
    0,                                            // 0x154
    0,                                            // 0x158
    0,                                            // 0x15C
    (uint32_t *) I2C3_EV_Handler,             // 0x160
    (uint32_t *) I2C3_ER_Handler,             // 0x164
    (uint32_t *) USB_HP_Handler,              // 0x168
    (uint32_t *) USB_LP_Handler,              // 0x16C
    (uint32_t *) USB_WakeUp_RMP_Handler,      // 0x170
    (uint32_t *) TIM20_BRK_Handler,           // 0x174
    (uint32_t *) TIM20_UP_Handler,            // 0x178
    (uint32_t *) TIM20_TRG_COM_Handler,       // 0x17C
    (uint32_t *) TIM20_CC_Handler,            // 0x180
    (uint32_t *) FPU_Handler,                 // 0x184
    0,                                            // 0x188
    0,                                            // 0x18C
    (uint32_t *) SPI4,                        // 0x190
};

extern uint32_t _etext;
extern uint32_t _ebss;
extern uint32_t _sbss;
extern uint32_t _edata;
extern uint32_t _sdata;
extern uint32_t _sidata;


void Default_Handler()
{
    while(1){}
}

void Reset_Handler(){
    // copy initialized data (.data) from flash to sram
    uint32_t size = (uint32_t) &_edata - (uint32_t) &_sdata;
    uint8_t *pDst = &_sdata;  // sram
    uint8_t *pSrc = &_sidata; // flash
    for (uint32_t i=0; i<size; i++)
    {
        *pDst++ = *pSrc++;
    }

    // zero out uninitialized data (.bss) in sram
    size = (uint32_t) &_ebss - (uint32_t) &_sbss;
    pDst = &_sbss;
    for (uint32_t i=0; i<size; i++)
    {
        *pDst++ = 0;
    }

    // initialize system
    SystemInit();
    // initialize standard c library
    __libc_init_array();
    // run main program
    main();
}