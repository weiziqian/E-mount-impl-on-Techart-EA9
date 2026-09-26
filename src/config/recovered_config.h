/* recovered_config.h -- the LM-EA9's driver configuration, read out of the stock
 * image rather than guessed.  Derivation and evidence: rebuild/NOTES.md §13.
 *
 * Everything here is a fact about firmware 1.8.0, recovered from its own
 * register writes and const tables.  Values carry the image address they came
 * from so they can be re-checked.
 */
#ifndef RECOVERED_CONFIG_H
#define RECOVERED_CONFIG_H

/* ---- clock tree -------------------------------------------------- 0x81b8, 0x770c */
#define CONF_GCLK_GEN0_SRC          7          /* DFLL48M, open loop  (GENCTRL 0x00010700) */
#define CONF_GCLK_GEN0_FREQUENCY    48000000
#define CONF_GCLK_GEN1_SRC          4          /* OSC32K              (GENCTRL 0x00010401) */
#define CONF_GCLK_GEN1_FREQUENCY    32768
#define CONF_GCLK_GEN2_SRC          6          /* OSC8M               (GENCTRL 0x00010602) */
#define CONF_GCLK_GEN2_FREQUENCY    8000000
/* DFLL48M runs OPEN LOOP: DFLLCTRL = ENABLE only, DFLLMUL = 0x04010000 (MUL = 0),
 * DFLLVAL takes COARSE from the factory calibration word at 0x00806024 with the
 * standard 0x3f -> 0x1f fallback, FINE = 512. */

/* ---- E-mount bus: SERCOM1 as USART ------------------- 0x6474, 0x6410, _usarts[0] @0x9920 */
#define CONF_EMOUNT_SERCOM          1
#define CONF_EMOUNT_SERCOM_ADDR     0x42000C00
#define CONF_EMOUNT_PM_APBCMASK_BIT 3
#define CONF_EMOUNT_GCLK_ID         21         /* GCLK_SERCOM1_CORE, generator 0 (CLKCTRL 0x4015) */
#define CONF_EMOUNT_CTRLA           0x40100004 /* MODE=1 int. clock, RXPO=PAD1, TXPO=PAD0,
                                                  DORD=1 LSB-first, SAMPR=0 16x arithmetic */
#define CONF_EMOUNT_CTRLB           0x00030000 /* RXEN|TXEN, 8 bits, 1 stop, no parity */
#define CONF_EMOUNT_BAUD            0xC000     /* 48 MHz, 16x -> 750000 baud EXACTLY */
#define CONF_EMOUNT_BAUDRATE        750000
#define CONF_EMOUNT_RX_RING_BYTES   16         /* buffer at 0x20000538 */

/* ---- motor PWM: TCC0 ------------------------------------------- 0x6558, 0x6538 */
#define CONF_PWM_TCC                0
#define CONF_PWM_TCC_ADDR           0x42002000
#define CONF_PWM_PM_APBCMASK_BIT    8
#define CONF_PWM_GCLK_ID            26         /* GCLK_TCC0_TCC1, generator 0 (CLKCTRL 0x401a) */

/* ---- timers ---------------------------------------------- 0x65ac/0x658c, 0x6208 */
#define CONF_TIMER1_TCC             1          /* TCC1 driven by hal_timer, started from main */
#define CONF_TIMER1_TCC_ADDR        0x42002400
#define CONF_TIMER1_PM_APBCMASK_BIT 9
#define CONF_TIMER1_GCLK_ID         26         /* shares the TCC0/TCC1 channel */
#define CONF_TIMER0_TC              4
#define CONF_TIMER0_TC_ADDR         0x42003000
#define CONF_TIMER0_PM_APBCMASK_BIT 12
#define CONF_TIMER0_GCLK_ID         28         /* GCLK_TC4_TC5, generator 0 (CLKCTRL 0x401c) */

/* ---- SPI: SERCOM0 ------------------------------------------------------ 0x63e8 */
#define CONF_SPI_SERCOM             0
#define CONF_SPI_SERCOM_ADDR        0x42000800

/* ---- NVIC priorities ---------------------------------------------------- 0x8b20
 * Only the top two bits are implemented on Cortex-M0+, so these are 0..3. */
#define CONF_IRQ_PRIO_EIC           0x40       /* IRQ 4  -> 1 */
#define CONF_IRQ_PRIO_NVMCTRL       0x00       /* IRQ 5  -> 0 */
#define CONF_IRQ_PRIO_DMAC          0x00       /* IRQ 6  -> 0 */
#define CONF_IRQ_PRIO_TCC1          0xC0       /* IRQ 16 -> 3 */
#define CONF_IRQ_PRIO_TC4           0x80       /* IRQ 19 -> 2 */
/* SERCOM1 (IRQ 10) and TCC0 (IRQ 15) are left at the reset default, 0. */

/* ---- pin map -- COMPLETE.  Mux letters verified against include/pio/samd21e17a.h.
 *
 *  pin   mux  function                            set by
 *  PA00   D   SERCOM1/PAD[0]  E-mount UART TX     0x6430   (CTRLA TXPO=0)
 *  PA01   D   SERCOM1/PAD[1]  E-mount UART RX     0x6430   (CTRLA RXPO=1)
 *  PA02   A   EIC/EXTINT[2]   input, no pull      0x6248
 *  PA03   A   EIC/EXTINT[3]   input, no pull      0x6248
 *  PA04   D   SERCOM0/PAD[0]  SPI                 0x6310
 *  PA05   D   SERCOM0/PAD[1]  SPI                 0x6310
 *  PA06   D   SERCOM0/PAD[2]  SPI, INEN           0x6310
 *  PA07   -   GPIO                                0x65d8
 *  PA08   E   TCC0/WO[0]      motor PWM           0x64c0
 *  PA09   E   TCC0/WO[1]      motor PWM           0x64c0
 *  PA10   -   GPIO                                0x65d8
 *  PA11   -   GPIO, driven HIGH at init           0x65d8
 *  PA16   -   GPIO, driven HIGH by main           0x65d8
 *  PA18   F   TCC0/WO[2]      motor PWM           0x64c0
 *  PA19   F   TCC0/WO[3]      motor PWM           0x64c0
 *  PA23   -   GPIO OUTPUT driven LOW              0x65d8
 *             -- the pin the bootloader tests to choose app vs USB (EA9.md 11.4)
 *
 * Four TCC0 outputs plus two EIC inputs is the shape of a bipolar stepper on two
 * H-bridges with quadrature (or dual-hall) feedback.
 */
#define CONF_PIN_EMOUNT_TX          0          /* PA00, mux D */
#define CONF_PIN_EMOUNT_RX          1          /* PA01, mux D */
#define CONF_PIN_ENCODER_A          2          /* PA02, mux A, EXTINT[2] */
#define CONF_PIN_ENCODER_B          3          /* PA03, mux A, EXTINT[3] */
#define CONF_PIN_SPI_PAD0           4          /* PA04, mux D */
#define CONF_PIN_SPI_PAD1           5          /* PA05, mux D */
#define CONF_PIN_SPI_PAD2           6          /* PA06, mux D */
#define CONF_PIN_MOTOR_WO0          8          /* PA08, mux E */
#define CONF_PIN_MOTOR_WO1          9          /* PA09, mux E */
#define CONF_PIN_MOTOR_WO2          18         /* PA18, mux F */
#define CONF_PIN_MOTOR_WO3          19         /* PA19, mux F */
#define CONF_PIN_GPIO_PA07          7
#define CONF_PIN_GPIO_PA10          10
#define CONF_PIN_GPIO_PA11          11         /* driven HIGH at init */
#define CONF_PIN_GPIO_PA16          16         /* driven HIGH by main */
#define CONF_PIN_BOOTMODE_PA23      23         /* OUTPUT, driven LOW */

/* ---- EIC ------------------------------------------------------------- 0x6248 */
#define CONF_EIC_GCLK_ID            5          /* GCLK_EIC, generator 0 (CLKCTRL 0x4005) */

/* ---- flash (the settings page) --------------------------------------- 0x62ec */
#define CONF_FLASH_DESCRIPTOR       0x2000087C /* same descriptor the settings writer uses */

#endif /* RECOVERED_CONFIG_H */
