/* main.c
 * I2C1 target (slave) serving 2-byte cell1 voltage (mV) to Raspberry Pi master.
 * Now also prints full BQ readings with timestamp (like your old main.c)
 */

#include "BQ769x2_protocol.h"
#include "I2C_communication.h"
#include "ti_msp_dl_config.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <time.h>

/* --- Utility Functions --- */
static void delayMS(uint32_t ms) { while (ms--) delay_cycles(32000); }

void print_timestamp(void)
{
    time_t now;
    struct tm *timeinfo;

    time(&now);
    now += 8 * 3600; // adjust to UTC+8 (Philippines)
    timeinfo = gmtime(&now);

    printf("[%04d-%02d-%02d %02d:%02d:%02d] ",
           timeinfo->tm_year + 1900,
           timeinfo->tm_mon + 1,
           timeinfo->tm_mday,
           timeinfo->tm_hour,
           timeinfo->tm_min,
           timeinfo->tm_sec);
}

/* --- I2C1 Buffers and Flags --- */
#define I2C1_TX_MAX_PACKET_SIZE (16)
#define I2C1_RX_MAX_PACKET_SIZE (16)
static uint8_t  i2c1TxPacket[I2C1_TX_MAX_PACKET_SIZE];
static volatile uint32_t i2c1TxLen = 2;
static volatile uint32_t i2c1TxCount = 0;
static uint8_t  i2c1RxPacket[I2C1_RX_MAX_PACKET_SIZE];
static volatile uint32_t i2c1RxLen = I2C1_RX_MAX_PACKET_SIZE;
static volatile uint32_t i2c1RxCount = 0;
static volatile bool i2c1_update_requested = false;

/* --- BQ769x2 Helpers --- */
extern void     BQ769x2_Init(void);
extern void     BQ769x2_SleepDisable(void);
extern uint16_t BQ769x2_ReadVoltage(uint8_t command);
extern void     BQ769x2_ReadAllVoltages(void);
extern void     BQ769x2_ReadCurrent(void);
extern void     BQ769x2_ReadAllTemperatures(void);
extern void     BQ769x2_ReadFETStatus(void);

/* Replace or add this helper on the slave */
static void update_tx_from_cells(void)
{
    /* Read two cells (millivolts) from BQ — non-IRQ context only */
    uint16_t cell1_mv = BQ769x2_ReadVoltage(Cell1Voltage);
    uint16_t cell2_mv = BQ769x2_ReadVoltage(Cell2Voltage);

    /* Little-endian layout: [c1_lo, c1_hi, c2_lo, c2_hi] */
    i2c1TxPacket[0] = (uint8_t)(cell1_mv & 0xFF);
    i2c1TxPacket[1] = (uint8_t)((cell1_mv >> 8) & 0xFF);
    i2c1TxPacket[2] = (uint8_t)(cell2_mv & 0xFF);
    i2c1TxPacket[3] = (uint8_t)((cell2_mv >> 8) & 0xFF);

    /* expose 4 bytes to the I2C FIFO/IRQ */
    i2c1TxLen = 4;
    i2c1TxCount = 0;
}


/* --- I2C1 IRQ --- */
void I2C_1_INST_IRQHandler(void)
{
    uint32_t pend = DL_I2C_getPendingInterrupt(I2C_1_INST);

    switch (pend)
    {
        case DL_I2C_IIDX_TARGET_START:
            i2c1TxCount = 0;
            i2c1RxCount = 0;
            DL_I2C_flushTargetTXFIFO(I2C_1_INST);
            break;

        case DL_I2C_IIDX_TARGET_RXFIFO_TRIGGER:
            while (DL_I2C_isTargetRXFIFOEmpty(I2C_1_INST) != true)
            {
                if (i2c1RxCount < i2c1RxLen)
                    i2c1RxPacket[i2c1RxCount++] = DL_I2C_receiveTargetData(I2C_1_INST);
                else
                    (void)DL_I2C_receiveTargetData(I2C_1_INST);
            }
            if (i2c1RxCount >= 1)
                i2c1_update_requested = true;
            break;

        case DL_I2C_IIDX_TARGET_TXFIFO_TRIGGER:
            if (i2c1TxCount < i2c1TxLen)
            {
                i2c1TxCount += DL_I2C_fillTargetTXFIFO(
                    I2C_1_INST,
                    &i2c1TxPacket[i2c1TxCount],
                    (i2c1TxLen - i2c1TxCount));
            }
            else
            {
                while (DL_I2C_transmitTargetDataCheck(I2C_1_INST, 0x00) != false);
            }
            break;

        case DL_I2C_IIDX_TARGET_STOP:
            DL_GPIO_togglePins(GPIO_LEDS_PORT, GPIO_LEDS_USER_LED_1_PIN);
            break;

        default:
            break;
    }
}

/* --- MAIN --- */
int main(void)
{
    SYSCFG_DL_init();

    DL_GPIO_setPins(GPIO_LEDS_PORT, GPIO_LEDS_USER_LED_1_PIN);

    BQ769x2_Init();
    delayMS(10);

    DL_I2C_enableInterrupt(
        I2C_1_INST,
        DL_I2C_INTERRUPT_TARGET_TXFIFO_TRIGGER |
        DL_I2C_INTERRUPT_TARGET_RXFIFO_TRIGGER |
        DL_I2C_INTERRUPT_TARGET_STOP |
        DL_I2C_INTERRUPT_TARGET_START);

#ifdef DL_I2C_setTargetAddress
    DL_I2C_setTargetAddress(I2C_1_INST, I2C_1_TARGET_OWN_ADDR);
#elif defined(DL_I2C_setTargetOwnAddress)
    DL_I2C_setTargetOwnAddress(I2C_1_INST, I2C_1_TARGET_OWN_ADDR);
#endif

    NVIC_EnableIRQ(I2C_1_INST_INT_IRQN);
    NVIC_EnableIRQ(GPIO_GRP_0_INT_IRQN);

    BQ769x2_SleepDisable();
    update_tx_from_cells();
    if (i2c1TxLen > 0) {
        i2c1TxCount += DL_I2C_fillTargetTXFIFO(I2C_1_INST, &i2c1TxPacket[0], i2c1TxLen);
    }

    while (1)
    {
                /* In the loop when an update is requested */
        if (i2c1_update_requested) {
            i2c1_update_requested = false;
            BQ769x2_SleepDisable();
            update_tx_from_cells();

            if (i2c1TxLen > 0) {
                i2c1TxCount = 0;
                i2c1TxCount += DL_I2C_fillTargetTXFIFO(I2C_1_INST, &i2c1TxPacket[0], i2c1TxLen);
            }
        }
        /* === Periodic BQ Readout + Timestamp Print === */
        BQ769x2_SleepDisable();
        print_timestamp();
        printf("ALL Voltages Reading:\n");
        BQ769x2_ReadAllVoltages();
        delayMS(10);

        print_timestamp();
        printf("Current Reading:\n");
        BQ769x2_ReadCurrent();
        delayMS(10);

        print_timestamp();
        printf("All Temperature Reading:\n");
        BQ769x2_ReadAllTemperatures();
        delayMS(10);

        print_timestamp();
        printf("FET Status:\n");
        BQ769x2_ReadFETStatus();

        /* Also print quick I2C1 cell1 data */
        uint16_t v1 = (uint16_t)i2c1TxPacket[0] | ((uint16_t)i2c1TxPacket[1] << 8);
        print_timestamp();
        printf("[I2C1-SLAVE] Cell1 = %u mV\n", (unsigned int)v1);
         /* Also print quick I2C1 cell1 data */
        uint16_t v2 = (uint16_t)i2c1TxPacket[2] | ((uint16_t)i2c1TxPacket[3] << 8);
        print_timestamp();
        printf("[I2C1-SLAVE] Cell2 = %u mV\n", (unsigned int)v2);

        delayMS(500);
    }

    return 0;
}
