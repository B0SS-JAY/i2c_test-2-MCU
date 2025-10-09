/* Master: automatically recover when I2C wires are removed/reinserted.
   Writes 1 request byte then reads 2 bytes (cell1 millivolts) and prints.
   Assumes stdio is routed (semihosting or UART). Keep SDA↔SDA, SCL↔SCL, GND↔GND.
*/
#include "ti_msp_dl_config.h"
#include <stdio.h>
#include <stdint.h>

/* Settings */
#define I2C_TX_MAX_PACKET_SIZE (16)
#define I2C_TX_PACKET_SIZE     (1)   /* request byte */
#define I2C_RX_MAX_PACKET_SIZE (16)
#define I2C_RX_PACKET_SIZE     (4)   /* 2 bytes = uint16 mV */
#define I2C_TARGET_ADDRESS     (0x48) /* change if your slave uses a different 7-bit address */

/* Buffers */
uint8_t gTxPacket[I2C_TX_MAX_PACKET_SIZE] = {
    0x01, /* request byte */
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
};
uint8_t gRxPacket[I2C_RX_MAX_PACKET_SIZE];

/* Counters & state */
uint32_t gTxLen, gTxCount;
uint32_t gRxLen, gRxCount;

enum I2cControllerStatus {
    I2C_STATUS_IDLE = 0,
    I2C_STATUS_TX_STARTED,
    I2C_STATUS_TX_INPROGRESS,
    I2C_STATUS_TX_COMPLETE,
    I2C_STATUS_RX_STARTED,
    I2C_STATUS_RX_INPROGRESS,
    I2C_STATUS_RX_COMPLETE,
    I2C_STATUS_ERROR,
} gI2cControllerStatus;

/* Helpers */
static void short_delay(void)
{
    for (volatile int i = 0; i < 200000; i++);
}

/* Recovery: reinitialize system (clears I2C hardware state) */
static void recover_i2c(void)
{
    /* Disable I2C IRQs to avoid races while reinit */
    DL_I2C_disableInterrupt(I2C_INST, DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER);
    DL_I2C_disableInterrupt(I2C_INST, DL_I2C_INTERRUPT_CONTROLLER_RXFIFO_TRIGGER);
    NVIC_DisableIRQ(I2C_INST_INT_IRQN);

    short_delay();

    /* Re-run board/system init to reset peripheral state */
    SYSCFG_DL_init();

    /* Re-enable IRQ & required interrupts (RX trigger always useful) */
    NVIC_EnableIRQ(I2C_INST_INT_IRQN);
    DL_I2C_enableInterrupt(I2C_INST, DL_I2C_INTERRUPT_CONTROLLER_RXFIFO_TRIGGER);
    /* TXFIFO trigger will be enabled conditionally when needed by code */

    /* Reset local counters/status */
    gTxCount = 0;
    gRxCount = 0;
    gI2cControllerStatus = I2C_STATUS_IDLE;

    short_delay();
}

/* Attempt TX with retries and heavier recovery on persistent failures */
static int attempt_tx_transfer_with_recovery(void)
{
    const int MAX_IMMEDIATE_RETRIES = 3;
    const int MAX_RECOVERY_ATTEMPTS = 5;
    int immediate_retry = 0;
    int recovery_attempts = 0;

    while (1) {
        /* Prepare FIFO: only 1 byte (request) */
        gTxLen = I2C_TX_PACKET_SIZE;
        gTxCount = DL_I2C_fillControllerTXFIFO(I2C_INST, &gTxPacket[0], gTxLen);

        if (gTxCount < gTxLen) {
            DL_I2C_enableInterrupt(I2C_INST, DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER);
        } else {
            DL_I2C_disableInterrupt(I2C_INST, DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER);
        }

        /* Start TX */
        gI2cControllerStatus = I2C_STATUS_TX_STARTED;
        while (!(DL_I2C_getControllerStatus(I2C_INST) & DL_I2C_CONTROLLER_STATUS_IDLE))
            ;
        DL_I2C_startControllerTransfer(I2C_INST, I2C_TARGET_ADDRESS,
                                       DL_I2C_CONTROLLER_DIRECTION_TX, gTxLen);

        /* Wait until TX completes or error */
        while ((gI2cControllerStatus != I2C_STATUS_TX_COMPLETE) &&
               (gI2cControllerStatus != I2C_STATUS_ERROR)) {
            __WFE();
        }

        if (gI2cControllerStatus == I2C_STATUS_TX_COMPLETE) {
            return 0; /* success */
        }

        /* Error path */
        immediate_retry++;
        printf("Master: TX error (immediate try %d)\n", immediate_retry);

        /* Quick immediate retries */
        if (immediate_retry < MAX_IMMEDIATE_RETRIES) {
            gTxCount = 0;
            gRxCount = 0;
            gI2cControllerStatus = I2C_STATUS_IDLE;
            short_delay();
            continue;
        }

        /* Heavy recovery attempts */
        if (recovery_attempts < MAX_RECOVERY_ATTEMPTS) {
            recovery_attempts++;
            printf("Master: performing I2C reinit recovery attempt %d\n", recovery_attempts);
            recover_i2c();
            immediate_retry = 0;
            continue;
        }

        /* If still failing, wait longer and try again */
        printf("Master: persistent TX failure — waiting before next attempts\n");
        for (volatile int i = 0; i < 1000000; i++);
        /* loop and try again */
    }
}

/* Attempt RX with retries and heavier recovery on persistent failures */
static int attempt_rx_transfer_with_recovery(void)
{
    const int MAX_IMMEDIATE_RETRIES = 3;
    const int MAX_RECOVERY_ATTEMPTS = 5;
    int immediate_retry = 0;
    int recovery_attempts = 0;

    while (1) {
        gRxLen = I2C_RX_PACKET_SIZE;
        gRxCount = 0;
        gI2cControllerStatus = I2C_STATUS_RX_STARTED;

        DL_I2C_startControllerTransfer(I2C_INST, I2C_TARGET_ADDRESS,
                                       DL_I2C_CONTROLLER_DIRECTION_RX, gRxLen);

        while ((gI2cControllerStatus != I2C_STATUS_RX_COMPLETE) &&
               (gI2cControllerStatus != I2C_STATUS_ERROR)) {
            __WFE();
        }

        if (gI2cControllerStatus == I2C_STATUS_RX_COMPLETE) {
            return 0; /* success */
        }

        /* Error path */
        immediate_retry++;
        printf("Master: RX error (immediate try %d)\n", immediate_retry);

        if (immediate_retry < MAX_IMMEDIATE_RETRIES) {
            gTxCount = 0;
            gRxCount = 0;
            gI2cControllerStatus = I2C_STATUS_IDLE;
            short_delay();
            continue;
        }

        if (recovery_attempts < MAX_RECOVERY_ATTEMPTS) {
            recovery_attempts++;
            printf("Master: performing I2C reinit recovery attempt %d\n", recovery_attempts);
            recover_i2c();
            immediate_retry = 0;
            continue;
        }

        printf("Master: persistent RX failure — waiting before next attempts\n");
        for (volatile int i = 0; i < 1000000; i++);
    }
}

int main(void)
{
    SYSCFG_DL_init();
    printf("Master: request->read cell1 loop starting (slave addr=0x%02X)\n", (unsigned)I2C_TARGET_ADDRESS);

    /* Visual LED indicator */
    DL_GPIO_setPins(GPIO_LEDS_PORT, GPIO_LEDS_USER_LED_1_PIN);

    NVIC_EnableIRQ(I2C_INST_INT_IRQN);
    DL_SYSCTL_disableSleepOnExit();

    gI2cControllerStatus = I2C_STATUS_IDLE;

    while (1) {
        /* Attempt TX (internal retries + recovery) */
        if (attempt_tx_transfer_with_recovery() != 0) {
            /* unreachable in this structure but keep loop robust */
            continue;
        }

        /* Small pause between write and read - allow slave to update */
        delay_cycles(1000); /* adjust if needed */

        /* Attempt RX (internal retries + recovery) */
        if (attempt_rx_transfer_with_recovery() != 0) {
            continue;
        }

        /* On success: parse 2 bytes as little-endian uint16 (millivolts) */
        /* On success: parse 4 bytes as two little-endian uint16 (millivolts) */
        if (gRxCount >= 4) {
            uint16_t mv1 = (uint16_t)gRxPacket[0] | ((uint16_t)gRxPacket[1] << 8);
            uint16_t mv2 = (uint16_t)gRxPacket[2] | ((uint16_t)gRxPacket[3] << 8);
            float v1 = ((float)mv1) / 1000.0f;
            float v2 = ((float)mv2) / 1000.0f;
            printf("Master received: Cell1 = %u mV (%.3f V), Cell2 = %u mV (%.3f V)\n",
                (unsigned)mv1, v1, (unsigned)mv2, v2);
        } else {
            printf("Master: received %u bytes (expected 4)\n", (unsigned)gRxCount);
        }


        /* Heartbeat toggle */
        DL_GPIO_togglePins(GPIO_LEDS_PORT, GPIO_LEDS_USER_LED_1_PIN | GPIO_LEDS_USER_TEST_PIN);

        /* Delay so console readable */
        for (volatile int i = 0; i < 500000; i++);
    }
}

/* I2C IRQ handler (keeps original behavior: update counts/status and read/write FIFOs) */
void I2C_INST_IRQHandler(void)
{
    switch (DL_I2C_getPendingInterrupt(I2C_INST)) {
        case DL_I2C_IIDX_CONTROLLER_RX_DONE:
            gI2cControllerStatus = I2C_STATUS_RX_COMPLETE;
            break;
        case DL_I2C_IIDX_CONTROLLER_TX_DONE:
            DL_I2C_disableInterrupt(I2C_INST, DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER);
            gI2cControllerStatus = I2C_STATUS_TX_COMPLETE;
            break;
        case DL_I2C_IIDX_CONTROLLER_RXFIFO_TRIGGER:
            gI2cControllerStatus = I2C_STATUS_RX_INPROGRESS;
            while (DL_I2C_isControllerRXFIFOEmpty(I2C_INST) != true) {
                if (gRxCount < gRxLen) {
                    gRxPacket[gRxCount++] = DL_I2C_receiveControllerData(I2C_INST);
                } else {
                    DL_I2C_receiveControllerData(I2C_INST);
                }
            }
            break;
        case DL_I2C_IIDX_CONTROLLER_TXFIFO_TRIGGER:
            gI2cControllerStatus = I2C_STATUS_TX_INPROGRESS;
            if (gTxCount < gTxLen) {
                gTxCount += DL_I2C_fillControllerTXFIFO(I2C_INST, &gTxPacket[gTxCount], gTxLen - gTxCount);
            }
            break;
        case DL_I2C_IIDX_CONTROLLER_ARBITRATION_LOST:
        case DL_I2C_IIDX_CONTROLLER_NACK:
            if ((gI2cControllerStatus == I2C_STATUS_RX_STARTED) || (gI2cControllerStatus == I2C_STATUS_TX_STARTED)) {
                gI2cControllerStatus = I2C_STATUS_ERROR;
            }
            break;
        default:
            break;
    }
}
