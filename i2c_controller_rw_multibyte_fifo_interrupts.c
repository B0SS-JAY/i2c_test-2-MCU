/* Master: continuously send "Hi" then read 8 bytes (expect "Hello123") and print.
   Assumes stdio is routed (semihosting or UART). Keep SDA↔SDA, SCL↔SCL, GND↔GND.
*/
#include "ti_msp_dl_config.h"
#include <stdio.h>

/* Maximum size of TX packet */
#define I2C_TX_MAX_PACKET_SIZE (16)

/* Number of bytes to send to target device */
#define I2C_TX_PACKET_SIZE (2)    /* send "Hi" */

/* Maximum size of RX packet */
#define I2C_RX_MAX_PACKET_SIZE (16)

/* Number of bytes to received from target */
#define I2C_RX_PACKET_SIZE (8)    /* expect "Hello123" */

/* I2C Target address */
#define I2C_TARGET_ADDRESS (0x48)

/* Data sent to the Target ("Hi") */
uint8_t gTxPacket[I2C_TX_MAX_PACKET_SIZE] = {
    'H','i', 0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
};
/* Counters for TX length and bytes sent */
uint32_t gTxLen, gTxCount;

/* Data received from Target */
uint8_t gRxPacket[I2C_RX_MAX_PACKET_SIZE];
/* Counters for RX length and bytes received */
uint32_t gRxLen, gRxCount;

/* Indicates status of I2C */
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

int main(void)
{
    SYSCFG_DL_init();

    printf("Master: starting continuous Hi->Hello123 loop\n");

    /* Set LED to indicate start */
    DL_GPIO_setPins(GPIO_LEDS_PORT, GPIO_LEDS_USER_LED_1_PIN);

    NVIC_EnableIRQ(I2C_INST_INT_IRQN);
    DL_SYSCTL_disableSleepOnExit();

    gI2cControllerStatus = I2C_STATUS_IDLE;

    /* main loop: send "Hi", then read "Hello123", print, repeat */
    while (1) {
        /* --- PREPARE & SEND "Hi" --- */
        gTxLen = I2C_TX_PACKET_SIZE;
        gTxCount = DL_I2C_fillControllerTXFIFO(I2C_INST, &gTxPacket[0], gTxLen);

        if (gTxCount < gTxLen) {
            DL_I2C_enableInterrupt(I2C_INST, DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER);
        } else {
            DL_I2C_disableInterrupt(I2C_INST, DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER);
        }

        gI2cControllerStatus = I2C_STATUS_TX_STARTED;

        while (!(DL_I2C_getControllerStatus(I2C_INST) & DL_I2C_CONTROLLER_STATUS_IDLE))
            ;
        DL_I2C_startControllerTransfer(I2C_INST, I2C_TARGET_ADDRESS, DL_I2C_CONTROLLER_DIRECTION_TX, gTxLen);

        /* Wait until TX finishes or error */
        while ((gI2cControllerStatus != I2C_STATUS_TX_COMPLETE) &&
               (gI2cControllerStatus != I2C_STATUS_ERROR)) {
            __WFE();
        }

        if (gI2cControllerStatus == I2C_STATUS_ERROR) {
            printf("Master: TX error\n");
            /* small retry delay */
            for (volatile int i = 0; i < 200000; i++);
            continue;
        }

        /* Ensure bus idle */
        while (DL_I2C_getControllerStatus(I2C_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS)
            ;

        /* small gap between write and read */
        delay_cycles(1000);

        /* --- REQUEST & RECEIVE (exact 8 bytes for "Hello123") --- */
        gRxLen = I2C_RX_PACKET_SIZE;   // 8
        gRxCount = 0;                  // reset counter BEFORE starting RX
        gI2cControllerStatus = I2C_STATUS_RX_STARTED;

        DL_I2C_startControllerTransfer(I2C_INST, I2C_TARGET_ADDRESS, DL_I2C_CONTROLLER_DIRECTION_RX, gRxLen);

        /* Wait for RX complete or error */
        while ((gI2cControllerStatus != I2C_STATUS_RX_COMPLETE) &&
               (gI2cControllerStatus != I2C_STATUS_ERROR)) {
            __WFE();
        }

        if (gI2cControllerStatus == I2C_STATUS_ERROR) {
            printf("Master: RX error\n");
            for (volatile int i = 0; i < 200000; i++);
            continue;
        }

        /* Make sure bus is idle before using data */
        while (DL_I2C_getControllerStatus(I2C_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS)
            ;

        /* Null-terminate exactly at 8 bytes and print */
        gRxPacket[8] = '\0';
        printf("Master received (%u bytes): %s\n", (unsigned)gRxCount, (char*)gRxPacket);

        /* visual heartbeat */
        DL_GPIO_togglePins(GPIO_LEDS_PORT, GPIO_LEDS_USER_LED_1_PIN | GPIO_LEDS_USER_TEST_PIN);

        /* Pause so output is human-readable (adjust as needed) */
        for (volatile int i = 0; i < 500000; i++);
    }
}

/* IRQ handler (keeps your original behavior — updates gRxCount/gTxCount and statuses) */
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
        default:
            break;
    }
}
