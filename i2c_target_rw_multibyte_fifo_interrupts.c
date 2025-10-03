/* Slave: always respond with "Hello123" (8 bytes) on master's read.
   Incoming writes from master are received but do not overwrite the response.
*/
#include "ti_msp_dl_config.h"

/* Maximum size of TX packet */
#define I2C_TX_MAX_PACKET_SIZE (16)

/* Maximum size of RX packet */
#define I2C_RX_MAX_PACKET_SIZE (16)

/* Data sent to Controller in response to Read transfer ("Hello123") */
uint8_t gTxPacket[I2C_TX_MAX_PACKET_SIZE] = {
    'H','e','l','l','o','1','2','3',
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
};

/* Counters for TX length and bytes sent */
uint32_t gTxLen, gTxCount;

/* Data received from Controller during a Write transfer */
uint8_t gRxPacket[I2C_RX_MAX_PACKET_SIZE];
/* Counters for RX length and bytes sent */
uint32_t gRxLen, gRxCount;

int main(void)
{
    SYSCFG_DL_init();

    /* Set LED to indicate start of transfer */
    DL_GPIO_setPins(GPIO_LEDS_PORT, GPIO_LEDS_USER_LED_1_PIN);

    gTxCount = 0;
    gTxLen   = I2C_TX_MAX_PACKET_SIZE;

    /* Enable necessary target interrupts (START/STOP/RX/TX triggers) */
    DL_I2C_enableInterrupt(I2C_INST,
        DL_I2C_INTERRUPT_TARGET_TXFIFO_TRIGGER |
        DL_I2C_INTERRUPT_TARGET_RXFIFO_TRIGGER |
        DL_I2C_INTERRUPT_TARGET_STOP |
        DL_I2C_INTERRUPT_TARGET_START);

    gRxCount = 0;
    gRxLen   = I2C_RX_MAX_PACKET_SIZE;

    NVIC_EnableIRQ(I2C_INST_INT_IRQN);

    /* Keep low-power sleep-on-exit behavior */
    DL_SYSCTL_enableSleepOnExit();

    while (1) {
        __WFI();
    }
}

void I2C_INST_IRQHandler(void)
{
    static bool dataRx = false;

    switch (DL_I2C_getPendingInterrupt(I2C_INST)) {
        case DL_I2C_IIDX_TARGET_START:
            /* Prepare for new transfer */
            gTxCount = 0;
            gRxCount = 0;
            dataRx   = false;
            DL_I2C_flushTargetTXFIFO(I2C_INST);
            break;
        case DL_I2C_IIDX_TARGET_RXFIFO_TRIGGER:
            /* Store received data in buffer (do not overwrite gTxPacket) */
            dataRx = true;
            while (DL_I2C_isTargetRXFIFOEmpty(I2C_INST) != true) {
                if (gRxCount < gRxLen) {
                    gRxPacket[gRxCount++] = DL_I2C_receiveTargetData(I2C_INST);
                } else {
                    DL_I2C_receiveTargetData(I2C_INST);
                }
            }
            break;
        case DL_I2C_IIDX_TARGET_TXFIFO_TRIGGER:
            /* Fill TX FIFO with the fixed response */
            if (gTxCount < gTxLen) {
                gTxCount += DL_I2C_fillTargetTXFIFO(I2C_INST, &gTxPacket[gTxCount], (gTxLen - gTxCount));
            } else {
                while (DL_I2C_transmitTargetDataCheck(I2C_INST, 0x00) != false)
                    ;
            }
            break;
        case DL_I2C_IIDX_TARGET_STOP:
            /* Do not overwrite gTxPacket — keep "Hello123" intact.
               Clear dataRx flag so next transfer starts clean. */
            if (dataRx == true) {
                dataRx = false;
            }
            DL_GPIO_togglePins(GPIO_LEDS_PORT, GPIO_LEDS_USER_LED_1_PIN);
            break;
        default:
            break;
    }
}
