#include "ti_msp_dl_config.h"

/* Maximum size of TX packet */
#define I2C_TX_MAX_PACKET_SIZE (16)

/* Maximum size of RX packet */
#define I2C_RX_MAX_PACKET_SIZE (16)

/* Data sent to Controller in response to Read transfer ("Hello") */
uint8_t gTxPacket[I2C_TX_MAX_PACKET_SIZE] = {
    'H','e','l','l','o', '1','2', '3',
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
};

/* Counters for TX length and bytes sent */
uint32_t gTxLen, gTxCount;

/* Data received from Controller during a Write transfer */
uint8_t gRxPacket[I2C_RX_MAX_PACKET_SIZE];
/* Counters for TX length and bytes sent */
uint32_t gRxLen, gRxCount;

int main(void)
{
    SYSCFG_DL_init();

    /* Set LED to indicate start of transfer */
    DL_GPIO_setPins(GPIO_LEDS_PORT, GPIO_LEDS_USER_LED_1_PIN);

    /*
     * Fill FIFO with data.
     * Transactions are initiated by the Controller, so this example
     * only fills the buffer — the Target will send this data when
     * requested by the Controller.
     */
    gTxCount = 0;
    gTxLen   = I2C_TX_MAX_PACKET_SIZE;

    /* IMPORTANT: enable other target interrupts so START/STOP/RX work too */
    DL_I2C_enableInterrupt(I2C_INST,
        DL_I2C_INTERRUPT_TARGET_TXFIFO_TRIGGER |
        DL_I2C_INTERRUPT_TARGET_RXFIFO_TRIGGER |
        DL_I2C_INTERRUPT_TARGET_STOP |
        DL_I2C_INTERRUPT_TARGET_START);

    /* Initialize variables to receive data inside RX ISR */
    gRxCount = 0;
    gRxLen   = I2C_RX_MAX_PACKET_SIZE;

    NVIC_EnableIRQ(I2C_INST_INT_IRQN);

    /* Keep default sleep-on-exit for low-power operation; ok for example */
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
            /* Store received data in buffer */
            dataRx = true;
            while (DL_I2C_isTargetRXFIFOEmpty(I2C_INST) != true) {
                if (gRxCount < gRxLen) {
                    gRxPacket[gRxCount++] = DL_I2C_receiveTargetData(I2C_INST);
                } else {
                    /* Prevent overflow and just ignore data */
                    DL_I2C_receiveTargetData(I2C_INST);
                }
            }
            break;
        case DL_I2C_IIDX_TARGET_TXFIFO_TRIGGER:
            /* Fill TX FIFO if there are more bytes to send */
            if (gTxCount < gTxLen) {
                gTxCount += DL_I2C_fillTargetTXFIFO(
                    I2C_INST, &gTxPacket[gTxCount], (gTxLen - gTxCount));
            } else {
                /* If master reads more than gTxLen, send 0x00 */
                while (DL_I2C_transmitTargetDataCheck(I2C_INST, 0x00) != false)
                    ;
            }
            break;
        case DL_I2C_IIDX_TARGET_STOP:
            /* If data was received, discard or process it as needed without
            copying into gTxPacket (so gTxPacket stays "Hello"). */
            if (dataRx == true) {
                // Optionally process gRxPacket here (store somewhere else)
                // For now we just clear the flag and ignore echo:
                dataRx = false;
            }
            /* Toggle LED to indicate successful RX or TX */
            DL_GPIO_togglePins(GPIO_LEDS_PORT, GPIO_LEDS_USER_LED_1_PIN);
            break;

        default:
            break;
    }
}
