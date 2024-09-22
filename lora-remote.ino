#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "HT_SSD1306Wire.h"
#include "esp_sleep.h"
#include "esp_attr.h"

#define RF_FREQUENCY 915000000 // Hz
#define TX_OUTPUT_POWER 20     // dBm (Maximum power)
#define RETRY_INTERVAL 10000   // 10 seconds
#define MAX_RETRY_DURATION 60000 // 1 minute

#define LORA_BANDWIDTH                              0         // [0: 125 kHz]
#define LORA_SPREADING_FACTOR                       7         // [SF7..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false
#define LORA_SYNC_WORD                              0xF3      // Custom sync word

#define RX_TIMEOUT_VALUE                            5000      // 5 seconds timeout for receiving ACK
#define BUFFER_SIZE                                 50        // Define the payload size here

#define LED_PIN                                     25        // Built-in LED on Heltec LoRa V3 board

char txpacket[50];
bool lora_idle = true;
bool ack_received = false;
unsigned long startRetryTime;
int randomNum;

static SSD1306Wire display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);

static RadioEvents_t RadioEvents;
void OnTxDone(void);
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);
void OnTxTimeout(void);
void OnRxTimeout(void);
void OnRxError(void);

void setup() {
    Serial.begin(115200);
    Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);

    // Display setup
    VextON();
    delay(100);
    display.init();
    display.setFont(ArialMT_Plain_16);
    display.clear();
    display.setTextAlignment(TEXT_ALIGN_CENTER);
    display.drawString(display.getWidth() / 2, (display.getHeight() - 16) / 2, "Ola...");
    display.display();

    // Initialize Radio
    RadioEvents.TxDone = OnTxDone;
    RadioEvents.RxDone = OnRxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxTimeout = OnRxTimeout;
    RadioEvents.RxError = OnRxError;
    Radio.Init(&RadioEvents);
    Radio.SetChannel(RF_FREQUENCY);
    Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                      LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                      LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                      true, 0, 0, LORA_IQ_INVERSION_ON, RX_TIMEOUT_VALUE);

    Radio.SetSyncWord(LORA_SYNC_WORD); // Custom sync word

    sendPacket();
}

void goToSleep() {
    // Add delay to let the OLED show the final message before going to sleep
    delay(1000);

    VextOFF();  

    Serial.println("Entering deep sleep... Press reset to wake.");
    esp_deep_sleep_start();  
}


void sendPacket() {
    snprintf(txpacket, sizeof(txpacket), "AVP:GARAGE:%d", esp_random());
    Radio.Send((uint8_t *)txpacket, strlen(txpacket));
    startRetryTime = millis();
    lora_idle = false;

    display.clear();
    display.setTextAlignment(TEXT_ALIGN_CENTER);
    display.drawString(display.getWidth() / 2, (display.getHeight() - 16) / 2, "Sending...");
    display.display();
}

void VextON(void) {
    pinMode(Vext, OUTPUT);
    digitalWrite(Vext, LOW);
}

void VextOFF(void) {
    pinMode(Vext, OUTPUT);
    digitalWrite(Vext, HIGH);
}

void loop() {
    Radio.IrqProcess();  // Process any pending interrupts

    if (!lora_idle) {
        return;  // Exit if LoRa is still busy
    }

    // Retry sending the packet if ACK not received and within retry duration
    if (!ack_received && (millis() - startRetryTime) > RETRY_INTERVAL) {
        if ((millis() - startRetryTime) < MAX_RETRY_DURATION) {
            Serial.println("Re-sending...");
            display.clear();
            display.setTextAlignment(TEXT_ALIGN_CENTER);
            display.drawString(display.getWidth() / 2, (display.getHeight() - 16) / 2, "Re-sending...");
            display.display();

            sendPacket();  // Retry sending the packet
        } else {
            Serial.println("Send failed after 1 minute. Entering deep sleep.");
            display.clear();
            display.setTextAlignment(TEXT_ALIGN_CENTER);
            display.drawString(display.getWidth() / 2, (display.getHeight() - 16) / 2, "Failed...");
            display.display();
            delay(2000); // Display "Failed..." for 2 seconds
            goToSleep(); // Call sleep function
        }
    }

    // Go to deep sleep if ACK is received
    if (ack_received) {
        Serial.println("ACK received. Going to deep sleep...");
        goToSleep();  // Call sleep function
    }
}

void OnTxDone(void) {
    Serial.println("TX done...");
    lora_idle = true;
    // Switch to receive mode to listen for ACK
    Radio.Rx(RX_TIMEOUT_VALUE);  // Listen for ACK with a 5-second timeout
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
    char rxPayload[BUFFER_SIZE + 1];
    memcpy(rxPayload, payload, size);
    rxPayload[size] = '\0';  // Null-terminate the received string

    Serial.printf("Received packet: \"%s\" with RSSI: %d and SNR: %d\r\n", rxPayload, rssi, snr);

    if (strcmp(rxPayload, "ACK") == 0) {
        ack_received = true;
    }

    lora_idle = true;
}

void OnTxTimeout(void) {
    Radio.Sleep();
    Serial.println("TX Timeout...");
    lora_idle = true;
}

void OnRxTimeout(void) {
    Radio.Sleep();
    Serial.println("RX Timeout...");
    lora_idle = true;
}

void OnRxError(void) {
    Radio.Sleep();
    Serial.println("RX Error...");
    lora_idle = true;
}