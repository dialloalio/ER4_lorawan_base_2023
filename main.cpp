/**
 * Copyright (c) 2017, Arm Limited and affiliates.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <stdio.h>
#include "th02.h"
#include "CayenneLPP.h"
#include "lorawan/LoRaWANInterface.h"
#include "lorawan/system/lorawan_data_structures.h"
#include "events/EventQueue.h"

// Application helpers
#include "DummySensor.h"
#include "trace_helper.h"
#include "lora_radio_helper.h"
#include "Servo.h"

using namespace events;

// Max payload size can be LORAMAC_PHY_MAXPAYLOAD.
// This example only communicates with much shorter messages (<30 bytes).
// If longer messages are used, these buffers must be changed accordingly.
uint8_t tx_buffer[30];
uint8_t rx_buffer[30];

#define MAX_SIZE 200 // depends on spreading factor and frequency used

CayenneLPP Payload(MAX_SIZE);

float celsius = -4.1;
float accel[] = {1.234, -1.234, 0};
float rh = 30;
float hpa = 1014.1;
float latitude = 42.3519;
float longitude = -87.9094;
float altitude=10;

int size = 0;


DigitalOut Alarme (PC_13);// alarme LED output
Servo Myservo(PA_7); //servomotor output
TH02 MyTH02 (I2C_SDA,I2C_SCL,TH02_I2C_ADDR<<1);// connect hsensor on RX2 TX2
/*
 * Sets up an application dependent transmission timer in ms. Used only when Duty Cycling is off for testing
 */
#define TX_TIMER                        20000

/**
 * Maximum number of events for the event queue.
 * 10 is the safe number for the stack events, however, if application
 * also uses the queue for whatever purposes, this number should be increased.
 */
#define MAX_NUMBER_OF_EVENTS            30

/**
 * Maximum number of retries for CONFIRMED messages before giving up
 */
#define CONFIRMED_MSG_RETRY_COUNTER     3

/**
 * Dummy pin for dummy sensor
 */
#define PC_9                            0

/**
 * Dummy sensor class object
 */
DS1820  ds1820(PC_9);

/**
* This event queue is the global event queue for both the
* application and stack. To conserve memory, the stack is designed to run
* in the same thread as the application and the application is responsible for
* providing an event queue to the stack that will be used for ISR deferment as
* well as application information event queuing.
*/
static EventQueue ev_queue(MAX_NUMBER_OF_EVENTS *EVENTS_EVENT_SIZE);

/**
 * Event handler.
 *
 * This will be passed to the LoRaWAN stack to queue events for the
 * application which in turn drive the application.
 */
static void lora_event_handler(lorawan_event_t event);

/**
 * Constructing Mbed LoRaWANInterface and passing it the radio object from lora_radio_helper.
 */
static LoRaWANInterface lorawan(radio);

/**
 * Application specific callbacks
 */
static lorawan_app_callbacks_t callbacks;

void servo(uint8_t uAngle)
{
    
    }



/**
 * Entry point for application
 */
int main(void)
{
    // setup tracing
    setup_trace();
   //  th02 temerature sensor section 
    int iTemp,iTime,iTempbrute,iRH,iRHbrute;
    printf ("\n\r start reading TH02 for first time");
     MyTH02.startTempConv(true,true);
  



    // stores the status of a call to LoRaWAN protocol
    lorawan_status_t retcode;

    // Initialize LoRaWAN stack
    if (lorawan.initialize(&ev_queue) != LORAWAN_STATUS_OK) {
        printf("\r\n LoRa initialization failed! \r\n");
        return -1;
    }

    printf("\r\n Mbed LoRaWANStack initialized \r\n");

    // prepare application callbacks
    callbacks.events = mbed::callback(lora_event_handler);
    lorawan.add_app_callbacks(&callbacks);

    // Set number of retries in case of CONFIRMED messages
    if (lorawan.set_confirmed_msg_retries(CONFIRMED_MSG_RETRY_COUNTER)
            != LORAWAN_STATUS_OK) {
        printf("\r\n set_confirmed_msg_retries failed! \r\n\r\n");
        return -1;
    }

    printf("\r\n CONFIRMED message retries : %d \r\n",
           CONFIRMED_MSG_RETRY_COUNTER);

    // Enable adaptive data rate
    if (lorawan.enable_adaptive_datarate() != LORAWAN_STATUS_OK) {
        printf("\r\n enable_adaptive_datarate failed! \r\n");
        return -1;
    }

    printf("\r\n Adaptive data  rate (ADR) - Enabled \r\n");

    retcode = lorawan.connect();

    if (retcode == LORAWAN_STATUS_OK ||
            retcode == LORAWAN_STATUS_CONNECT_IN_PROGRESS) {
    } else {
        printf("\r\n Connection error, code = %d \r\n", retcode);
        return -1;
    }

    printf("\r\n Connection - In Progress ...\r\n");

    // make your event queue dispatching events forever
    ev_queue.dispatch_forever();

    return 0;
}

/*************************************************************************************************************
 * Sends a message to the Network Server
 *************************************************************************************************************/
static void send_message()
 {int iTime,iTempbrute,iRHbrute;   
 uint16_t packet_len;
    int16_t retcode;
    int32_t sensor_value, rh_value;

 MyTH02.startTempConv(true,true);
    iTime= MyTH02.waitEndConversion();// wait until onversion  is done
     iTempbrute= MyTH02.getConversionValue();
   
    printf ("\n\r temp value=%d",iTempbrute );
  
 MyTH02.startRHConv(true,true);
    iTime= MyTH02.waitEndConversion();// wait until onversion  is done
     iRHbrute= MyTH02.getConversionValue();
   
    printf ("\n\r humidity value=  %d",iRHbrute );

/*
    if (ds1820.begin()) {
        ds1820.startConversion();
        sensor_value = ds1820.read();
        printf("\r\n Dummy Sensor Value = %d \r\n", sensor_value);
        ds1820.startConversion();
    } else {
        printf("\r\n No sensor found \r\n");
        return;
    }
  */  
          
    Payload.reset();
    size = Payload.addTemperature(1, (float) iTempbrute);    
    size =size+ Payload.addRelativeHumidity(1, (float) iRHbrute);  

// send complete message with cayenne format
      retcode = lorawan.send(MBED_CONF_LORA_APP_PORT, Payload.getBuffer(), Payload.getSize(),
                           MSG_UNCONFIRMED_FLAG);                

    if (retcode < 0) {
        retcode == LORAWAN_STATUS_WOULD_BLOCK ? printf("send - WOULD BLOCK\r\n")
        : printf("\r\n send() - Error code %d \r\n", retcode);

        if (retcode == LORAWAN_STATUS_WOULD_BLOCK) {
            //retry in 3 seconds
            if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
                ev_queue.call_in(3000, send_message);
            }
        }
        return;
    }

    printf("\r\n %d bytes scheduled for transmission \r\n", retcode);
    memset(tx_buffer, 0, sizeof(tx_buffer));
}

/**
 * Receive a message from the Network Server
 */
static void receive_message()
{int num_port, iPosition,iIndex;
    uint8_t port;
    int flags;
    int16_t retcode = lorawan.receive(rx_buffer, sizeof(rx_buffer), port, flags);

    if (retcode < 0) {
        printf("\r\n receive() - Error code %d \r\n", retcode);
        return;
    }

    printf(" RX Data on port %u (%d bytes): ", port, retcode);
    for (uint8_t i = 0; i < retcode; i++) {
        printf("%02x", rx_buffer[i]);
    }
     printf("\n test value=%d", port); 
 // *****************************code todo here   ********************************************
     switch (port){
        case 3: // control led
         printf("\n led=%d", (int)rx_buffer[0]); 
            Alarme.write((int) rx_buffer[0]);
        break;
        case 4:// control servomotor
        for (iIndex=0;iIndex<retcode;iIndex++)
        {iPosition=iPosition*10+(rx_buffer[iIndex]-0x30);// convert receive string to angular position
        } 
              
      
        printf("\n servo=%d",(int) iPosition); 
      Myservo.position ( iPosition ); // set servo motor position from 0 to 180
       break;
       default: printf("\n port inconnu =%d",(int)port); 
       break;
        }
 
 
 //  ***************************** end code todo here   *****************************************
   
        
    memset(rx_buffer, 0, sizeof(rx_buffer));
}

/**
 * Event handler
 */
static void lora_event_handler(lorawan_event_t event)
{
    switch (event) {
        case CONNECTED:
            printf("\r\n Connection - Successful \r\n");
            if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
                send_message();
            } else {
                ev_queue.call_every(TX_TIMER, send_message);
            }

            break;
        case DISCONNECTED:
            ev_queue.break_dispatch();
            printf("\r\n Disconnected Successfully \r\n");
            break;
        case TX_DONE:
            printf("\r\n Message Sent to Network Server \r\n");
            if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
                send_message();
            }
            break;
        case TX_TIMEOUT:
        case TX_ERROR:
        case TX_CRYPTO_ERROR:
        case TX_SCHEDULING_ERROR:
            printf("\r\n Transmission Error - EventCode = %d \r\n", event);
            // try again
            if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
                send_message();
            }
            break;
        case RX_DONE:
            printf("\r\n Received message from Network Server \r\n");
            receive_message();
            break;
        case RX_TIMEOUT:
         printf("\r\n timeout in reception - Code = %d \r\n", event);
            break;
        case RX_ERROR:
            printf("\r\n Error in reception - Code = %d \r\n", event);
            break;
        case JOIN_FAILURE:
            printf("\r\n OTAA Failed - Check Keys \r\n");
            break;
        case UPLINK_REQUIRED:
            printf("\r\n Uplink required by NS \r\n");
            if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
                send_message();
            }
            break;
        default:
            MBED_ASSERT("Unknown Event");
    }
}

// EOF
