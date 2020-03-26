/**
 ******************************************************************************
 * @file    S2LP_HelloWorld.ino
 * @author  SRA
 * @version V1.0.0
 * @date    March 2020
 * @brief   Arduino test application for the STMicrolectronics
 *          Sub-1 GHz RF expansion board based on the S2-LP module.
 *          This application makes use of C++ classes obtained from the C
 *          components' drivers.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2020 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

#include "SPI.h"
#include "S2LP.h"

#define SerialPort Serial

SPIClass *devSPI;
S2LP *myS2LP;
volatile uint8_t receive_packet = 0;
const int buttonPin = PC13; // set buttonPin to digital pin PC13 */
int pushButtonState = LOW;

static uint8_t send_buf[FIFO_SIZE] ={'S','2','L','P',' ','H','E','L','L','O',' ','W','O','R','L','D',' ','P','2','P',' ','D','E','M','O'};
static uint8_t read_buf[FIFO_SIZE] ={0};

void callback_func(void);
void recv_data(void);
void blink_led(void);


/* Setup ---------------------------------------------------------------------*/

void setup() {
  // Initialize serial for output.
  SerialPort.begin(115200);

  // Initialize Led.
  pinMode(LED_BUILTIN, OUTPUT);

  // Initialize Button
  pinMode(buttonPin, INPUT);
  pushButtonState = (digitalRead(buttonPin)) ?  LOW : HIGH;

  // Initialize SPI
  devSPI = new SPIClass(D11, D12, D3);
  devSPI->begin();

  // Initialize S2-LP
  myS2LP = new S2LP(devSPI, A1, D7, A5);
  myS2LP->begin();
  myS2LP->attachS2LPReceive(callback_func);
}

/* Loop ----------------------------------------------------------------------*/

void loop() {
  if(digitalRead(buttonPin) == pushButtonState)
  {
    /* Debouncing */
    HAL_Delay(50);

    /* Wait until the button is released */
    while (digitalRead(buttonPin) == pushButtonState);

    /* Debouncing */
    HAL_Delay(50);

    if(!myS2LP->send(send_buf, (strlen((char *)send_buf) + 1), 0x44, true))
    {
      /* Blink LED */
      blink_led();

      /* Print message */
      SerialPort.print("Transmitted ");
      SerialPort.print((strlen((char *)send_buf) + 1));
      SerialPort.println(" bytes successfully");
    } else
    {
      SerialPort.println("Error in transmission");
    }
  }

  if(receive_packet)
  {
    receive_packet = 0;
    recv_data();

    /* Blink LED */
    blink_led();
  }
}

void recv_data(void)
{
  uint8_t data_size = myS2LP->getRecvPayloadLen();

  myS2LP->read(read_buf, data_size);

  SerialPort.print("Received packet (size of ");
  SerialPort.print(data_size);
  SerialPort.print(" bytes): ");
  SerialPort.println((char *)read_buf);
}

void callback_func(void)
{
  receive_packet = 1;
}

void blink_led(void)
{
  digitalWrite(LED_BUILTIN, HIGH);
  delay(10);
  digitalWrite(LED_BUILTIN, LOW);
  delay(10);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(10);
  digitalWrite(LED_BUILTIN, LOW);
}
