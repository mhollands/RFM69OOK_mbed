
// **********************************************************************************
// Copyright Matt Hollands (2016), felix@lowpowerlab.com
// http://projects.matthollands.com
// **********************************************************************************
// License
// **********************************************************************************
// This program is free software; you can redistribute it
// and/or modify it under the terms of the GNU General
// Public License as published by the Free Software
// Foundation; either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will
// be useful, but WITHOUT ANY WARRANTY; without even the
// implied warranty of MERCHANTABILITY or FITNESS FOR A
// PARTICULAR PURPOSE. See the GNU General Public
// License for more details.
//
// You should have received a copy of the GNU General
// Public License along with this program.
// If not, see <http://www.gnu.org/licenses/>.
//
// Licence can be viewed at
// http://www.gnu.org/licenses/gpl-3.0.txt
//
// Please maintain this license information along with authorship
// and copyright notices in any redistribution of this code
// **********************************************************************************

#include "mbed.h"
#include <RFM69OOK.h>
#include <RFM69OOKregisters.h>

void resetModule();
void irq();
DigitalOut myled(LED1);
RFM69OOK radio; //define the RFM69 Radio
Serial pc(USBTX, USBRX); // define the USB serial port
DigitalOut reset(p21); //define the Reset pin for the Radio

int main() {
    SPI spi(p11, p12, p13); //initialse SPI port
    resetModule(); //reset RFM69
    radio.attachUserInterrupt(irq); //Set data pin interrupt function
    radio.initialize(&spi); //initialise radio
    radio.setFrequencyMHz(433.9);
    radio.setPowerLevel(20);

    while(1==1)
    {
        myled = 1; //turn on LED
        radio.transmitBegin(); //start a transmission 
         
        radio.send(1);
        wait_us(300L);
        
        radio.send(0);
        wait_us(300L);
        
        radio.send(1);
        wait_us(300L);
        
        radio.send(0);
        wait_us(300L);
        
        radio.send(1);
        wait_us(300L);
        
        radio.send(0);
        wait_us(300L);
        
        radio.transmitEnd();
        
        radio.receiveBegin(); //being receiving
        pc.putc(pc.getc()); //wait for serial character
        radio.receiveEnd(); //end receiving
    }
    
}

void irq()
{
    //when interrupt is thrown (ie RFM data pin changes and in receive mode), turn off LED
    myled = 0;   
}

void resetModule()
{
    reset = 1;
    wait(0.2);
    reset = 0;
    wait(0.1);
}