#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include "bitmanip.h"

uint16_t Crc;
uint8_t Stuff_currbyte;

//COSTAB: Total 48 points, fs=9600Hz
//output every point: 200Hz
int16_t COSTAB[]={2047, 2029, 1977, 1891, 1772, 1623,
    1447, 1246, 1023, 783, 529, 267,
    0, -267, -529, -783, -1023, -1246,
    -1447, -1623, -1772, -1891, -1977, -2029,
    -2047, -2029, -1977, -1891, -1772, -1623,
    -1447, -1246, -1023, -783, -529, -267, 
    0, 267, 529, 783, 1023, 1246,
    1447, 1623, 1772, 1891, 1977, 2029};

//1200=200*6, 2200=200*11
//send every 6 points for 1200Hz, every 11 points for 2200Hz
uint16_t DDS_INCR[]={6, 11};

void modulate(uint8_t bit);
void dataframe_start(void);
void crc_update_byte(uint8_t byte);
void crc_update_bit(uint8_t crc_bit);
void transmit_APRS_frame(void);
void transmit_byte(uint8_t byte);
void stuff_transmit(uint8_t bit);
void transmit_sync(void);
void nrzi_modulate(uint8_t bit);
void modulate(uint8_t bit);

int main(void){
    
    transmit_APRS_frame();    

    return 0;
}
uint8_t Info[]="!0000.00N/00000.00W>";
uint8_t Info2[]="!4000.00N/11600.00W>";

void transmit_APRS_frame(void){
    uint8_t i, crcl, crch;
    uint8_t* pi;

    for(i=0;i<5;i++){
        transmit_sync();
    }
    dataframe_start();
    transmit_byte('A'<<1);
    transmit_byte('P'<<1);
    transmit_byte('R'<<1);
    transmit_byte('S'<<1);
    transmit_byte(' '<<1);
    transmit_byte(' '<<1);
    transmit_byte(0xe0);

    transmit_byte('B'<<1);
    transmit_byte('H'<<1);
    transmit_byte('1'<<1);
    transmit_byte('P'<<1);
    transmit_byte('H'<<1);
    transmit_byte('L'<<1);
    transmit_byte(0xe2);

    transmit_byte('W'<<1);
    transmit_byte('I'<<1);
    transmit_byte('D'<<1);
    transmit_byte('E'<<1);
    transmit_byte('1'<<1);
    transmit_byte(' '<<1);
    transmit_byte(0x63);

    transmit_byte(0x03);
    transmit_byte(0xf0);
    for(pi=Info; *pi!=0; pi++){
        transmit_byte(*pi);
    }
    Crc^=0xffff;
    crcl=Crc&0xff;
    crch=(Crc>>8)&0xff;
    transmit_byte(crcl);
    transmit_byte(crch);

    //Test "back-to-back" data frames
    transmit_sync();

    dataframe_start();
    transmit_byte('A'<<1);
    transmit_byte('P'<<1);
    transmit_byte('R'<<1);
    transmit_byte('S'<<1);
    transmit_byte('2'<<1);
    transmit_byte(' '<<1);
    transmit_byte(0xe0);

    transmit_byte('B'<<1);
    transmit_byte('H'<<1);
    transmit_byte('1'<<1);
    transmit_byte('P'<<1);
    transmit_byte('H'<<1);
    transmit_byte('L'<<1);
    transmit_byte(0xe4);

    transmit_byte('W'<<1);
    transmit_byte('I'<<1);
    transmit_byte('D'<<1);
    transmit_byte('E'<<1);
    transmit_byte('1'<<1);
    transmit_byte(' '<<1);
    transmit_byte(0x63);

    transmit_byte(0x03);
    transmit_byte(0xf0);
    for(pi=Info2; *pi!=0; pi++){
        transmit_byte(*pi);
    }
    Crc^=0xffff;
    crcl=Crc&0xff;
    crch=(Crc>>8)&0xff;
    transmit_byte(crcl);
    transmit_byte(crch);
    for(i=0;i<5;i++){
        transmit_sync();
    }

} 

void dataframe_start(void){
    Crc=0xffff;
    Stuff_currbyte=0;
}



void crc_update_byte(uint8_t byte){
    uint8_t i;
    for(i=0;i<8;i++){
       crc_update_bit(CHB(byte,BIT(i))?1:0);
    }
}
//Update CRC by bit
void crc_update_bit(uint8_t crc_bit){
    uint8_t shiftbit;
    
    shiftbit=0x0001&(uint8_t)Crc;
    Crc>>=1;
    if(shiftbit != crc_bit){ 
        Crc ^= 0x8408;
    }
}

void transmit_byte(uint8_t byte){
    uint8_t i;
    for (i=0;i<8;i++){
        stuff_transmit(CHB(byte,BIT(i))?1:0);
    }
    crc_update_byte(byte);
    
}

//Send one bit. If five "1"s are seen, stuff a 0
void stuff_transmit(uint8_t bit){
    
    Stuff_currbyte=(Stuff_currbyte<<1) | bit;
    nrzi_modulate(bit);
    if ((Stuff_currbyte & 0x1f)==0x1f){   //Stuff a "0" after five successive "1"s
        Stuff_currbyte=(Stuff_currbyte<<1); 
        nrzi_modulate(0);    
    } 
}

//Send sync word 0b01111110 
void transmit_sync(void){
    nrzi_modulate(0);    
    nrzi_modulate(1);    
    nrzi_modulate(1);    
    nrzi_modulate(1);    

    nrzi_modulate(1);    
    nrzi_modulate(1);    
    nrzi_modulate(1);    
    nrzi_modulate(0);    
}

//NRZI coding: state changed (0->1 or 1->0) for "0", state not changed for "1" 
void nrzi_modulate(uint8_t bit){
    static uint8_t oldstat=0;
    if (bit==0){
        oldstat=oldstat?0:1;
    }
    modulate(oldstat);
}

//Output at 1200 Baud
//0:1200Hz, 1:2200Hz 
void modulate(uint8_t bit){
    static uint16_t dds_phase=0;
    uint8_t i; 
    for(i=0;i<8;i++){    
        dds_phase = (dds_phase+DDS_INCR[bit])%48;
        printf("%d\n",COSTAB[dds_phase]);
    }
} 
