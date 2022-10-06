#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include "bitmanip.h"

//fs=9600Hz
 
int16_t Coeffloi[] = {32767, 23169, 0, -23169, -32767, -23169, 0, 23169};
int16_t Coeffloq[] = {0, 23169, 32767, 23169, 0, -23169, -32767, -23169};
int16_t Coeffhii[] = {32767, 4276, -31650, -12539, 28377, 19947, -23169, -25995};
int16_t Coeffhiq[] = {0, 32486, 8480, -30272, -16383, 25995, 23169, -19947};

int32_t demodulate(int16_t data);
uint8_t hysteresis_comparator(int32_t data);
void clock_regen(uint8_t r);
uint8_t diff_decode(uint8_t r);
void search_frame(uint8_t bit);
void unstuff(uint8_t bit, int8_t se_flag);
void frame_process(uint8_t byte, int8_t se_flag);
uint8_t TMRdecode(uint8_t r1, uint8_t r2,uint8_t r3); 
void crc_update_bit(uint8_t crc_bit);
void crc_update_byte(uint8_t byte);
void crc_init(void);
uint16_t Crc;

int main(void){
    int16_t dat;

    while(scanf("%hd", &dat)!=EOF){
        clock_regen(hysteresis_comparator(demodulate(dat)));
    } 
    return 0;
}

#define THRES_MAX   100000L
#define THRES_MIN  -100000L

uint8_t hysteresis_comparator(int32_t data){
    static uint8_t res=0;
    if (res==0){
        if (data>THRES_MAX) res=1;
    }else{
        if (data<THRES_MIN) res=0;
    }
    return res;
}

#define PHASE_INC (256/8) // 2^(phase register bit length=8)/(samples per code=8)
#define PHASE_CORR (PHASE_INC/2)

//regenerate clock according to edges
void clock_regen(uint8_t r){
    static uint8_t bit_phase;
    static uint8_t oldr;
    static uint8_t oldoldr; 
    uint16_t temp;
    if (oldr!=r){ //edge detection 
        if(bit_phase<0x80){
            bit_phase+=PHASE_CORR;
        }else{ 
            bit_phase-=PHASE_CORR;
        } 
    }
    temp=bit_phase+PHASE_INC;
    bit_phase=temp&0xff;
    if(temp>0xff) { //sample at half cycle
        //TMR decoding
        search_frame(diff_decode(TMRdecode(r, oldr, oldoldr)));   
    }
    oldoldr=oldr;
    oldr=r;

}

//Triple modular redundant (two-out-of-three) decoder
//Inputs must be 0/1
uint8_t TMRdecode(uint8_t r1, uint8_t r2,uint8_t r3){
    if (r1+r2+r3>=2){
        return 1;
    }else{
        return 0; 
    } 
}

#define SE_NORMAL 0
#define SE_START -1
#define SE_END 1

#define SF_STAT_SEARCHSYNC 1
#define SF_STAT_FOUNDSYNC 2
#define SF_STAT_DATAVALID 4

#define SYNC_WORD 0x7eU //0b01111110
#define SYNC_WORD_SHR 0x3fU //0b111111

void search_frame(uint8_t bit){
    static uint8_t currbyte=0;
    static uint8_t sync_counter=8;  //slide in 8 bits after sync word is found 
    static uint8_t status=SF_STAT_SEARCHSYNC;
    uint8_t i;
    
    currbyte=(currbyte<<1) | bit;
    switch(status){
    case SF_STAT_SEARCHSYNC:
        //If sync word is found, change status to SF_STAT_FOUNDSYNC
        if (currbyte==SYNC_WORD){
            sync_counter=8; 
            status=SF_STAT_FOUNDSYNC;
        }
        break;
    case SF_STAT_FOUNDSYNC:
        //slide in 8 bits after sync word is found 
        sync_counter--;
        if (sync_counter==0){
            sync_counter=8; 
            //find first "not sync word", i.e. data word
            if (currbyte!=SYNC_WORD){
                //change status
                status=SF_STAT_DATAVALID;
                //send data word to unstuff()
                unstuff(0, SE_START); 
                for(i=7;i>5;i--){
                    unstuff(CHB(currbyte,BIT(i))?1:0, SE_NORMAL);
                } 
            } 
        } 
        
        break;
    case SF_STAT_DATAVALID:
        //If successive six "1"s are found, change status to SF_STAT_SEARCHSYNC 
        if ((currbyte & SYNC_WORD_SHR) == SYNC_WORD_SHR){
            //End data frame 
            unstuff(0, SE_END);            
            status=SF_STAT_SEARCHSYNC;
        }else{
	        //Process normal data
            unstuff(CHB(currbyte,BIT(6))?1:0, SE_NORMAL);
        }
        break;        
    default:
        break;
        
    } 

}



//Unstuff "0" after five successive "1"s and process data in byte 
//Normal: se_flag=SE_NORMAL
//End sync frame is received: se_flag=SE_END
//Initialize at frame start: se_flag=SE_START 
void unstuff(uint8_t bit, int8_t se_flag){
    static uint8_t currbyte=0, outbyte=0;
    static uint8_t byte_counter=8;  //output in 8 bits
    
    if (se_flag==SE_START){
        currbyte=0;
        outbyte=0;
        byte_counter=8; 
        frame_process(0, se_flag);
    }else{
     
        currbyte=(currbyte<<1) | bit;
        
        if ((currbyte & 0x3e)==0x3e) { //If 0b11111x
            //then chomp "x" and do not count
        }else{
            outbyte=(outbyte>>1) | (bit<<7);
            byte_counter--;
        }
        
        
        if (byte_counter==0){
            byte_counter=8;
            frame_process(outbyte, se_flag);
        }
    
        if (se_flag==SE_END){
            frame_process(outbyte, se_flag);
        } 
    } 
}


//Process frame
//extract address, CTRL, PID, data, CRC
//3 bytes must be buffered, for AX.25 frame doesn't have a length field
//print CRC (2 bytes) after frame end and reset statuses
#define FR_STAT_FIRST 1
#define FR_STAT_ADDR 2 
#define FR_STAT_CTRL 4 
#define FR_STAT_PID 8 
#define FR_STAT_DAT 16 
 
void frame_process(uint8_t byte, int8_t se_flag){
    static uint8_t status=FR_STAT_FIRST;
    static int16_t oldbyte[3]={-1, -1, -1};  //oldbyte:0 oldoldbyte:1 oldoldbyte:2
    static uint8_t addrptr=0;
    uint8_t addrb;
    
    if (se_flag==SE_START){
        //Initialize
        status=FR_STAT_FIRST;
        oldbyte[0]=-1;
        oldbyte[1]=-1;
        oldbyte[2]=-1;
        addrptr=0;  
        crc_init();

        printf("\n"); 
    }else{        
        switch(status){
        case FR_STAT_FIRST:

            crc_update_byte(byte);

            addrptr++;
            printf("\nADDR: %c", byte>>1);
            status=FR_STAT_ADDR;
            break; 
        case FR_STAT_ADDR:

            crc_update_byte(byte);

            addrb=byte>>1;
            addrptr++;
   
            if (addrptr==7){
                printf("-%d", addrb&0x0f);
            }else{
                printf("%c", addrb);        
            }
            if (addrptr>=7){
                addrptr=0;
                printf(" ");
            }
            if ((byte & 1) == 1){
                printf ("\nCTRL: ");
                status=FR_STAT_CTRL;
            }
            
            break;
        case FR_STAT_CTRL:

            crc_update_byte(byte);

            printf ("%2.2X\nPID:  ", byte);
            status=FR_STAT_PID;
            break;
        case FR_STAT_PID:

            crc_update_byte(byte);

            printf ("%2.2X\nDATA: ", byte);
            status=FR_STAT_DAT;
                
            break;
        case FR_STAT_DAT:
            if (oldbyte[0]!=-1 && oldbyte[1]!=-1 && oldbyte[2]!=-1){

                crc_update_byte(oldbyte[2]);

                if (oldbyte[2]>=32 && oldbyte[2]<=126){
                    printf ("%c", (uint8_t)oldbyte[2]);
                }else{
                    printf(".");        
                }

                if (se_flag==SE_END){
                    printf("\nCRC:  %2.2X %2.2X\n", oldbyte[0], oldbyte[1]);
                    printf("\nCrc:  %4.4X\n", Crc^0xffff);
                    
                }    
            } 
            oldbyte[2]=oldbyte[1];
            oldbyte[1]=oldbyte[0];
            oldbyte[0]=byte;
            break;
    
        default:
            break;
        }    
    }
}



void crc_init(void){
    Crc=0xffff;
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


uint8_t diff_decode(uint8_t r){
    static uint8_t oldr;
    uint8_t out;
    if (r!=oldr){
        out=0;
    }else{ 
        out=1;
    }
    oldr=r; 
    return out; 
}

//demodulator modified from Wayne Holder's "Bell 202, 1200 baud Demodulator in an ATTiny10"
int32_t demodulate(int16_t data){
    int32_t outloi, outloq, outhii, outhiq;
    int16_t d;
    uint8_t i;
    
    static uint8_t idx=0;
    static uint16_t datseq[8];
   
    outloi=0;
    outloq=0;
    outhii=0;
    outhiq=0;
    
    datseq[idx]=data;
    idx=(idx+1)%8;

    for (i=0; i<8; i++) {
        d=datseq[(idx+i)%8]; 
        outloi += d * Coeffloi[i];
        outloq += d * Coeffloq[i];
        outhii += d * Coeffhii[i];
        outhiq += d * Coeffhiq[i];
    }
    
    
    return (outloi >> 16) * (outloi >> 16) + (outloq >> 16) * (outloq >> 16) - 
        ((outhii >> 16) * (outhii >> 16) + (outhiq >> 16) * (outhiq >> 16));
}
