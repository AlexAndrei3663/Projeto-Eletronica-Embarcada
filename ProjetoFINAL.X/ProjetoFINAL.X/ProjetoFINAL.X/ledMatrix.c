
#include "mcc_generated_files/mcc.h"
#include "ledMatrix.h"
#include <string.h>

// Define valores de uma matriz de 4x8 para repersentra números entre 0-9
// A parte inferior do número é o bit 0.
// Tabela de conversão (lookup) na EEPROM. São 10 símbolos (0-9) de 4x8 LEDs.
__EEPROM_DATA(
        0b11111111,     // 0
        0b10000001,
        0b10000001,
        0b11111111,
        0b00000000,     // 1
        0b00000000,
        0b00000000,   
        0b11111111);
__EEPROM_DATA(
        0b11000010,     // 2
        0b10100001,
        0b10010001,
        0b10001110,
        0b01000010,     // 3
        0b10001001,
        0b10001001,
        0b01110110   
    );
__EEPROM_DATA(
        0b00011111,     // 4
        0b00010000,
        0b00010000,
        0b11111111,
        0b10001111,     // 5
        0b10001001,
        0b10001001,
        0b01110001);
__EEPROM_DATA(
        0b01111110,     // 6
        0b10001001,
        0b10001001,
        0b01110010,
        0b00000000,     // -
        0b00001000,
        0b00001000,
        0b00001000);
__EEPROM_DATA(
        0b00000000,     // >
        0b00110000,
        0b00001100,
        0b00110000,
        0b00000000,     // <
        0b00001100,
        0b00110000,
        0b00001100);      

void txMAX7219(uint8_t addr0, uint8_t dat0, uint8_t addr1, uint8_t dat1){
//void txMAX7219(uint8_t addr0, uint8_t dat0){
    CS_SetLow();
    SPI1_WriteByte(addr0);
    while(!PIR1bits.SSP1IF);
    PIR1bits.SSP1IF = 0;
    SPI1_WriteByte(dat0);
    while(!PIR1bits.SSP1IF);
    PIR1bits.SSP1IF = 0;
    SPI1_WriteByte(addr1);
    while(!PIR1bits.SSP1IF);
    PIR1bits.SSP1IF = 0;
    SPI1_WriteByte(dat1);
    while(!PIR1bits.SSP1IF);
    PIR1bits.SSP1IF = 0;
    CS_SetHigh();
}

void initMAX7219(){
    // Decode mode = 0
    txMAX7219(0x09,0x00,0x09,0x00);
    // Intensity 17/32
    txMAX7219(0x0A,0x00,0x09,0x00);
    // Shutdown mode = 0
    txMAX7219(0x0C,0x00,0x0C,0x00);
    // Scan Limit
    txMAX7219(0x0B,0x07,0x0B,0x07);
    // Shutdown mode = 1
    txMAX7219(0x0C,0x01,0x0C,0x01);
    // Display-Test = 1
    txMAX7219(0x0F,0x01,0x0F,0x01);
    __delay_ms(1000);
    // Display-Test = 0
    txMAX7219(0x0F,0x00,0x0F,0x00);
    // Shutdown mode = 1
    txMAX7219(0x0C,0x01,0x0C,0x01);
}
//void initMAX7219(){
//    // Decode mode = 0
//    txMAX7219(0x09,0x00);
//    // Intensity 17/32
//    txMAX7219(0x0A,0x00);
//    // Shutdown mode = 0
//    txMAX7219(0x0C,0x00);
//    // Scan Limit
//    txMAX7219(0x0B,0x07);
//    // Shutdown mode = 1
//    txMAX7219(0x0C,0x01);
//    // Display-Test = 1
//    txMAX7219(0x0F,0x01);
//    __delay_ms(1000);
//    // Display-Test = 0
//    txMAX7219(0x0F,0x00);
//    // Shutdown mode = 1
//    txMAX7219(0x0C,0x01);
//}

void updateMatrix(uint8_t pos, uint8_t val){
	// Leitura do valor (a eeprom guarda 8 bits por localização)
	// e se precisa de 4 localizações para um valor na matriz de LEDs.
	// Se multiplica o valor (val) por 4 (deslocar para a direita 2 vezes) e com isso temos a localização na EEPROM 	
	uint8_t eeAdd = val<<2;
	for(uint8_t i=0;i<4;i++){
		matrix[pos++]=DATAEE_ReadByte(eeAdd++);   
    }
}

void sendMatrix(){
    for(uint8_t i=0;i<8;i++){
#if CAMERA_FLIP==1
        //txMAX7219(i+1,matrix[7-i],i+1,matrix[15-i]);
        txMAX7219(i+1,matrix[7-i],i+1,matrix[7-i]);
#else
        //txMAX7219(i+1,matrix[i+8],i+1,matrix[i]);
        txMAX7219(i+1,matrix[i+8],i+1,matrix[7-i]);
#endif
    }
}

