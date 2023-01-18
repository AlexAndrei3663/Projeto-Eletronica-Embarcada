/* 
 * File:   ledMatrix.h
 * Author: Guillermo
 *
 * Created on October 13, 2021, 2:45 PM
 */

#ifndef LEDMATRIX_H
#define	LEDMATRIX_H

#define CAMERA_FLIP 1

uint8_t matrix[16];  ///< Array que vai armazenar o que será enviado para as matrizes de LEDs (MAX7219). São 16 columnas ou registradores (2 matrizes) x 8 filas de LEDs cada.

/**
 * Envia dados para os MAX7219
 * @param addr0 Enedereço do regsitrador do primeiro MAX7219 a ser escrito.
 * @param dat0  Valor a ser enviado para o primeiro MAX7219.
 */
void txMAX7219(uint8_t addr0, uint8_t dat0,uint8_t addr1, uint8_t dat1);

/**
 * Inicializa os MAX7219
 */
void initMAX7219();

/**
 * Atualiza variável que contem os valores a serem enviados para o MAX7219.
 * @param pos Posição de inicio do caracter na matriz de LEDs (número do registrador)
 * @param val Valor a ser exibido
 */
void updateMatrix(uint8_t pos, uint8_t val);
/**
 * Envia matriz para os MAX7219
 */
void sendMatrix();


#endif	/* LEDMATRIX_H */

