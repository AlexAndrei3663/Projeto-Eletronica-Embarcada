/* 
 * File:   bin2bcd.h
 * Author: Guillermo
 *
 * Created on 11 de Outubro de 2021, 23:17
 */

#ifndef BIN2BCD_H
#define	BIN2BCD_H

#include <string.h>
#include <stdint.h>
#include "bin2bcd.h"

 /**
 * Converte de binario para BCD
 * @param binVal Valor binário a ser convertido.
 * @return Valor BCD retornado
 */
uint16_t bin2bcd(uint16_t binVal);


#endif	/* BIN2BCD_H */

