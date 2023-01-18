/*******************************************************************************
**                 UNIVERSIDADE DE BRASILIA - UNB                             **
** DISCIPLINA: FGA0096 - ELETRONICA EMBARCADA        TURMA:                   **
** PROFESSOR: GUILLERMO ALVAREZ BESTARD , DR. ENG. MECATRONICA                **
** ALUNO: FERNANDA DINIZ                           MATRICULA:19/0087099       **
**        NATHAN SPINOLA ZEIDAN                    MATRICULA:18/0025864       **
**        THALLES ANTONIO NEIVA SOUTO              MATRICULA:18/0028189       **
**        GUILHERME CARBALLAL OLIVEIRA SANTOS      MATRICULA:16/0050421       **
**               TRABALHO FINAL ELETRONICA EMBARCADA                          **
*******************************************************************************/

#include "mcc_generated_files/mcc.h"
#include "ledMatrix.h"

//constantes
#define pulso       .837       // 180mm/215pulso
#define conv_s      .000002  //para converter ms para s no calculo de speed
#define duty_pwm    409      // Se o PWM que define o ciclo util é de 10 bits, seu valor vai de 0(0%) a 1023(100%), como vamos manter fixo em 40%, o valor do ciclo util é 409.2, vamos usar 409 para aproximar

//variaveis
float position;
int temp =0,
    speed = 0,          // Vel 
    motor = 0,
    pulsos = 0,   
    pos_final = 0,  // recebe da serial um valor de 0-3
    pos_atual = 2;     
//bool templuzflag;
uint16_t  colect_value = 0;     // valor coletado para calculo de velocidade
uint8_t LED[8],                 // Vetor para armazenar os valores da matriz de led
        rxByte,                 // Vetor para armazenar os valores recebidos pelo módulo USART
        txByte[4];              // Vetor para armazenar os valores transmitidos para o módulo USART


/*
                         Main application
 * 
 * Perifericos usados:
 *  ADC      : Coleta dos dados de temperatura faz uso do FVR
 *  CCP3     : PWM 
 *  CCP4     : Capture ainda foi config - encoder
 *  CMP1     : comparacao usado para verificar o S3 SENSOR HALL (refenciador dac)
 *  CMP2     : comparacao usado para verificar o S4 SENSOR HALL (refenciador dac)
 *  DAC(5bit): setado para negativo VSS e para o positivo o VDD
 *  EUSART   : Comm serial com PC - a verificar
 *  FVR      : Para trabalhar com a tensao de 889mV do termometro  (buffer 1x para trabalhar com sessao 1.024V)
 *  MEMORY   : Guardar os valores da matrix de leds
 *  MSSP1    : Display led - falta testar
 *  TMR1     : nao foi setado usado para o Encoder
 *  TMR2     : nao foi setado usado para PWM
 *  TMR4     : TIMER DE INTERRUPCAO PARA COMUNICACAO 100mS POST 1:4 PRE 1:64 (99.84mS)
 */


//----------------------------------------Interrupcoes------------------------------------------------
//Veriricar se esta correto
void Enc(uint16_t enc_value){

    colect_value = enc_value;
    if(motor==2 && pulsos >= 0){
        pulsos--; 
    }
    else if(motor==1 && pulsos <= 215){
        pulsos++;
    }
    else if (motor==0){
        speed = 0;
    }
    else if (pulsos >160){    //zera o pwm e nao altera o valor dos pulsos (caso de erro)
        pos_atual = 4;
    }else{    
        pulsos = 0;
    }
    TMR1_WriteTimer(0);
}

// passa a info para o display de leds 8X8
void leds(void){
    for (int i =1;i<=8;i++)
    {
        txMAX7219(i, LED[i-1],i, LED[i]); //usado para 2 display leds so teremos um talvez tenha que realziar mudancas no 
                                              // arquivo ledmatrix.c
    }
}

// Troca de informações via Serial com o PC
void comm (void){   
    // TX
    txByte[0] = 0xBF & ((uint8_t)motor<<4 | (0b10000000) | (pos_atual & 0x03)); //1011 1111 AND 00XX 0000 AND (Pos_atual AND 0000 0011) -- Na duvida, fazer atualizacoes sequenciais
    txByte[1] = 0x7F & (((uint8_t)position)>>1);        //0111 1111 AND variável
    txByte[2] = 0x7F & (((uint8_t)speed)<<2);           //0111 1111 AND variável
    txByte[3] = 0x7F & ((temp)<<1);                     //0111 1111 AND variável

    for (int i=0;i<4;i++){
      EUSART_Write(txByte[i]);
    }
}

//Funcoes dos sensores hall
void S1(){	
	pos_atual = 0; //Recebe o valor 0 mas envia posicao com +1 na comm
	pulsos = 0;    //Valor inicial dos pulsos
}

void S2(){
	pos_atual = 1; //Recebe o valor 1 mas envia posicao com +1 na comm

}

void CMP1_ISR(void){
    //S3
    pos_atual = 2; //Recebe o valor 2 mas envia posicao com +1 na com
    // clear the CMP2 interrupt flag
    PIR2bits.C1IF = 0;
}


void CMP2_ISR(void){
    //S4
    pos_atual = 3; //Recebe o valor 3 mas envia posicao com +1 na comm
	//pulsos = 215;  //Valor maximo de pulsos 
    // clear the CMP2 interrupt flag
    PIR2bits.C1IF = 0;
}

//----------------------------------FUNCOES--------------------------------------------
void andar(void){
    for (int i =0;i<4;i++)
    {
        LED[i]= DATAEE_ReadByte(i+((1+pos_atual)*4));
        LED[4+i]= DATAEE_ReadByte(i+((7+motor)*4));
    }
    leds();
    // carrega o estado do motor: 
    // 0 parado         -
    // 1 ascendente     >
    // 2 descendente    <
}

void deslocamento(){
    
    // indicando o incio do movimento do motor
    if(pos_final==pos_atual){ 
         PWM3_LoadDutyValue(0);
         motor = 0;
         __delay_ms(2000); // 2 segundos para permitir o fluxo de pessoas (pode ser implementado diferente depois)
    }else if (pos_final != pos_atual) {
        PWM3_LoadDutyValue(duty_pwm);        
    }
}

 //controle do motor
void controle(){    
    //Dir = porta RA7   
    //Dir = 1 movimento ascendente 
    //Dir = 0 movimento decendente                          
    if(pos_final>pos_atual){
        if(motor==2){    
            PWM3_LoadDutyValue(0);
            __delay_ms(500);
        }
        //subida
        Dir_SetHigh();
        motor = 1;         
    } else if(pos_final<pos_atual){
        if(motor==1){
            PWM3_LoadDutyValue(0);
            __delay_ms(500);
        }
        //descida
        Dir_SetLow();
        motor = 2; 
    } 
    deslocamento();
}
void verificacao(){
    if(S1_GetValue()== 0){
        pos_atual = 0;
    } else if (S4_GetValue()== 0){
        pos_atual = 3;
    } else if (S3_GetValue()==0){
        pos_atual = 2;
    } else if (S2_GetValue()==0){
        pos_atual = 1;
    }
}
void main(void){
    // initialize the device
    SYSTEM_Initialize();
    SPI1_Open(SPI1_DEFAULT);
    initMAX7219();
    // Enable the Global Interrupts
    INTERRUPT_GlobalInterruptEnable();
    // Enable the Peripheral Interrupts
    INTERRUPT_PeripheralInterruptEnable();
    
    TMR4_SetInterruptHandler(comm);
	CCP4_SetCallBack(Enc);
    //EUSART_SetRxInterruptHandler(recepcao);
	
	//Conferir as interrupcoes dos sensores (configuracoes dos pinos) parece ok
	IOCBF0_SetInterruptHandler(S1); //Confirmar se é nessa interrupção
    IOCBF3_SetInterruptHandler(S2); //Confirmar se é nessa interrupção
    //reset de posicao
    
    verificacao();
    while (1){
       
        controle();
        andar();
        temp = (int)(ADC_GetConversion(TEMP)/5.0); //captura a temperatura do motor
        speed = (int)(pulso/(((float)colect_value)*conv_s))*4; // verificar cal velocidade
        position = pulso*pulsos; //posicao em mm
        
        if(EUSART_is_rx_ready()){
            rxByte = EUSART_Read();
            pos_final = (int)(rxByte & 0x03); 
        } 
    }
}
/**
 End of File
*/