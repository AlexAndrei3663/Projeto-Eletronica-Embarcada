/*******************************************************************************
**                 UNIVERSIDADE DE BRASILIA - UNB                             **
** DISCIPLINA: FGA0096 - ELETRONICA EMBARCADA        TURMA:                   **
** PROFESSOR: GUILLERMO ALVAREZ BESTARD , DR. ENG. MECATRONICA                **
** ALUNO: FERNANDA DINIZ                           MATRICULA:19/0087099       **
**        NATHAN                                   MATRICULA:00/0000000       **
**        THALLES                                  MATRICULA:18/0028189       **
**        GUILHERME CARBALLAL OLIVEIRA SANTOS      MATRICULA:16/0050421       **
**               TRABALHO FINAL ELETRONICA EMBARCADA                          **
*******************************************************************************/

#include "mcc_generated_files/mcc.h"
#include "ledMatrix.h"

//constantes
#define pulso       .83       // 180mm/215pulso
#define conv_s      .000002  //para converter ms para s no calculo de speed
#define duty_pwm    510      // Se o PWM que define o ciclo util é de 10 bits, seu valor vai de 0(0%) a 1023(100%), como vamos manter fixo em 40%, o valor do ciclo util é 409.2, vamos usar 409 para aproximar


//variaveis
float position;
int temp =0,
    speed = 0,          // Vel 
    motor = 0,
    count = 10,
    pulsos = 0,   
    pos_final = 0,  // recebe da serial um valor de 0-3
    pos_atual = 0,    
    u ; 
//bool templuzflag;
uint16_t  colect_value = 0,     // valor coletado para calculo de velocidade
          BCD,                  // talvez nao necessario e para convesao multiplos digitos (sendo usada na funcao andar)
          pwm=0;                // Ciclo Util para o PWM 
uint8_t LED[8],                 // Vetor para armazenar os valores da matriz de led
        rxByte,                 // Vetor para armazenar os valores recebidos pelo módulo USART
        txByte[4];              // Vetor para armazenar os valores transmitidos para o módulo USART
bool    enviarDados=false;      // Flag para enviar os dados pelo módulo USART
int         iRx=0;              // Indice para dados recebidos pelo módulo USART



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
 *  MEMORY   :
 *  MSSP1    : Display led - falta testar
 *  TMR1     : nao foi setado usado para o Encoder
 *  TMR2     : nao foi setado usado para PWM
 *  TMR4     : TIMER DE INTERRUPCAO PARA COMUNICACAO 100mS POST 1:4 PRE 1:64 (99.84mS)
 */


// para comunicacao serial (quase pronta e atualizada)


// passa a info para o display de leds 8X8
void leds(void){
    for (int i =1;i<8;i++)
    {
        txMAX7219(i, LED[8-i],i, LED[8-i]); //usado para 2 display leds so teremos um talvez tenha que realziar mudancas no 
                                              // arquivo ledmatrix.c
    }
}

// Troca de informações via Serial com o PC
void comm (void){
       
    // TX
    txByte[0] = 0xBF & ((0b10000000) | ((uint8_t)motor<<4) | (pos_atual & 0x03)); //1011 1111 AND 00XX 0000 AND (Pos_atual AND 0000 0011) -- Na duvida, fazer atualizacoes sequenciais
    txByte[1] = 0x7F & (((uint8_t)position)>>1);        //0111 1111 AND variável
    txByte[2] = 0x7F & (((uint8_t)speed)<<2);           //0111 1111 AND variável
    txByte[3] = 0x7F & ((temp)<<1);                     //0111 1111 AND variável
    // RX
    if(EUSART_is_rx_ready()){
        rxByte = EUSART_Read();
        pos_final = (int)(rxByte & 0x03); 

    } 
    for (int i=0;i<4;i++){
      EUSART_Write(txByte[i]);
    }
    leds(); //carrega a matrix de leds
}

void andar(void){
    u = pos_atual;
    for (int i =0;i<4;i++)
    {
        LED[i]= DATAEE_ReadByte(i+((1+u)*4));
    }
    // carrega o estado do motor 
    // 0 parado         -
    // 1 ascendente     >
    // 2 descendente    <
    for (int i =0;i<4;i++)
    {
        LED[4+i]= DATAEE_ReadByte(i+((7+motor)*4));
    }
}

void deslocamento(){
    
    // indicando o incio do movimento do motor
    while(pos_final!=pos_atual){
        
        PWM3_LoadDutyValue(duty_pwm);
    }
    
    PWM3_LoadDutyValue(0);
    motor = 0;
    __delay_ms(2000); // 2 segundos para permitir o fluxo de pessoas (pode ser implementado diferente depois)
}


 //controle do motor
void controle(){    
    //Dir = porta RA7   
    //Dir = 1 movimento ascendente 
    //Dir=0 movimento decendente                          
    if(pos_final>pos_atual){
        if(Dir_GetValue()==0){    
            PWM3_LoadDutyValue(0);
            __delay_ms(500);
        }
        //subida
        Dir_SetHigh();
        motor = 1;
        deslocamento();
         
    } else if(pos_final<pos_atual){
        if(Dir_GetValue()==1){
            PWM3_LoadDutyValue(0);
            __delay_ms(500);
        }
        //descida
        Dir_SetLow();
        motor = 2; 
        deslocamento();
    } else if(pos_final==pos_atual){
        motor = 0;
        
    } else{    // indica um erro no sistema caso imposivel
        PWM3_LoadDutyValue(0);
    }
}

//Veriricar se esta correto
void Enc(uint16_t colect_value){
    
    TMR1_WriteTimer(0);
    
    if(motor==2 && pulsos >= 0){
        pulsos--; 
    }
    else if(motor==1 && pulsos <= 215){
        pulsos++;
    }
    else{    //zera o pwm e nao altera o valor dos pulsos (caso de erro)
        pulsos = 0;
        position = 0;
        speed = 0;
        PWM3_LoadDutyValue(0);
    }
}


//Funcoes dos sensores hall
void S1(){
	
	pos_atual = 0; //Recebe o valor 0 mas envia posicao com +1 na comm
	pulsos = 0;    //Valor inicial dos pulsos
    PWM3_LoadDutyValue(0); // desligamento do motor
    Dir_SetHigh(); // juntamente com alteracao da direcao do motor para evitar que ele tente descer mais ambos entrariam como boa pratica
	
}

void S2(){
	
	pos_atual = 1; //Recebe o valor 1 mas envia posicao com +1 na comm
	
}

void S3(){
	
	pos_atual = 2; //Recebe o valor 2 mas envia posicao com +1 na comm
	
}

void CMP1_ISR(void)
{
    S3();
    // clear the CMP1 interrupt flag
    PIR2bits.C1IF = 0;
}

void S4(){
	
	pos_atual = 3; //Recebe o valor 3 mas envia posicao com +1 na comm
	pulsos = 215;  //Valor maximo de pulsos 
    PWM3_LoadDutyValue(0); // desligamento do motor
    Dir_SetLow(); // evita que ele suba mais que deveria fazendo ele descer mesmo que algo de errado
    
}

void CMP2_ISR(void)
{
    S4();
    // clear the CMP1 interrupt flag
    PIR2bits.C1IF = 0;
}

void main(void)
{
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
	
	//Conferir as interrupcoes dos sensores (configuracoes dos pinos) parece ok
	IOCBF0_SetInterruptHandler(S1); //Confirmar se é nessa interrupção
    IOCBF3_SetInterruptHandler(S2); //Confirmar se é nessa interrupção
    PWM3_LoadDutyValue(500);
    while (1){        
        position = pulso*pulsos; //posicao em mm
        andar();
        controle();
        temp = (ADC_GetConversion(TEMP)*0.48); //captura a temperatura do motor
        speed = (int)(pulso/(((float)colect_value)*conv_s))*4; // verificar cal velocidade
    
    }
}
/**
 End of File
*/