/******************************************************************************
*  CENTRO FEDERAL DE EDUCA�AO TECNOL�GICA DE MINAS GERAIS                     *
*  Departamento de Qu�mica - /07/2014                                         *
*  Sistema de comunica��o e posicionamento motor de passo                     *
*                                                                             *
******************************************************************************/

/******************************************************************************
* Compilador IAR Embedded Workbench IDE - 5.40.6                              *
* Dispositivo MSP430G2553                                                     *
* Descri��o: Possibilitar o controle do posicionamento de uma carga  mec�nica *
*            acionada por motor de passo na configura��o bipolar s�rie via    *
*            comando serial (RS232).                                          *
*            As rotinas do programa tem a capacidade de interpretar o valor   *
*            absoluto da refer�ncia, traduzindo em n�mero de passos e sentido *
*            de rota��o.                                                      *
*                                                                             *
* PINOS UTILIZADOS                                                            *
* P1.0 (2)  - ENABLE    (Digital Out)                                         *
* P1.1 (3)  - RX        (UART In)                                             *
* P1.2 (4)  - TX        (UART Out)                                            *
* P1.3 (5)  - CONTROL   (Digital Out)                                         *
* P1.4 (6)  - CW/CCW    (Digital Out)                                         *
* P1.5 (7)  - CLOCK     (Digital Out)                                         *
* P1.6 (14) - HALF/FULL (Digital Out)                                         *
* P1.7 (15) - HOME      (Digital In)                                          *
* P2.0 (8)  - START     (Digital In)                                          *
*                                                                             *
******************************************************************************/
//Bibliotecas
#include  <msp430.h>
#include <stdlib.h> 

//Defini��o de constantes (CAIXA ALTA)
#define ENABLE_DRIVER P1OUT |= BIT0         //Habilita o driver do motor
#define DISABLE_DRIVER P1OUT &= ~BIT0       //Desabilita o driver do motor
#define CW P1OUT |= BIT4                    //Sentido hor�rio
#define CCW P1OUT &= ~BIT4                  //Sentido AntiHor�rio
#define HALF_STEP P1OUT |= BIT6             //Habilita passo completo
#define FULL_STEP P1OUT &= ~BIT6            //Habilita meio passo
#define CHOPPER_INH P1OUT |= BIT3           //PWM linhas INH1/INH2
#define CHOPPER_ABCD P1OUT &= ~BIT3         //PWM linhas ABCD
#define START BIT1                          //Chave de posi��o inicial do processo

//Defini��o de Vari�veis (caixa baixa)
int flag_clock, pos_final=0, pos_inicial=0, i=0;
unsigned int aux_conv[4],mult_div[4]={1000,100,10,1}, ramp;
char rx_ch[5];

//Prot�tipo de fun��es
void main(void);              //Programa principal
void ini_config(void);        //Subrotina config. inicial
void config_uart(void);       //Subrotina config. UART
void rampa(void);

void main(void)
{
//First Call
  ini_config();                //Configura��o inicial
       config_uart();          //Configura a UART
     
   for(;;)
    {
      //Loop
     }  
}

/*******************************************************************************
*         ROTINA DE TRATAMENTO DA RECEP��O SERIAL                              *
* A subrotina recebe o caractere para controle do sistema:                     *
* Formato <Comando><CR> - onde CR - 0x000D                                     *
* Comandos:                                                                    *
*           <P> - Posi��o absoluta                                             *
*                                                                              *
*******************************************************************************/
#pragma vector=USCIAB0RX_VECTOR
__interrupt void serial_rx(void)
{  
  //Verifica string recebida
 rx_ch[i++]=UCA0RXBUF;
  
  if (UCA0RXBUF==0xD)               //Fim da transmiss�o? 0x000D
  {
    i=0;

  if (rx_ch[0]=='P')
  {
   //Caracter 'P' recebido?
  for(i=1;i<=4;i++)
       {//Converte a vari�vel recebido pela serial em inteiro
         aux_conv[i-1]=rx_ch[i];
         aux_conv[i-1]=aux_conv[i-1]-48;   

         aux_conv[i-1]=aux_conv[i-1]*mult_div[i-1];  
       }
      pos_final=aux_conv[0]+aux_conv[1]+aux_conv[2]+aux_conv[3];
  
  if((pos_final-pos_inicial)!=0) 
  {
    ENABLE_DRIVER;              //Habilita o Driver
    __delay_cycles(2000);       //Delay 1ms
        CCTL0|=CCIE;            //Dispara timer
        P1OUT |= BIT5;          
  }   
     } 
    i=0;     
     }
}

/*******************************************************************************
*            ROTINA DE TRATAMENTO DA INTERRUP��O DO TIMER0                     *
* A rotina compara a posi��o atual do motor com a refer�ncia, estabelecendo    *
* o sentido de rota��o correspondente para alcan�ar o valor do deslocamento.   *
*******************************************************************************/
#pragma vector=TIMER0_A0_VECTOR
__interrupt void clock_timer (void)
{//Desabilita a interrup��o da serial at� a conclus�o do deslocamento
  IE2 &= ~UCA0RXIE; 
    
  rampa();
  
  
  if ((pos_final-pos_inicial)>0)            //Verfica posi��o relativa
  {
    CW;                                     //Sentido hor�rio
    if (!(P1OUT&0x0020))                    //Testa a transi��o de subida do sinal de clock
    {
    pos_inicial++;                          
    }
  P1OUT ^= BIT5;                            //Altera o estado do clock de passo
  }
  else
    if((pos_final-pos_inicial)<0)          //Verfica posi��o relativa
  {
    CCW;                                   //Sentido anti-hor�rio
    if (!(P1OUT&0x0020))                   //Testa a transi��o de subida do sinal de clock
    {
    pos_inicial--;
    }
  P1OUT ^= BIT5;                          //Altera o estado do clock de passo
  }
  else
    if(!(pos_final-pos_inicial))         //Verifica final do deslocamento
    { 
     DISABLE_DRIVER;                     //Desabilita o driver
     IE2 |= UCA0RXIE;                    //Habilita a interrup��o serial
     CCTL0 &=~CCIE;                      //Desabilita interrup��o do TIMER
     ramp=0;
     CCR0=1990;
    }
}
/*******************************************************************************
*                                                                              *
*******************************************************************************/
void rampa(void)
{
  //Rampa de acelera��o
  if (CCR0>330)
  {
    CCR0=CCR0-20;
  }
  
//Rampa de desacelera��o
  if (abs((pos_final-pos_inicial))<=83)
  {
    CCR0=CCR0+20;
  }
  
}



/*******************************************************************************
*         PROCEDIMENTO PARA CONFIGURA��O DA COMUNICA��O SERIAL                 *
*******************************************************************************/
void config_uart(void)
{
UCA0CTL1 |= UCSSEL_2;               //Fonte de clock SMCLK --> DCO
UCA0BR0 = 104;                      //1MHz/9600bauds - 104ms
UCA0BR1 = 0;                        
UCA0MCTL = UCBRS0;                  //Modula��o UCBRSx =  1
UCA0CTL1 &= ~UCSWRST;               //Inicializa a m�quina de estado USCI
IE2 |= UCA0RXIE;                    //Habilita a interru��o de recep��o 
}

/*******************************************************************************
*         PROCEDIMENTO PARA DEFINIR AS CONFIGURA��ES INICIAIS DO SISTEMA       *
*******************************************************************************/
void ini_config(void)
{
WDTCTL = WDTPW + WDTHOLD;         //Desabilita WDT  

// Calibra��o do DCO para 1MHz
  if (CALBC1_1MHZ ==0xFF || CALDCO_1MHZ == 0xFF)                                     
  {  
    while(1);                 //Se constantes de calibra��o apagadas
                              //aguardar CPU!!
  } 
  // 1Mhz
  BCSCTL1 = CALBC1_1MHZ;         //Configura intervalo
  DCOCTL = CALDCO_1MHZ;          //Configura DCO + modula��o  */

//Configura��es das portas
P1DIR=0x7D;                      //Configura pinos de entrada(L) ou sa�da(H)
P2DIR=0x01;                 

P1REN=0x80;                      //PullUp/Down
P2REN=0x01;

P1OUT |= BIT7;                   //Habilita o Pull UP
P2OUT &= BIT0;                   //Habilita o Pull Down

P1SEL2=0x06;                    //Configura fun��o das portas
P1SEL=0x06;
P2SEL2=0x00;
P2SEL=0x00;

P1OUT=0x00;                     //Limpa as sa�das
P2OUT=0x00;

P1IFG=0x00;
P2IFG=0x00;

//Configura��o - TimerA0
TACTL = 0x0210;               //Fonte de clock ACLK, Modo de contagem up mode
CCR0 = 1990;                  //Per�odo do PWM

__bis_SR_register(GIE);       //Habilita a interrup��o geral

//Configura��o do driver motor de passo

DISABLE_DRIVER;
FULL_STEP;
CHOPPER_ABCD;

}                           
   
