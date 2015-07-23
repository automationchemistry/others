/******************************************************************************
*  CENTRO FEDERAL DE EDUCAÇAO TECNOLÓGICA DE MINAS GERAIS                     *
*  Departamento de Química - /07/2014                                         *
*  Sistema de comunicação e posicionamento motor de passo                     *
*                                                                             *
******************************************************************************/

/******************************************************************************
* Compilador IAR Embedded Workbench IDE - 5.40.6                              *
* Dispositivo MSP430G2553                                                     *
* Descrição: Possibilitar o controle do posicionamento de uma carga  mecânica *
*            acionada por motor de passo na configuração bipolar série via    *
*            comando serial (RS232).                                          *
*            As rotinas do programa tem a capacidade de interpretar o valor   *
*            absoluto da referência, traduzindo em número de passos e sentido *
*            de rotação.                                                      *
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

//Definição de constantes (CAIXA ALTA)
#define ENABLE_DRIVER P1OUT |= BIT0         //Habilita o driver do motor
#define DISABLE_DRIVER P1OUT &= ~BIT0       //Desabilita o driver do motor
#define CW P1OUT |= BIT4                    //Sentido horário
#define CCW P1OUT &= ~BIT4                  //Sentido AntiHorário
#define HALF_STEP P1OUT |= BIT6             //Habilita passo completo
#define FULL_STEP P1OUT &= ~BIT6            //Habilita meio passo
#define CHOPPER_INH P1OUT |= BIT3           //PWM linhas INH1/INH2
#define CHOPPER_ABCD P1OUT &= ~BIT3         //PWM linhas ABCD
#define START BIT1                          //Chave de posição inicial do processo

//Definição de Variáveis (caixa baixa)
int flag_clock, pos_final=0, pos_inicial=0, i=0;
unsigned int aux_conv[4],mult_div[4]={1000,100,10,1}, ramp;
char rx_ch[5];

//Protótipo de funções
void main(void);              //Programa principal
void ini_config(void);        //Subrotina config. inicial
void config_uart(void);       //Subrotina config. UART
void rampa(void);

void main(void)
{
//First Call
  ini_config();                //Configuração inicial
       config_uart();          //Configura a UART
     
   for(;;)
    {
      //Loop
     }  
}

/*******************************************************************************
*         ROTINA DE TRATAMENTO DA RECEPÇÃO SERIAL                              *
* A subrotina recebe o caractere para controle do sistema:                     *
* Formato <Comando><CR> - onde CR - 0x000D                                     *
* Comandos:                                                                    *
*           <P> - Posição absoluta                                             *
*                                                                              *
*******************************************************************************/
#pragma vector=USCIAB0RX_VECTOR
__interrupt void serial_rx(void)
{  
  //Verifica string recebida
 rx_ch[i++]=UCA0RXBUF;
  
  if (UCA0RXBUF==0xD)               //Fim da transmissão? 0x000D
  {
    i=0;

  if (rx_ch[0]=='P')
  {
   //Caracter 'P' recebido?
  for(i=1;i<=4;i++)
       {//Converte a variável recebido pela serial em inteiro
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
*            ROTINA DE TRATAMENTO DA INTERRUPÇÃO DO TIMER0                     *
* A rotina compara a posição atual do motor com a referência, estabelecendo    *
* o sentido de rotação correspondente para alcançar o valor do deslocamento.   *
*******************************************************************************/
#pragma vector=TIMER0_A0_VECTOR
__interrupt void clock_timer (void)
{//Desabilita a interrupção da serial até a conclusão do deslocamento
  IE2 &= ~UCA0RXIE; 
    
  rampa();
  
  
  if ((pos_final-pos_inicial)>0)            //Verfica posição relativa
  {
    CW;                                     //Sentido horário
    if (!(P1OUT&0x0020))                    //Testa a transição de subida do sinal de clock
    {
    pos_inicial++;                          
    }
  P1OUT ^= BIT5;                            //Altera o estado do clock de passo
  }
  else
    if((pos_final-pos_inicial)<0)          //Verfica posição relativa
  {
    CCW;                                   //Sentido anti-horário
    if (!(P1OUT&0x0020))                   //Testa a transição de subida do sinal de clock
    {
    pos_inicial--;
    }
  P1OUT ^= BIT5;                          //Altera o estado do clock de passo
  }
  else
    if(!(pos_final-pos_inicial))         //Verifica final do deslocamento
    { 
     DISABLE_DRIVER;                     //Desabilita o driver
     IE2 |= UCA0RXIE;                    //Habilita a interrupção serial
     CCTL0 &=~CCIE;                      //Desabilita interrupção do TIMER
     ramp=0;
     CCR0=1990;
    }
}
/*******************************************************************************
*                                                                              *
*******************************************************************************/
void rampa(void)
{
  //Rampa de aceleração
  if (CCR0>330)
  {
    CCR0=CCR0-20;
  }
  
//Rampa de desaceleração
  if (abs((pos_final-pos_inicial))<=83)
  {
    CCR0=CCR0+20;
  }
  
}



/*******************************************************************************
*         PROCEDIMENTO PARA CONFIGURAÇÃO DA COMUNICAÇÃO SERIAL                 *
*******************************************************************************/
void config_uart(void)
{
UCA0CTL1 |= UCSSEL_2;               //Fonte de clock SMCLK --> DCO
UCA0BR0 = 104;                      //1MHz/9600bauds - 104ms
UCA0BR1 = 0;                        
UCA0MCTL = UCBRS0;                  //Modulação UCBRSx =  1
UCA0CTL1 &= ~UCSWRST;               //Inicializa a máquina de estado USCI
IE2 |= UCA0RXIE;                    //Habilita a interrução de recepção 
}

/*******************************************************************************
*         PROCEDIMENTO PARA DEFINIR AS CONFIGURAÇÕES INICIAIS DO SISTEMA       *
*******************************************************************************/
void ini_config(void)
{
WDTCTL = WDTPW + WDTHOLD;         //Desabilita WDT  

// Calibração do DCO para 1MHz
  if (CALBC1_1MHZ ==0xFF || CALDCO_1MHZ == 0xFF)                                     
  {  
    while(1);                 //Se constantes de calibração apagadas
                              //aguardar CPU!!
  } 
  // 1Mhz
  BCSCTL1 = CALBC1_1MHZ;         //Configura intervalo
  DCOCTL = CALDCO_1MHZ;          //Configura DCO + modulação  */

//Configurações das portas
P1DIR=0x7D;                      //Configura pinos de entrada(L) ou saída(H)
P2DIR=0x01;                 

P1REN=0x80;                      //PullUp/Down
P2REN=0x01;

P1OUT |= BIT7;                   //Habilita o Pull UP
P2OUT &= BIT0;                   //Habilita o Pull Down

P1SEL2=0x06;                    //Configura função das portas
P1SEL=0x06;
P2SEL2=0x00;
P2SEL=0x00;

P1OUT=0x00;                     //Limpa as saídas
P2OUT=0x00;

P1IFG=0x00;
P2IFG=0x00;

//Configuração - TimerA0
TACTL = 0x0210;               //Fonte de clock ACLK, Modo de contagem up mode
CCR0 = 1990;                  //Período do PWM

__bis_SR_register(GIE);       //Habilita a interrupção geral

//Configuração do driver motor de passo

DISABLE_DRIVER;
FULL_STEP;
CHOPPER_ABCD;

}                           
   
