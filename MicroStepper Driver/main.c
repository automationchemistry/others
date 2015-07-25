//Bibliotecas
#include  <msp430.h>
#include <stdlib.h> 
#include <string.h>

// Teste Commit
//Definição de constantes (CAIXA ALTA)
#define CW P1OUT |= BIT0                    //Sentido horário
#define CCW P1OUT &= ~BIT0                  //Sentido AntiHorário
#define SLEEP_ON P1OUT &= ~BIT2             //Modo baixo consumo
#define SLEEP_OFF P1OUT |= BIT2             //Modo baixo consumo off    
#define RESET_ON P1OUT &= ~BIT3             //Reset ativado
#define RESET_OFF P1OUT |= BIT3             //Reset desativado
#define ENABLE P1OUT &= ~BIT7                //Habilita driver
#define DISABLE P1OUT |= BIT7               //Habilita driver
#define MS1 BIT6
#define MS2 BIT5
#define MS3 BIT4

//Protótipo de funções
void main(void);              //Programa principal
void ini_config(void);        //Subrotina config. inicial
void config_op(char valor);          //Configura a operação do driver

unsigned int conta = 0;

void main(void)
{
 ini_config();
 
 //P1OUT &= ~MS1; 
 //P1OUT &= ~MS2; 
 //P1OUT &= ~MS3;
 
 //Resolução de 1/16 passos (cf. datasheet)
 P1OUT |= MS1; 
 P1OUT |= MS2; 
 P1OUT |= MS3;
 
 RESET_OFF;
 
 for(;;)
    {    
      
     }  
 
 
}


#pragma vector=TIMER0_A0_VECTOR
__interrupt void clock_timer (void)
{
    P1OUT ^= BIT1;                          //Altera o estado do clock de passo 
  
}


void ini_config(void)
{
WDTCTL = WDTPW + WDTHOLD;         //Desabilita WDT  

// Calibração do DCO para 1MHz
  if (CALBC1_1MHZ == 0xFF || CALDCO_1MHZ == 0xFF)                                     
  {  
    while(1);                 //Se constantes de calibração apagadas
                              //aguardar CPU!!
  } 
  // 1Mhz
  BCSCTL1 = CALBC1_1MHZ;         //Configura intervalo
  DCOCTL = CALDCO_1MHZ;          //Configura DCO + modulação  */

//Configurações das portas
P1DIR = 0xFF;                      //Configura pinos de entrada(L) ou saída(H)
P2DIR = 0xFF;                 

P1REN = 0x00;                      //PullUp/Down
P2REN = 0x00;

P1SEL2 = 0x00;                    //Configura função das portas
P1SEL = 0x00;

P2SEL2 = 0x00;
P2SEL = 0x00;

P1OUT = 0x00;                     //Limpa as saídas

P1IFG = 0x00;
P2IFG = 0x00;

//Configuração - TimerA0
TA0CTL = 0x0210;
TA0CCR0  = 300;               //Configura contador do timer (base de tempo 1us)
TA0CCTL0 = CCIE;              //Habilita a interrupção do timer     
__bis_SR_register(GIE);       //Habilita a interrupção geral

//Configuração do driver motor de passo
DISABLE;
 SLEEP_OFF;
  CCW;
   ENABLE;
   
}          
