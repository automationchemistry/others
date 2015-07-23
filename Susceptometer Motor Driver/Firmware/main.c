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
* P1.0 (2)  - VBAT      (Analog In)                                           *
* P1.1 (3)  - RX        (UART In)                                             *
* P1.2 (4)  - TX        (UART Out)                                            *
* P1.3 (5)  - CONTROL   (Digital Out)                                         *
* P1.4 (6)  - CW/CCW    (Digital Out)                                         *
* P1.5 (7)  - CLOCK     (Digital Out)                                         *
* P1.6 (14) - HALF/FULL (Digital Out)                                         *
* P1.7 (15) - HOME      (Digital In)                                          *
* P2.0 (8)  - START     (Digital In)                                          *
* P2.1 (9)  - ENABLE    (Digital Out)                                         *
*                                                                             *
******************************************************************************/
//Bibliotecas
#include  <msp430.h>
#include <stdlib.h> 
#include <string.h>

//Defini��o de constantes (CAIXA ALTA)
#define ENABLE_DRIVER P2OUT |= BIT1         //Habilita o driver do motor
#define DISABLE_DRIVER P2OUT &= ~BIT1       //Desabilita o driver do motor
#define CW P1OUT |= BIT4                    //Sentido hor�rio
#define CCW P1OUT &= ~BIT4                  //Sentido AntiHor�rio
#define HALF_STEP P1OUT |= BIT6             //Habilita passo completo
#define FULL_STEP P1OUT &= ~BIT6            //Habilita meio passo
#define CHOPPER_INH P1OUT |= BIT3           //PWM linhas INH1/INH2
#define CHOPPER_ABCD P1OUT &= ~BIT3         //PWM linhas ABCD
#define START BIT0                          //Chave de posi��o inicial do processo

//Defini��o de Vari�veis (caixa baixa)
int pos_final=0, pos_inicial=0, i=0,result_conv=0;
unsigned int aux_conv[4],mult_div[4]={1000,100,10,1}, ramp,int_conv, vel_n;
char rx_ch[5], pos[4],vel_conv[4];
float conv_ascii,pos_calc,bateria=0;
unsigned int flag_clock, flag_config=0, flag_rampa=0, flag_vel=0, flag_ccw=0;


//Prot�tipo de fun��es
void main(void);              //Programa principal
void ini_config(void);        //Subrotina config. inicial
void config_uart(void);       //Subrotina config. UART
void rampa(void);             //Gera rampa de acelera��o e desacelera��o
int ascii_int(char *rx_ch);   //Fun��o para converter tipo de dados
void conv_serial(void);       //Fun��o para converver tamanho do dado recebido
void erro_msg(void);          //Gera mensagens de erro
void scan_pos();              //Posiciona inicial do sistema mec�nico

void main(void)
{
//First Call
  ini_config();                //Configura��o inicial
       config_uart();          //Configura a UART
          scan_pos();          //Faz a localiza��o da posi��o inicial
       
   for(;;)
    {    

     }  
}

/*******************************************************************************
*         ROTINA DE TRATAMENTO DA RECEP��O SERIAL                              *
* A subrotina recebe o caractere para controle do sistema:                     *
* Formato <Comando><CR> - onde CR - 0x000D                                     *
* Comandos:                                                                    *
*           <P> - Posi��o absoluta                                             *
*           <V> - Valor da posi��o atual                                       *
*           <B> - Retorna o valor de tens�o da bateria                         *
*******************************************************************************/
#pragma vector=USCIAB0RX_VECTOR
__interrupt void serial_rx(void)
{  
  __bic_SR_register_on_exit(CPUOFF);        // Retira a CPU do modo LPM0
  
    //Verifica string recebida
 rx_ch[i++]=UCA0RXBUF;
  
  if (UCA0RXBUF==0xD)              //Fim da transmiss�o? 0x000D = Carrier Return
  {
     if (rx_ch[0]=='P')            //Caracter 'P' recebido?
       {
        //Converte qualquer sequencia de caracteres recebidos no formato v�lido
        conv_serial();
    
       //Converte ASCII para n�meros inteiros
         pos_final=ascii_int(rx_ch);
   
       //Verifica se houve mudan�a no valor da posi��o, isso evita acionamento desnecess�rio do driver
        if((pos_final-pos_inicial)!=0) 
          {
            ENABLE_DRIVER;              //Habilita o Driver
            __delay_cycles(2000);       //Delay 1ms
            CCTL0|=CCIE;                //Dispara timer
            P1OUT |= BIT5;
            i=0;
          }   
              else
                if ((pos_final-pos_inicial)==0) 
                {       
                  i=0;
                  __bis_SR_register(CPUOFF + GIE);

                }
        }
               
          else     

   if (rx_ch[0]=='V')                             //Caracter V recebido?
      {
        strncpy(pos, vel_conv,4);
        IE2 |= UCA0TXIE;                         //Habilita a transmiss�o serial
        i=0;                                     //Inicia nova recep��o
      }        
     
     else
       
  if (rx_ch[0] == 'B')    //Envia o valor da tens�o da bateria
  {
   ADC10CTL0 |= ENC + ADC10SC;    //Inicia a convers�o
    __delay_cycles(500);          

    bateria=(0.29325513*ADC10MEM);
    ADC10CTL0 &= ~(ENC + ADC10SC);    //Interrompe a convers�o    
    
    //Converte para ASCII
     for (i=0;i<=3;i++)
     {
      conv_ascii=bateria/mult_div[i];      
      pos[i]=(int)conv_ascii+48;
      bateria=(conv_ascii-(int)conv_ascii)*mult_div[i];  
     }
     i=0;
       IE2 |= UCA0TXIE;   
  }
 }
}
/*******************************************************************************
*         FUN��O PARA ENVIAR MENSAGEM DE ERRO VIA SERIAL                       *
*******************************************************************************/
void erro_msg(void)
{
  strcpy(pos,"Erro");                      //Grava a mensagem de erro na variavel
  IE2 |= UCA0TXIE;                         //Habilita a transmiss�o serial
        i=0;                               //Inicia nova recep��o
}
/*******************************************************************************
*         LOCALIZA A POSI��O INICIAL DO CONJUNTO MEC�NICO                      *
*******************************************************************************/
void scan_pos(void)
{
   ENABLE_DRIVER;              //Habilita o Driver
   __delay_cycles(2000);       //Delay 2ms
   CCW;                        //Sentido anti-hor�rio
  
  //Gira o motor indefinidamente at� a chave fim de curso ser pressionada
  while (P2IN & START)
  {
   P1OUT ^= BIT5;
   __delay_cycles(5000);       //Delay 2ms
   P1OUT ^= BIT5;
   __delay_cycles(5000);       //Delay 2ms
   }
  
   DISABLE_DRIVER;              //Desabilita o Driver
     __bis_SR_register(CPUOFF + GIE);
}
/*******************************************************************************
*         FUN��O PARA AJUSTE DOS VALORES RECEBIDOS                             *
* Descri��o: permite que sejam enviados caracteres seriais em qualquer formato.* 
* Exemplo: P0001 = P001 = P01 = P1, para qualquer valor.                       *
*******************************************************************************/
void conv_serial(void)
{
  switch (i)
    {
    case 3: rx_ch[4]=rx_ch[1];
            rx_ch[3]='0';
            rx_ch[2]='0';
            rx_ch[1]='0';
            break;
    case 4: rx_ch[4]=rx_ch[2];
            rx_ch[3]=rx_ch[1];
            rx_ch[2]='0';
            rx_ch[1]='0';
            break;
    case 5: rx_ch[4]=rx_ch[3];
            rx_ch[3]=rx_ch[2];
            rx_ch[2]=rx_ch[1];
            rx_ch[1]='0';
            break;
    }
}
/*******************************************************************************
*         FUN��O PARA CONVERS�O DE ASCII-->INT                                 *
* Descri��o: recebe caracteres no formato ASCII('Char') e converte para valores*
* inteiros de 16 bits (Int)
*******************************************************************************/
int ascii_int(char *rx_ch)
{
  for(i=1;i<=4;i++)
       {//Converte a vari�vel recebido pela serial em inteiro
         aux_conv[i-1]=rx_ch[i];
         aux_conv[i-1]=aux_conv[i-1]-48;   
         aux_conv[i-1]=aux_conv[i-1]*mult_div[i-1];      
                  
        vel_conv[i-1]=rx_ch[i];         //Coloca o valor recebido pela serial na variavel de posi��o atual
       }

  result_conv=aux_conv[0]+aux_conv[1]+aux_conv[2]+aux_conv[3];  //Soma todos os valores convertidos
  
  return result_conv;
}
/*******************************************************************************
*         ROTINA DE TRATAMENTO DA TRANSMISS�O SERIAL                           *
* A subrotina mede o per�odo do sinal obtido pelo sensor e calcula a velocida- *
* de de rota��o em RPM.
*******************************************************************************/
#pragma vector=USCIAB0TX_VECTOR
__interrupt void USCI0TX_ISR(void)
{
  UCA0TXBUF = pos[i++];            //Transmite
  if (i == sizeof pos)             //Trasmiss�o conclu�da?
  {
    IE2 &= ~UCA0TXIE;             //Desabilita transmiss�o
    i=0;
    __bis_SR_register(CPUOFF + GIE);
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
         
     __bis_SR_register(CPUOFF + GIE);
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
  if (CALBC1_1MHZ == 0xFF || CALDCO_1MHZ == 0xFF)                                     
  {  
    while(1);                 //Se constantes de calibra��o apagadas
                              //aguardar CPU!!
  } 
  // 1Mhz
  BCSCTL1 = CALBC1_1MHZ;         //Configura intervalo
  DCOCTL = CALDCO_1MHZ;          //Configura DCO + modula��o  */

//Configura��es das portas
P1DIR = 0x7D;                      //Configura pinos de entrada(L) ou sa�da(H)
P2DIR = 0x02;                 

P1REN = 0x80;                      //PullUp/Down
P2REN = 0x01;

P1OUT |= BIT7;                   //Habilita o Pull UP
P2OUT |= BIT0;                   //Habilita o Pull Up

P1SEL2 = 0x07;                    //Configura fun��o das portas
P1SEL = 0x07;

P2SEL2 = 0x00;
P2SEL = 0x00;

P1OUT = 0x00;                     //Limpa as sa�das

P1IFG = 0x00;
P2IFG = 0x00;

//Configura��o - TimerA0
TACTL = 0x0210;               //Fonte de clock ACLK, Modo de contagem up mode
CCR0 = 1990;                  //Per�odo do PWM

//Configura��o do ADC
// Fonte de clock atrav�s do oscilador ADC
// Fator de divis�o = 1 --> Freq. do oscilador 5MHz
// Fonte de disparo via oscilador ADC
// Tempo de amostragem - 16 ciclos
// Refer�ncia de tens�o Vcc
// Velocidade de convers�o = 50kbps
ADC10CTL0 = 0x0800;           
ADC10CTL1 = INCH_0;             //Seleciona ADC Canal A0
ADC10AE0 = 0x01;                //Liga o conversor e habilita o canal 1
ADC10CTL0 |= BIT4;
           
__bis_SR_register(GIE);       //Habilita a interrup��o geral

//Configura��o do driver motor de passo
DISABLE_DRIVER;
 FULL_STEP;
  CHOPPER_ABCD;
}                           
   
