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

//Definição de constantes (CAIXA ALTA)
#define ENABLE_DRIVER P2OUT |= BIT1         //Habilita o driver do motor
#define DISABLE_DRIVER P2OUT &= ~BIT1       //Desabilita o driver do motor
#define CW P1OUT |= BIT4                    //Sentido horário
#define CCW P1OUT &= ~BIT4                  //Sentido AntiHorário
#define HALF_STEP P1OUT |= BIT6             //Habilita passo completo
#define FULL_STEP P1OUT &= ~BIT6            //Habilita meio passo
#define CHOPPER_INH P1OUT |= BIT3           //PWM linhas INH1/INH2
#define CHOPPER_ABCD P1OUT &= ~BIT3         //PWM linhas ABCD
#define START BIT0                          //Chave de posição inicial do processo

//Definição de Variáveis (caixa baixa)
int pos_final=0, pos_inicial=0, i=0,result_conv=0;
unsigned int aux_conv[4],mult_div[4]={1000,100,10,1}, ramp,int_conv, vel_n;
char rx_ch[5], pos[4],vel_conv[4];
float conv_ascii,pos_calc,bateria=0;
unsigned int flag_clock, flag_config=0, flag_rampa=0, flag_vel=0, flag_ccw=0;


//Protótipo de funções
void main(void);              //Programa principal
void ini_config(void);        //Subrotina config. inicial
void config_uart(void);       //Subrotina config. UART
void rampa(void);             //Gera rampa de aceleração e desaceleração
int ascii_int(char *rx_ch);   //Função para converter tipo de dados
void conv_serial(void);       //Função para converver tamanho do dado recebido
void erro_msg(void);          //Gera mensagens de erro
void scan_pos();              //Posiciona inicial do sistema mecânico

void main(void)
{
//First Call
  ini_config();                //Configuração inicial
       config_uart();          //Configura a UART
          scan_pos();          //Faz a localização da posição inicial
       
   for(;;)
    {    

     }  
}

/*******************************************************************************
*         ROTINA DE TRATAMENTO DA RECEPÇÃO SERIAL                              *
* A subrotina recebe o caractere para controle do sistema:                     *
* Formato <Comando><CR> - onde CR - 0x000D                                     *
* Comandos:                                                                    *
*           <P> - Posição absoluta                                             *
*           <V> - Valor da posição atual                                       *
*           <B> - Retorna o valor de tensão da bateria                         *
*******************************************************************************/
#pragma vector=USCIAB0RX_VECTOR
__interrupt void serial_rx(void)
{  
  __bic_SR_register_on_exit(CPUOFF);        // Retira a CPU do modo LPM0
  
    //Verifica string recebida
 rx_ch[i++]=UCA0RXBUF;
  
  if (UCA0RXBUF==0xD)              //Fim da transmissão? 0x000D = Carrier Return
  {
     if (rx_ch[0]=='P')            //Caracter 'P' recebido?
       {
        //Converte qualquer sequencia de caracteres recebidos no formato válido
        conv_serial();
    
       //Converte ASCII para números inteiros
         pos_final=ascii_int(rx_ch);
   
       //Verifica se houve mudança no valor da posição, isso evita acionamento desnecessário do driver
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
        IE2 |= UCA0TXIE;                         //Habilita a transmissão serial
        i=0;                                     //Inicia nova recepção
      }        
     
     else
       
  if (rx_ch[0] == 'B')    //Envia o valor da tensão da bateria
  {
   ADC10CTL0 |= ENC + ADC10SC;    //Inicia a conversão
    __delay_cycles(500);          

    bateria=(0.29325513*ADC10MEM);
    ADC10CTL0 &= ~(ENC + ADC10SC);    //Interrompe a conversão    
    
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
*         FUNÇÃO PARA ENVIAR MENSAGEM DE ERRO VIA SERIAL                       *
*******************************************************************************/
void erro_msg(void)
{
  strcpy(pos,"Erro");                      //Grava a mensagem de erro na variavel
  IE2 |= UCA0TXIE;                         //Habilita a transmissão serial
        i=0;                               //Inicia nova recepção
}
/*******************************************************************************
*         LOCALIZA A POSIÇÃO INICIAL DO CONJUNTO MECÂNICO                      *
*******************************************************************************/
void scan_pos(void)
{
   ENABLE_DRIVER;              //Habilita o Driver
   __delay_cycles(2000);       //Delay 2ms
   CCW;                        //Sentido anti-horário
  
  //Gira o motor indefinidamente até a chave fim de curso ser pressionada
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
*         FUNÇÃO PARA AJUSTE DOS VALORES RECEBIDOS                             *
* Descrição: permite que sejam enviados caracteres seriais em qualquer formato.* 
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
*         FUNÇÃO PARA CONVERSÃO DE ASCII-->INT                                 *
* Descrição: recebe caracteres no formato ASCII('Char') e converte para valores*
* inteiros de 16 bits (Int)
*******************************************************************************/
int ascii_int(char *rx_ch)
{
  for(i=1;i<=4;i++)
       {//Converte a variável recebido pela serial em inteiro
         aux_conv[i-1]=rx_ch[i];
         aux_conv[i-1]=aux_conv[i-1]-48;   
         aux_conv[i-1]=aux_conv[i-1]*mult_div[i-1];      
                  
        vel_conv[i-1]=rx_ch[i];         //Coloca o valor recebido pela serial na variavel de posição atual
       }

  result_conv=aux_conv[0]+aux_conv[1]+aux_conv[2]+aux_conv[3];  //Soma todos os valores convertidos
  
  return result_conv;
}
/*******************************************************************************
*         ROTINA DE TRATAMENTO DA TRANSMISSÃO SERIAL                           *
* A subrotina mede o período do sinal obtido pelo sensor e calcula a velocida- *
* de de rotação em RPM.
*******************************************************************************/
#pragma vector=USCIAB0TX_VECTOR
__interrupt void USCI0TX_ISR(void)
{
  UCA0TXBUF = pos[i++];            //Transmite
  if (i == sizeof pos)             //Trasmissão concluída?
  {
    IE2 &= ~UCA0TXIE;             //Desabilita transmissão
    i=0;
    __bis_SR_register(CPUOFF + GIE);
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
         
     __bis_SR_register(CPUOFF + GIE);
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
  if (CALBC1_1MHZ == 0xFF || CALDCO_1MHZ == 0xFF)                                     
  {  
    while(1);                 //Se constantes de calibração apagadas
                              //aguardar CPU!!
  } 
  // 1Mhz
  BCSCTL1 = CALBC1_1MHZ;         //Configura intervalo
  DCOCTL = CALDCO_1MHZ;          //Configura DCO + modulação  */

//Configurações das portas
P1DIR = 0x7D;                      //Configura pinos de entrada(L) ou saída(H)
P2DIR = 0x02;                 

P1REN = 0x80;                      //PullUp/Down
P2REN = 0x01;

P1OUT |= BIT7;                   //Habilita o Pull UP
P2OUT |= BIT0;                   //Habilita o Pull Up

P1SEL2 = 0x07;                    //Configura função das portas
P1SEL = 0x07;

P2SEL2 = 0x00;
P2SEL = 0x00;

P1OUT = 0x00;                     //Limpa as saídas

P1IFG = 0x00;
P2IFG = 0x00;

//Configuração - TimerA0
TACTL = 0x0210;               //Fonte de clock ACLK, Modo de contagem up mode
CCR0 = 1990;                  //Período do PWM

//Configuração do ADC
// Fonte de clock através do oscilador ADC
// Fator de divisão = 1 --> Freq. do oscilador 5MHz
// Fonte de disparo via oscilador ADC
// Tempo de amostragem - 16 ciclos
// Referência de tensão Vcc
// Velocidade de conversão = 50kbps
ADC10CTL0 = 0x0800;           
ADC10CTL1 = INCH_0;             //Seleciona ADC Canal A0
ADC10AE0 = 0x01;                //Liga o conversor e habilita o canal 1
ADC10CTL0 |= BIT4;
           
__bis_SR_register(GIE);       //Habilita a interrupção geral

//Configuração do driver motor de passo
DISABLE_DRIVER;
 FULL_STEP;
  CHOPPER_ABCD;
}                           
   
