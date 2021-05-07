/* 
 * File:   mainP2.c
 * Autor: Jose Luis Alvarez Pineda (19392)
 *
 * Creado el 6 de Mayo de 2021.
 * Ultima edicion: 6 de Mayo de 2021.
 */

/*------------------------------------------------------------------------------
 *                       Palabras de configuracion y librerias
 -----------------------------------------------------------------------------*/
// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits 
//(INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, 
//I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and 
//can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = ON       // RE3/MCLR pin function select bit (
//RE3/MCLR pin function is MCLR)
#pragma config CP = OFF         // Code Protection bit 
//(Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit 
//(Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits 
//(BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit 
//(Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit 
//(Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit 
//(RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit 
//(Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory 
//Self Write Enable bits (Write protection off)


#include <xc.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#define _XTAL_FREQ 8000000 // Frecuencia de reloj
/*------------------------------------------------------------------------------
 *                                  Variables
 -----------------------------------------------------------------------------*/
union BANDERAS{ //para manejar grupos de bits
    struct{
        unsigned bit0: 5; //es de 5 bits-para temporizar los cambios
        unsigned bit1: 1; // es de 1 bit-indicar el cambio y lectura
        unsigned modo: 1; // es de 1 bit-cambio de modo
    };
}SERVOS;

uint8_t POT1,POT2,POT3,POT4,EXTREC;    //para manejar los 4 potenciometros
/*------------------------------------------------------------------------------
 *                                  Prototipos
 -----------------------------------------------------------------------------*/
void configuraciones(void); //rutina de configuracion
void servos(void);          //manejo de los servos
void AnalogReadServo(void); //para leer los pots para los servos
/*------------------------------------------------------------------------------
 *                                  Interrupcion
 -----------------------------------------------------------------------------*/
void __interrupt() rutInter(void){

    if(INTCONbits.TMR0IF) {
        SERVOS.bit0++;
        SERVOS.bit1 = 1;
        INTCONbits.TMR0IF = 0;
        if(SERVOS.modo) ADCON0bits.GO = 1;
    }
    
    if(INTCONbits.RBIF && !PORTBbits.RB0){
        SERVOS.modo = ~SERVOS.modo;
        INTCONbits.RBIF = 0;
    }
    INTCONbits.RBIF = 0;
    
    if(PIR1bits.RCIF){
        EXTREC = RCREG; 
    }
    
}

/*------------------------------------------------------------------------------
 *                                Loop principal
 -----------------------------------------------------------------------------*/
void main(void) {
    configuraciones();
    while(1){
        if(SERVOS.modo){
            AnalogReadServo();
            PORTBbits.RB7 = 1;
        }
        else{ PORTBbits.RB7 = 0; POT1 = EXTREC;}
        servos();
    }
}

/*------------------------------------------------------------------------------
 *                           Funciones utilizadas
 -----------------------------------------------------------------------------*/
void configuraciones(void){
    //Configuracion de puertos
    ANSEL =         0X0F;//4 entradas analogicas, las demas I/O
    ANSELH =        0X00;
    TRISA =         0X0F;
    TRISB =         0X01;
    TRISC =         0X80;
    TRISD =         0X00;
    TRISE =         0X00;
    PORTA =         0X00;
    PORTB =         0X00;
    PORTC =         0X00;
    PORTD =         0X00;
    PORTE =         0X00;
    
    //Configuracion del reloj
    OSCCONbits.IRCF = 0b111; //oscilador a 8Mhz
    OSCCONbits.SCS = 0b1;
    
    //Configuracion de interrupciones
    INTCONbits.TMR0IF =     0;
    INTCONbits.TMR0IE =     1;
    INTCONbits.RBIF =       0;
    INTCONbits.RBIE =       0;
    INTCONbits.PEIE =       1;
    PIE1bits.RCIE   =       1; //permite interrupciones de recepcion de datos
    INTCONbits.GIE =        1;         //habilita interrupciones
    
    //Configuracion de cambio de modo
    IOCBbits.IOCB0 = 1;     //detecta cambios en RB0
    
    //Configutacion del ADC
    ADCON0bits.ADCS =   0b10;
    ADCON0bits.CHS =    0b0000;  //comience leyendo en el 0
    ADCON0bits.GO =     0b0;
    ADCON0bits.ADON =   0b1;
    ADCON1bits.ADFM =   0b0; //justificado a la IZQUIERDA
    ADCON1bits.VCFG1 =  0b0; //referencias a alimentacion del PIC
    ADCON1bits.VCFG0 =  0b0;
    
    //Configuracion del EUSART
    SPBRG =                 12;      //12 para un baud rate de 9615
    TXSTAbits.BRGH =        0;      //baja velocidad por el reloj
    TXSTAbits.TXEN =        1;      //habilitar transmision
    RCSTAbits.CREN =        1;      //habilita la recepcion
    TXSTAbits.SYNC =        0;      //modo asincrono
    RCSTAbits.SPEN =        1;      //configura los pines como seriales
    
    //Configuracion del timmer 0
    OSCCONbits.SCS =      1;
    OPTION_REGbits.T0CS = 0;    //Timmer 0 a FOSC y Prescalador asignado
    OPTION_REGbits.PSA  = 0;
    OPTION_REGbits.PS2  = 0;    //valor del prescalador 16
    OPTION_REGbits.PS1  = 1;
    OPTION_REGbits.PS0  = 1;
    INTCONbits.T0IF =     0;
    TMR0 =              131;                 //Cambia cada 1 ms
    SERVOS.bit1 =         0;     //esperar el timmer0
    SERVOS.modo =         0;
}

void servos(void){
    //cambair los valores por el tiempo estipulado
    if(SERVOS.bit1){
            if(SERVOS.bit0 == 15) SERVOS.bit0 = 0;
            SERVOS.bit1 = 0;
            switch(SERVOS.bit0){
                //estos apagan las señales
                case 1:
                    TMR0 = 255-POT1; PORTDbits.RD0 = 0;
                    break;
                case 4:
                     TMR0 = 255-POT2; PORTDbits.RD1 = 0;
                    break;
                case 7:
                    TMR0 = 255-POT3; PORTDbits.RD2 = 0;
                    break;
                case 10:
                    TMR0 = 255-POT4; PORTDbits.RD3 = 0;
                    break;
                //estos encienden las señales
                case 0:
                    TMR0 = POT1; PORTDbits.RD0 = 1;
                    break;
                case 3:
                    TMR0 = POT2; PORTDbits.RD1 = 1;
                    break;
                case 6:
                    TMR0 = POT3; PORTDbits.RD2 = 1;
                    break;
                case 9:
                    TMR0 = POT4; PORTDbits.RD3 = 1;
                    break;
                    
            }
    }
    
}

void AnalogReadServo(void){
    //cambiar los valores acorde a las nuevas lecturas
    if(!ADCON0bits.GO){
        switch(SERVOS.bit0){
            case 0:
                 POT2 = ADRESH;
            break;
            
            case 1:
                ADCON0bits.CHS = 2;
                if(POT2>190) POT2 = 190;
                if(POT2<10) POT2 = 10;
            break;
            
            case 3:
                POT3 = ADRESH;
            break;
            
            case 4:
                ADCON0bits.CHS = 3;
                if(POT3>190) POT3 = 190;
                if(POT3<10) POT3 = 10;
            break;
            
            case 6:
                POT4 = ADRESH;
            break;
            
            case 7:
                ADCON0bits.CHS = 0;
                if(POT4>190) POT4 = 190;
                if(POT4<10) POT4 = 10;
            break;
            
            case 9:
                POT1 = ADRESH;
            break;
            
            case 10:
                ADCON0bits.CHS = 1;
                if(POT1>190) POT1 = 190;
                if(POT1<10) POT1 = 10;
            break;
        }
            
    }   
}