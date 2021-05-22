/* 
 * File:   mainP2.c
 * Autor: Jose Luis Alvarez Pineda (19392)
 *
 * Creado el 6 de Mayo de 2021.
 * Ultima edicion: 19 de Mayo de 2021.
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
        unsigned guardar: 1; // indica para guardar
    };
}SERVOS;

union MENSAJE{ //para manejar grupos de bits
    struct{
        unsigned datorecep: 1; //1 bit- indica que recibe al usuario
        unsigned leerpos:   1;  //para indicar que se lee posicion de servos
        unsigned piederecho: 1; //para indicar que el valor pasa al pie derecho
        unsigned pieizquierdo: 1; //indicar que se mueve el pieizquierdo
        unsigned piernaderecho: 1; //indicar que se mueve piernaderecho
        unsigned piernaizquierda: 1; //indicar que se mueve piernaizquierda
    };
}UART;


uint8_t POT1,POT2,POT3,POT4,EXTREC,SERVINDIC; //SERVO y serial
uint8_t datEEPROM; //dato y direccion EEPROM
uint8_t posicion = 0; //numero de posicion a guardar
/*------------------------------------------------------------------------------
 *                                  Prototipos
 -----------------------------------------------------------------------------*/
void configuraciones(void); //rutina de configuracion
void servos(void);          //manejo de los servos
void AnalogReadServo(void); //para leer los pots para los servos
void send1dato(char dato); //para enviar un dato
void guardarposiciones(uint8_t guardar, uint8_t direccion); 
//guardar en EEPROM
void guardarservos(uint8_t desfase); //guardar los 4 servos
uint8_t leerposiciones(uint8_t direccion); //leer la posicion de memoria dada
void leerSERVOS(uint8_t desfase); //leer los 4 servos
//leer las posiciones de los servos
void guardar3SEG(void); //guardar posiciones en los 3 segundos
void leer3SEG(void); //leer las posiciones en los 3 segundos
/*------------------------------------------------------------------------------
 *                                  Interrupcion
 -----------------------------------------------------------------------------*/
void __interrupt() rutInter(void){

    if(INTCONbits.TMR0IF) {
        SERVOS.bit0++;
        INTCONbits.TMR0IF = 0;
        servos();
        if(SERVOS.modo) ADCON0bits.GO = 1;
    }
    
    if(PIR1bits.TMR1IF){
        SERVOS.guardar = 1;
        posicion ++;
        PIR1bits.TMR1IF = 0;
        TMR1H = 0B00111100;     //para overflow cada 0.1seg
        TMR1L = 0B10101111;
    }
    
    if(INTCONbits.RBIF && PORTBbits.RB0){ //cambia de modo
        if(!T1CONbits.TMR1ON)SERVOS.modo = ~SERVOS.modo;//no cambiar en 
        //guardado o lectura
        INTCONbits.RBIF = 0;
    }
    
    if(INTCONbits.RBIF && PORTBbits.RB1){ //guardar la posicion actual
        if(SERVOS.modo){T1CONbits.TMR1ON = 1;//enciende el timer 1
        PORTEbits.RE0 = 1;}
        if(SERVOS.modo)SERVOS.guardar = 1;
        INTCONbits.RBIF = 0;
    }
    
    INTCONbits.RBIF = 0;
    
    if(PIR1bits.RCIF){
        EXTREC = RCREG;
        UART.datorecep = 1;
        PIR1bits.RCIF = 0;
    }
    
    
}

/*------------------------------------------------------------------------------
 *                                Loop principal
 -----------------------------------------------------------------------------*/
void main(void) {
    configuraciones();
    while(1){
        
        switch(SERVOS.modo){
            case 0: //modo UART
                PORTBbits.RB7 = 0;     
                
                if(UART.datorecep){
                    switch(EXTREC){
                        case 1:
                            T1CONbits.TMR1ON = 1;
                            PORTEbits.RE0 = 1;
                            EXTREC = 0;
                            break;
                        case 2:
                            CCPR2L = 0xFF;
                            send1dato('b');
                            EXTREC = 0;
                            break;
                        case 3:
                            CCPR2L = 0x0F;
                            send1dato('c');
                            EXTREC = 0;
                            break;
                        case 4:
                            UART.piederecho = 1;
                            EXTREC = 95;
                            break;
                        case 5:
                            UART.pieizquierdo = 1;
                            EXTREC = 95;
                            break;
                        case 6:
                            UART.piernaderecho = 1;
                            EXTREC = 95;
                            break;
                        case 7:
                            UART.piernaizquierda = 1;
                            EXTREC = 95;
                            break;
                        case 8:
                            UART.piederecho = 0;
                            UART.pieizquierdo = 0;
                            UART.piernaderecho = 0;
                            UART.piernaizquierda = 0;
                            break;
                        default:
                            break;
                    }
                    
                    UART.datorecep = 0;
                    
                if(UART.piederecho){
                    if(EXTREC<=10)EXTREC = 10;
                    if(EXTREC>=160)EXTREC = 160;
                    POT1 = EXTREC;
                }
                
                if(UART.pieizquierdo){
                    if(EXTREC<=10)EXTREC = 10;
                    if(EXTREC>=160)EXTREC = 160;
                    POT2 = EXTREC;
                }
                    
                if(UART.piernaderecho){
                    if(EXTREC<=10)EXTREC = 10;
                    if(EXTREC>=160)EXTREC = 160;
                    POT3 = EXTREC;
                }
                    
                if(UART.piernaizquierda){
                    if(EXTREC<=10)EXTREC = 10;
                    if(EXTREC>=160)EXTREC = 160;
                    POT4 = EXTREC;
                }
                
                }
                if(T1CONbits.TMR1ON){
                    leer3SEG();//reproduce el movimiento de los 3 segundos
                }
                
                break;
            case 1: //modo manual
                PORTBbits.RB7 = 1;

                if(SERVOS.guardar){ //Para guardar la posicion en un tiempo dado
                    guardar3SEG(); //guarda durante 3 segundos el movimiento
                    SERVOS.guardar = 0;
                }
                
                AnalogReadServo();
                break;
        }
   
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
    TRISB =         0X07;//3 BOTONES
    TRISC =         0X82;
    TRISD =         0X00;
    TRISE =         0X00;
    PORTA =         0X00;
    PORTB =         0X00;
    PORTC =         0X00;
    PORTD =         0X00;
    PORTE =         0X00;
    
    //inicializando valores
    posicion = 0;
    UART.piederecho = 0;
    POT1 = 95;
    POT2 = 95;
    POT3 = 95;
    POT4 = 95;

    //Configuracion del reloj
    OSCCONbits.IRCF = 0b111; //oscilador a 8Mhz
    OSCCONbits.SCS = 0b1;
    
    //Configuracion del TIMER 1
    T1CONbits.T1CKPS = 0B10;    //preescalador de 4
    TMR1H = 0B00111100;     //para overflow cada 0.1seg
    TMR1L = 0B10101111;
    T1CONbits.TMR1ON = 0; //mantenerlo apagado
    
    //Configuracion de interrupciones
    INTCONbits.TMR0IF =     0;
    INTCONbits.TMR0IE =     1;//interrupciones timer0
    PIR1bits.TMR1IF =       0;
    PIE1bits.TMR1IE =       1;//interrupciones timer1
    INTCONbits.RBIF =       0;
    INTCONbits.RBIE =       0;//interrupciones IOC puerto B
    INTCONbits.PEIE =       1;//interrupciones de perifericos
    PIE1bits.RCIE   =       1; //permite interrupciones de recepcion de datos
    INTCONbits.GIE =        1;         //habilita interrupciones
    
    //Configuracion de cambio de modo
    IOCBbits.IOCB0 = 1;     //detecta cambios en RB0
    IOCBbits.IOCB1 = 1;     //detecta cambios en RB1
    IOCBbits.IOCB2 = 1;     //detecta cambios en RB2
    
    //Configutacion del ADC
    ADCON0bits.ADCS =   0b10;
    ADCON0bits.CHS =    0b0000;  //comience leyendo en el 0
    ADCON0bits.GO =     0b0;
    ADCON0bits.ADON =   0b1;
    ADCON1bits.ADFM =   0b0; //justificado a la IZQUIERDA
    ADCON1bits.VCFG1 =  0b0; //referencias a alimentacion del PIC
    ADCON1bits.VCFG0 =  0b0;
    
    //configuracion del PWM (usado el 2)
    PR2 = 249;                  //precargando para el periodo del PWM
    CCP2CONbits.CCP2M = 0b1111; //CCP2 como PWM
    CCPR2L = 0x0F;              //precargando duty cicle inicial
    PIR1bits.TMR2IF = 0;        //apaga la bandera
    T2CONbits.T2CKPS = 0b11;    //prescalador en 16
    T2CONbits.TMR2ON = 1;       //enciende el timmer
    
    while(!PIR1bits.TMR2IF);    //loop para que no cambie a salida hasta un ciclo
    TRISC = 0X80;               //CCP2 como salida habilitada
    
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
    if(SERVOS.bit0 == 15) SERVOS.bit0 = 0;

    switch(SERVOS.bit0){
        case 0:
            TMR0 = POT1; PORTDbits.RD0 = 1;
            break;
        case 1:
            TMR0 = 255-POT1; PORTDbits.RD0 = 0;
            break;
        case 3:
            TMR0 = POT2; PORTDbits.RD1 = 1;
            break;
        case 4:
             TMR0 = 255-POT2; PORTDbits.RD1 = 0;
            break;
        case 6:
            TMR0 = POT3; PORTDbits.RD2 = 1;
            break;
        case 7:
            TMR0 = 255-POT3; PORTDbits.RD2 = 0;
            break;
        case 9:
            TMR0 = POT4; PORTDbits.RD3 = 1;
            break;
        case 10:
            TMR0 = 255-POT4; PORTDbits.RD3 = 0;
            break;       
    }

    
}

void AnalogReadServo(void){
    //cambiar los valores acorde a las nuevas lecturas
    if(!ADCON0bits.GO){
        switch(SERVOS.bit0){
            case 0:
                 if(!EECON1bits.WR && ADCON0bits.CHS == 1)POT2 = ADRESH;
                 else POT2 = POT2;
            break;
            
            case 1:
                if(!EECON1bits.WR)ADCON0bits.CHS = 2;
                if(POT2>=160) POT2 = 160;
                if(POT2<=10) POT2 = 10;
            break;
            
            case 3:
                if(!EECON1bits.WR && ADCON0bits.CHS == 2)POT3 = ADRESH;
                else POT3 = POT3;
            break;
            
            case 4:
                if(!EECON1bits.WR)ADCON0bits.CHS = 3;
                if(POT3>=160) POT3 = 160;
                if(POT3<=10) POT3 = 10;
            break;
            
            case 6:
                if(!EECON1bits.WR && ADCON0bits.CHS == 3)POT4 = ADRESH;
                else POT4 = POT4;
            break;
            
            case 7:
                if(!EECON1bits.WR)ADCON0bits.CHS = 0;
                if(POT4>=160) POT4 = 160;
                if(POT4<=10) POT4 = 10;
            break;
            
            case 9:
                if(!EECON1bits.WR && ADCON0bits.CHS == 0)POT1 = ADRESH;
                else POT1 = POT1;
            break;
            
            case 10:
                if(!EECON1bits.WR)ADCON0bits.CHS = 1;
                if(POT1>=160) POT1 = 160;
                if(POT1<=10) POT1 = 10;
            break;
        }
            
    }   
}

void send1dato(char dato){ 
    TXREG = dato;   //carga el dato que se va a enviar
    while(!TXSTAbits.TRMT); //espera hasta que se envie el caracter
}

void guardarposiciones(uint8_t guardar, uint8_t direccion){
    EEADR = direccion;  //direccion a escribir
    EEDAT = guardar;    //dato a guardar
    EECON1bits.WREN = 1; //permite escribir
    INTCONbits.GIE = 0;
    EECON2 = 0X55;      //obligatorio
    EECON2 = 0XAA;
    EECON1bits.WR = 1;
    INTCONbits.GIE = 1;
    while(EECON1bits.WR);
    EECON1bits.WREN = 0; //no permite escribir
}

void guardarservos(uint8_t desfase){
    for(uint8_t n=0;n<=3;n++){
        switch(n){
            case 0: guardarposiciones(POT1,n+desfase);
                break;
            case 1: guardarposiciones(POT2,n+desfase);
                break;
            case 2: guardarposiciones(POT3,n+desfase);
                break;
            case 3: guardarposiciones(POT4,n+desfase);
                break;
        }
    }
}

uint8_t leerposiciones(uint8_t direccion) {
    EEADR = direccion;    //direccion a leer
    EECON1bits.EEPGD = 0; //apuntar a memoria de datos
    EECON1bits.RD = 1;      //Comenzar a leer
    return EEDAT;
}

void leerSERVOS(uint8_t desfase){
    for(uint8_t n=0;n<=3; n++){
        switch(n){
            case 0: POT1 = leerposiciones(n+desfase);
                break;
            case 1: POT2 = leerposiciones(n+desfase);
                break;
            case 2: POT3 = leerposiciones(n+desfase);
                break;
            case 3: POT4 = leerposiciones(n+desfase);
                break;   
        }        
    }
}

void guardar3SEG(void){
    switch(posicion){
        case 31:
            T1CONbits.TMR1ON = 0;
            TMR1H = 0;     //reinicio
            TMR1L = 0;
            posicion = 0;
            PORTE = 0;
            break;
        default:
            guardarservos(4*posicion);
            break;
    }
}

void leer3SEG(void){
    switch(posicion){
        case 31:
            T1CONbits.TMR1ON = 0;
            TMR1H = 0;     //reinicio
            TMR1L = 0;
            posicion = 0;
            PORTE = 0;
            POT1 = 95;
            POT2 = 95;
            POT3 = 95;
            POT4 = 95;
            send1dato('a'); //confirma de terminado replicado
            break;
        default:
            leerSERVOS(4*posicion);
            break;
    }
}