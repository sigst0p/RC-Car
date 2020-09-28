// MIT License

// Copyright (c) 2020 sigst0p

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.


//Example JSON-STRING
//{"cmd":"lenkung","params":{"angle":51,"speed":100}}


#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

#include <util/delay.h>

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include "jsmn.h"//Quelle : https://github.com/zserge/jsmn

#define USART_BAUDRATE 38400

#define UBRR_VALUE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)

#define BUF_SIZE 70
#define DATAMAX 4

#define HIGH 1
#define LOW 0

#define TRUE 1
#define FALSE 0

#define JSMN_STATIC 1
#define JSMN_STRICT 1

#define MOTORFWD 0
#define MOTORBWD 1
#define SERVOR 2
#define SERVOL 3

typedef struct {
    char *cmd;
    uint8_t cmdsize;
    void *params;
}result;

struct limit{
    uint16_t value;
};

struct limit s_limit;

struct motor{
    uint8_t speed;
    uint16_t angle;
};

struct motor s_motor;

//Haellt alle Informationen die aus dem JSON-String extrahiert werden
volatile result JsonResult;

volatile uint8_t collision_lock = 0;
volatile uint16_t collision_limit = 100;

//FIFO Buffer
struct datastruct{
    uint8_t buf[BUF_SIZE];
    uint8_t writepos;
    uint8_t locked;
};

//FIFO Buffer intialisieren
struct datastruct data[DATAMAX] = {0};
volatile uint8_t datapos = 0;


volatile uint16_t analogValServo;
volatile uint16_t analogValSensor;


//Wrapper Funktion zum schreiben der Signale an den IO Ports
void write_signal(uint8_t ioport, uint8_t signal)
{
    switch (ioport)
    {
    case MOTORFWD:
        if(signal){
            PORTD |= (1 << PORTD6);
        }else{
            PORTD &= ~(1 << PORTD6);
        }
        break;
    case MOTORBWD:
        if(signal){
            PORTD |= (1 << PORTD5);
        }else{
            PORTD &= ~(1 << PORTD5);
        }
        break;
    case SERVOR:
        if(signal){
            PORTD |= (1 << PORTD7);
        }else{
            PORTD &= ~(1 << PORTD7);
        }
        break;
    case SERVOL:
        if(signal){
            PORTB |= (1 << PORTB0);
        }else{
            PORTB &= ~(1 << PORTB0);
        }
        break;
    default:
        break;
    }
}

//Wrapper Funktion zum lesen der Signale an den IO Ports
uint8_t read_signal(uint8_t ioport)
{
    switch (ioport)
    {
    case MOTORFWD:
        return PORTD & (1 << PORTD6);
        break;
    case MOTORBWD:
        return PORTD & (1 << PORTD5);
        break;
    case SERVOR:
        return PORTD & (1 << PORTD7);
        break;
    case SERVOL:
        return PORTB & (1 << PORTB0);
        break;
    default:
        break;
    }
    return 0;
}

//Funktion zur Initialiserung des USART
void USART0_Init(void)
{
    //Baud rate festlegen
    UBRR0H = (uint8_t)(UBRR_VALUE>>8);
    UBRR0L = (uint8_t)UBRR_VALUE;
    //frame format auf 8 data bits, keine parity, 1 stop bit setzen
    UCSR0C |= (1<<UCSZ01)|(1<<UCSZ00);
    //Reception und RX complete interrupt aktivieren
    UCSR0B |= (1<<RXEN0)|(1<<RXCIE0);
}

//Funktion zur Initialiserung des ADC
void ADC_Init(void)
{
    // Right Adjust Result verwenden damit eine Aufloesung von 10bit moeglich wird
    ADMUX &= ~(1 << ADLAR);
 
    //REFS0 als Referenz Spannung nutzen
    ADMUX |= (1 << REFS0);
 
    //Analog input MUX3..0 sauber machen damit kein falscher Channel genommen wird
    ADMUX &= ~((1 << MUX3) | (1 << MUX2) | (1 << MUX1) | (1 << MUX0));
 
    //Channel auf A1 Pin setzen
    ADMUX |= (1 << MUX0);

    //ADEN setzen damit der ADC aktiv wird
    ADCSRA |= (1 << ADEN);
 
    // Auto trigger Mode aktivieren.
    //ADCSRA |= (1 << ADATE);
 
    //Trigger mode auf free running setzen
    ADCSRB &= ~((1 << ADTS2) | (1 << ADTS1) | (1 << ADTS0));

    //Den Prescaler auf 128 setzen.
    ADCSRA |= ((1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0));
 
    //Den ADC ISR aktivieren
    ADCSRA |= (1 << ADIE);

    //Konversion starten (auto trigger wird nicht genutzt)
    ADCSRA |= (1 << ADSC);
}

//Funktion zur Initialiserung der IO
void IO_Init(void)
{
    //Alle IO Ports die gebraucht werden als OUTPUT setzen
    DDRD |= (1<<DDD5);
    DDRD |= (1<<DDD6);
    DDRD |= (1<<DDD7);
    DDRB |= (1<<DDB0);
    //Alle Signale auf HIGH setzen denn die Relais sind Active-Low
    write_signal(MOTORFWD, HIGH);
    write_signal(MOTORBWD, HIGH);
    write_signal(SERVOL, HIGH);
    write_signal(SERVOR, HIGH);
}

//Funktion zum fuellen des FIFOs, wenn ein Byte vom Bluetooth Module empfangen wurde
//Funktion wird von der USART_RX_vect ISR Routine aufgerufen
void write_buf(uint8_t u8data){
    if (datapos == DATAMAX){
        datapos = 0;
    }

    for(int i = datapos; i < DATAMAX; i++){
        if(data[i].locked == FALSE){
            if (data[i].writepos < BUF_SIZE){
                //Wenn ein line feed empfangen wurde so wird der String als vollstaendig angesehen,
                //also wird der Buffer gesperrt so dass beim naechsten empfangen nichts ausversehen
                //ueberschrieben werden kann
                if (u8data == '\n'){
                    data[i].buf[data[i].writepos] = 0;
                    data[i].locked = TRUE;
                    datapos++;
                    break;
                }else{
                    data[i].buf[data[i].writepos] = u8data;
                    data[i].writepos++;
                    break;
                }
            }
            //Falls beim empfangen durch ein Uebertragungsfehler kein line feed empfangen wird
            //so muss man aufpassen dass der Buffer nicht ueberlaeuft.
            else{
                data[i].locked = TRUE;
                datapos++;
                break;
            }
        }
    }
}

//RX Complete ISR
//Wenn ein Byte vom Bluetooth Module zum MCU gesendet wurde
//so wird diese ISR ausgefuhrt
ISR(USART_RX_vect)
{
    //Alle anderen ISR sperren
    cli();

    write_buf(UDR0);

    //Alle ISR wieder aktivieren
    sei();
}



ISR(ADC_vect){
    //A1
    //Wenn das erste Bit im ADMUX register gesetzt war so muss der konvertierte Wert,
    //der Wert sein der vom IR-Sensor kam
    if (ADMUX & 1){
        analogValSensor = ADCL | (ADCH << 8);
        //Toggeln damit bei der naechsten Konversion der Wert fuer den Servo ausgewertet wird
        ADMUX &= 0b11110000;
    }
    //A0
    //Falls das Bit nicht gesetzt wurde so muss dass der Wert fuer von dem Servo
    //Potentiometer sein 
    else if ((ADMUX & 1) == FALSE){
        analogValServo = ADCL | (ADCH << 8);
        //Toggeln damit bei der naechsten Konversion der Wert fuer den Sensor ausgewertet wird
        ADMUX |= 1;
    }

    //naechste Konversion starten
    ADCSRA |= (1 << ADSC);
}

//Funktion zum konvertieren eines String in ein Integer
uint16_t str2int(char* str, uint8_t len)
{
    uint16_t ret = 0;
    for(int i = 0; i < len; ++i)
    {
        ret = ret * 10 + (str[i] - '0');
    }
    return ret;
}

//Funktion zum parsen des JSON-String
int parse_json(uint8_t* databuf, uint8_t data_len) {
    //Variable gibt an wie viele JSON Tokens ein String enthaellt
    int r;
    //Json Parser Object
    jsmn_parser p;
    //mehr als 15 Tokens brauchen wir nicht
    jsmntok_t t[15];

    jsmn_init(&p);
    r = jsmn_parse(&p, databuf, data_len, t, 15);

    if (r < 0) {
        //Gibt fehler zuruck
        return 1;
    }

    /* Assume the top-level element is an object */
    if (r < 1 || t[0].type != JSMN_OBJECT) {
        //Gibt fehler zuruck
        return 1;
    }


    JsonResult.cmd = (databuf + t[2].start);
    JsonResult.cmdsize = t[2].end - t[2].start;

    //Wenn der Json String 3 Token enthaellt so muss dieser nur ein einfaches Befehl enthalten
    if (r == 3){
        return 0;
    }

    //Wenn der Json String 9 Token enthaellt so muss dieser Informationen zur Motor/Lenkung Steuerung enthalten
    if(r == 9){

        JsonResult.params = &s_motor;
        uint8_t speedsize = t[8].end - t[8].start;
        ((struct motor*)JsonResult.params)->speed = str2int(((char*)(databuf + t[8].start)), speedsize);
        uint8_t anglesize = t[6].end - t[6].start;
        ((struct motor*)JsonResult.params)->angle = str2int(((char*)(databuf + t[6].start)), anglesize);
        

        return 0;
    }

    //Wenn der Json String 7 Token enthaellt so muss dieser Informationen zur Konfiguration des IR-Sensor enthalten
    if(r == 7){

        JsonResult.params = &s_limit;
        uint8_t valuesize = t[6].end - t[6].start;
        ((struct limit*)JsonResult.params)->value = str2int(((char*)(databuf + t[6].start)), valuesize);
        

        return 0;
    }

    //Gibt fehler zuruck
    return 1;
}

//Mit dieser Funktion soll geprueft werden ob die Lenkung nicht zu sehr ausschwenkt
int check_servo(void){
    if (analogValServo >= 750 && analogValServo != 0) {
        return SERVOR;
    }
    if (analogValServo <= 350 && analogValServo != 0){
        return SERVOL;
    }

    return 0;
}

//Mit dieser Funktion soll geprueft werden ob sich ein Objekt vor dem Fahrzeug befindet
int check_collision(void){
    if ((collision_lock == TRUE) && (analogValSensor > collision_limit)){
        return TRUE;
    }
    
    return FALSE;
}

//Mit dieser Funktion werden die Motor Steuerbefehle ausgewertet
void parse_motor(void){
    int collision = check_collision();
    if (collision){
        write_signal(MOTORFWD, HIGH);
    }

    uint16_t angle = ((struct motor*)JsonResult.params)->angle;
    uint8_t speed = ((struct motor*)JsonResult.params)->speed;
    
    if ((speed < 50) || (angle == 0)){
        write_signal(MOTORFWD, HIGH);
        write_signal(MOTORBWD, HIGH);
        return;
    }

    if ((angle > 0 && angle <= 90) && !collision ){
        write_signal(MOTORFWD, LOW);
        write_signal(MOTORBWD, HIGH);
        return;
    }
    if ((angle >= 270 && angle <= 360) && !collision){
        write_signal(MOTORFWD, LOW);
        write_signal(MOTORBWD, HIGH);
        return;
    }
    if (angle >= 90 && angle <= 270){
        write_signal(MOTORBWD, LOW);
        write_signal(MOTORFWD, HIGH);
        return;
    }
    return;
}

//Mit dieser Funktion werden die Lenk Steuerbefehle ausgewertet
void parse_lenkung(void){
    int lock = check_servo();
    if (lock == SERVOL){
        write_signal(SERVOL, HIGH);
    }
    if (lock == SERVOR){
        write_signal(SERVOR, HIGH);
    }

    uint16_t angle = ((struct motor*)JsonResult.params)->angle;
    uint8_t speed = ((struct motor*)JsonResult.params)->speed;
    
    if ((speed < 50) || (angle == 0)){
        write_signal(SERVOL, HIGH);
        write_signal(SERVOR, HIGH);
        return;
    }
    
    if ((angle > 0 && angle <= 180) && (lock != SERVOL) ){
        write_signal(SERVOL, LOW);
        write_signal(SERVOR, HIGH);
        return;
    }
    if (angle >= 180 && angle <= 360 && (lock != SERVOR)){
        write_signal(SERVOR, LOW);
        write_signal(SERVOL, HIGH);
        return;
    }

    return;
}

//Mit dieser Funktion wird der ausgewertet JSON String analysiert
void parse_command(void)
{
    if (strncmp(JsonResult.cmd, "lenkung", JsonResult.cmdsize) == 0){
        parse_lenkung();
        return;
    }

    if (strncmp(JsonResult.cmd, "motor", JsonResult.cmdsize) == 0){
        parse_motor();
        return;
    }

    if (strncmp(JsonResult.cmd, "colon", JsonResult.cmdsize) == 0){
        collision_lock = TRUE;
        return;
    }

    if (strncmp(JsonResult.cmd, "coloff", JsonResult.cmdsize) == 0){
        collision_lock = FALSE;
        return;
    }

    if (strncmp(JsonResult.cmd, "limit", JsonResult.cmdsize) == 0){
        collision_limit = (((struct limit*)JsonResult.params)->value) * 4;
        return;
    }
}

//Mit dieser Funktion wird mit jedem neuen Zyklus geprueft dass bestimme
//Zustaende niemals erreicht werden koennen
//Der Motor sollte niemals gleichzeitig vorwaerts und rueckwaerts eingestellt
//sein und es wird geprueft ob sich ein Objekt vor dem Fahrzeug befindet
void super_gau(void){
    int motorfwdstatus = read_signal(MOTORFWD);
    int motorbwdstatus = read_signal(MOTORBWD);
    int servolstatus = read_signal(SERVOL);
    int servorstatus = read_signal(SERVOR);

    if((motorfwdstatus == LOW) && (motorbwdstatus == LOW)){
        write_signal(MOTORFWD, HIGH);
        write_signal(MOTORBWD, HIGH);
    }
    if((servolstatus == LOW) && (servorstatus == LOW)){
        write_signal(SERVOL, HIGH);
        write_signal(SERVOR, HIGH);
    }

    int servo = check_servo();
    if (servo == SERVOL){
        write_signal(SERVOL, HIGH);
    }
    if (servo == SERVOR){
        write_signal(SERVOR, HIGH);
    }

    int collision = check_collision();
    if (collision){
        write_signal(MOTORFWD, HIGH);
    }

}

int main (void)
{
    //Init IO
    IO_Init();
    //Initialize USART0
    USART0_Init();
    //ADC.
    ADC_Init();
    //enable global interrupts
    sei();
    while(TRUE)
    {
        super_gau();

        //FIFO Buffer parsen
        for(int i = 0; i < DATAMAX; i++){
            if (data[i].locked == TRUE){
                if (parse_json(&data[i].buf[0], data[i].writepos) == 0){
                    parse_command();
                }
                data[i].writepos = 0;
                //Speicher wieder freigeben
                data[i].locked = FALSE;
            }
        }

    }
}