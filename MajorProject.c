/*
 * Major Project - Traffic Light System
 * Lee Forster - 45950393
 * Created on 4 May 2021, 2:01 PM
 */

#include <xc.h>
#include <avr/io.h>
#include <avr/interrupt.h>


volatile uint32_t buzzer_timer = 0; // Buzzer previous value for sound switching
volatile uint32_t current_ms = 0; // System clock from time started
volatile int btnFlag = 0; // Button Flag variable

// Sensor variables
volatile int BNS_S0 = 0x01; // S0 - portA
volatile int BST_S1 = 0x10; // S1 - portA
volatile int LS_S2 = 0x01; // S2 - portB
volatile int BN_S4 = 0x10; // S4 - portB


// Flags for each sensor
volatile int HAZARD_Flag = 0;
volatile int BS_Flag = 0;
volatile int BST_Flag = 0;
volatile int BN_Flag = 0;
volatile int BNT_Flag = 0;
volatile int LS_Flag = 0;
volatile int BP_Flag = 0;
volatile int LSP_Flag = 0;

// Inputs variable for ISR and button checker functions
 volatile uint8_t inputsD = 0b10011000;
 volatile uint8_t inputsC = 0b00000010;


// Phase Variables
int HazardPhase = 1;
int BNS_Phase = 1;
int BST_Phase = 1;
int LS_Phase = 1;
int LSP_Phase = 1;
int BP_Phase = 1;
int Speaker_Phase = 1;


// Timer and prev_ms variables
volatile uint32_t prev_ms = 0;
volatile uint32_t Hazard_Timer = 0;
volatile uint32_t LSP_Timer = 0;
volatile uint32_t BP_Timer = 0;


// LCD Globals
uint8_t LCD_Addr = 0x27;
char sensorDisp[] ="OOOOOOOO";
    
// Main State Machine enum variables
enum STATE {HAZARD, BNS, BST, LS, BP, LSP};
enum STATE main_state; 



// Main setup procedure for Arduino Ports
void setup() {
    DDRD |= 0b01100000; // 3,4,7 Inputs, 5,6 Outputs
    DDRB |= 0b00111111; // All used pins are outputs
    DDRC |= 0b11001100; // 7,6,3,2 Outputs, rest inputs
    PORTC |= 0b00110010;  // Pull-up resistor Configuration
    PORTD |= 0b10011100;  // Pull-up resistor Configuration
    
}


void setup_Timer1() { // Buzzer Setup
    TCNT1 = 0;
    OCR1A = 0; 
    TCCR1A = 0b01000000;
    TCCR1B = 0b00001001;
    TCCR1C = 0b00000000;
}


uint8_t SPI_transfer(uint8_t data) { // SPI Setup
    
    SPDR = data;
    while ((SPSR & _BV(SPIF)) == 0) {
        ;   // wait until transfer completed
    }
    return SPDR;
}

void SPI_Send_Command(uint8_t reg, uint8_t data) {
        PORTB &= ~_BV(2); // SS Line Low
        SPI_transfer(0x40); // Begin transfer
        SPI_transfer(reg);
        SPI_transfer(data);
        PORTB |= _BV(2); // SS Line high

   
}

uint8_t SPI_Read_Command(uint8_t reg) { 
    uint8_t data; 
    
    PORTB &= ~_BV(2); // SS Line low
    SPI_transfer(0x41); // Read Register
    SPI_transfer(reg);
    data = SPI_transfer(0);
    PORTB |= _BV(2); // SS Line High

    return data;
}




void setup_SPI() {
 SPCR = _BV(SPE) | _BV(MSTR); // set master SPI, SPI mode 0 operation
 SPSR = 0; // clear interrupt flags and oscillator mode.

    // Input = 1, Output = 0
    SPI_Send_Command(0x0A, 0b01000010); // IOCON Reg set to mirror
    SPI_Send_Command(0x0B, 0b01000010);
    SPI_Send_Command(0x00, 0x11); // register IODIRA, Inputs and Outputs Configured
    SPI_Send_Command(0x0C, 0x11);  // register GPPUA - Turning on pull-ups for Inputs
    SPI_Send_Command(0x01, 0x11); // register IODIRB Inputs and Outputs Configured
    SPI_Send_Command(0x0D, 0x11); // register GPPUB - Turning on pull-ups for Inputs
    SPI_Send_Command(0x04, 0x11); // register GPINTENA Turning on PORTA's Interrupts
    SPI_Send_Command(0x05, 0x11); // register GPINTENB Turning on PORTB's Interrupts
    SPI_Send_Command(0x08, 0x00); // register INTCONA - Compare to previous bit mode
    SPI_Send_Command(0x09, 0x00); // register INTCONB - Compare to previous bit mode
}




// Globals for I2C Processes
#define I2C_READ    1
#define I2C_WRITE   0

#define I2C_LCD_BACKLIGHT   8
#define I2C_LCD_ENABLE      4
#define I2C_LCD_RW          2
#define I2C_LCD_RS          1

void setup_I2C() { // Setup Registers, PORTC Registers already configured
    TWBR = 193; // Division Factor for Bit Rate
    TWSR = 0; // Status Register set 0
}


void I2C_wait() { // Function for waiting for I2C Operations
    while ((TWCR & _BV(TWINT)) == 0) {
        ;
    }
}


int I2C_Start() { // Sends the starting I2C Bit
    TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN);
    I2C_wait();
    return ((TWSR & 0xf8) == 0x08);
}

/**
 * Send an I2C address byte and R/W flag
 * 
 * @param addr I2C address of the slave
 * @param rw whether to read or write: 0 to write, 1 to read
 * @return true if the address byte was successfully transmitted
 */
int I2C_SLA(uint8_t addr, uint8_t rw) {
    // Send I2C slave address
    TWDR = (addr << 1) | (rw & 1);
    TWCR = _BV(TWINT) | _BV(TWEN);
    I2C_wait();
    return ((TWSR & 0xf8) == 0x18);
}

/**
 * Send a byte of data through the I2C bus
 * 
 * @param data data to transmit
 * @return true if the data was successfully transmitted
 */
int I2C_Send(uint8_t data) {
    // Send I2C data byte
    TWDR = data;
    TWCR = _BV(TWINT) | _BV(TWEN);
    I2C_wait();
    return ((TWSR & 0xf8) == 0x28);
}

/**
 * Send the stop flag on the I2C bus
 */
void I2C_Stop() {
    // Send I2C Stop flag
    TWCR = _BV(TWINT) | _BV(TWSTO) | _BV(TWEN);
    for (volatile long x = 0; x < 100; x++) {
        ;
    }
}

/**
 * Check if there is a device on the I2C bus at address
 * 
 * @param addr address to check if a device exists there.
 * @return true if a device acknowledges the address probe
 */
int I2C_CheckAddress(uint8_t addr) {
    int ret;
    
    ret = I2C_Start() & I2C_SLA(addr, I2C_WRITE);
    if (ret) {
        I2C_Stop();
    }
    return ret;
}

/**
 * Send four bits of data to a PCF8574 controlled HD44780 LCD display
 * We need to toggle the E bit (bit 2) from high to low to transmit the data
 * 
 * The 8 bits transmitted are:
 * bit7 bit6 bit5 bit4 bit3 bit2 bit1 bit0
 * DB7  DB6  DB5  DB4  BL   E    R/W  RS
 * BL is the back light (1 = on, 0 = off)
 * E is the enable bit (high to low transition latches the data
 * R/W is the read/write line (1 = read, 0 = write)
 * RS is Register Select (0 = control, 1 = data)
 * 
 * @param data the data to transmit
 * @return true if the data was transmitted
 */
int I2C_PCF8574_LCD_Nibble(uint8_t data) {
    TWDR = data | I2C_LCD_ENABLE;
    TWCR = _BV(TWINT) | _BV(TWEN);
    I2C_wait();
    if ((TWSR & 0xf8) == 0x28) {
        TWDR = data & (~I2C_LCD_ENABLE);
        TWCR = _BV(TWINT) | _BV(TWEN);
        I2C_wait();
    }
    return ((TWSR & 0xf8) == 0x28);
}

/**
 * Transmit the 8 bits of data as two four bit nibbles to a HD44780 LCD
 * controller in 4 bit mode attached through a PCF8574 port expander.
 * 
 * The byte is transmitted as the top nibble then the bottom nibble with
 * the bottom four bits being the control flags.
 * 
 * @param data 8 bits of data to transmit
 * @param flags 4 bits if flags
 * @return true if the data was transmitted
 */
int I2C_PCF8574_LCD_Byte(uint8_t data, uint8_t flags) {
    return I2C_PCF8574_LCD_Nibble ((data & 0xf0) | (flags & 0x0f)) && 
    I2C_PCF8574_LCD_Nibble (((data << 4) & 0xf0) | (flags & 0x0f));
}

/**
 * Send multiple bytes of data to the LCD display
 * 
 * @param addr address of the display
 * @param array pointer to a char array of data to transmit
 * @param len number of bytes in the array
 * @param flags the flags to transmit as the lower 4 bits
 */
void I2C_SendData(uint8_t addr, uint8_t *array, uint8_t len, uint8_t flags) {
    if (I2C_Start() & I2C_SLA(addr, I2C_WRITE)) {
        while (len > 0) {
            len--;
            if (I2C_PCF8574_LCD_Byte(*array++, flags) == 0) {
                break;  // bad send
            }
        }
    }
    I2C_Stop(); 
}

/**
 * Send the initialisation string for a HD44780 LCD controller connected in
 * 4 bit mode. Taken from the data sheet. Transmit 0x30 three times to ensure
 * it is in 8 bit mode, then 0x20 to switch to 4 bit mode.
 * We then turn on the blinking cursor, backlight and clear the display.
 * 
 * @param addr address of the LCD display
 * @return -1 if the display doesn't respond to a selection
 */
int8_t LCD_PCF8574_Setup(uint8_t addr) {
    if (!(I2C_Start() && I2C_SLA(addr, I2C_WRITE))) {
        return -1;
    }
    I2C_Send(0);    // ensure the PCF8574 enable line is low
    I2C_PCF8574_LCD_Nibble(0x30);
    I2C_PCF8574_LCD_Nibble(0x30);
    I2C_PCF8574_LCD_Nibble(0x30);
    I2C_PCF8574_LCD_Nibble(0x20);
    I2C_PCF8574_LCD_Byte(0x0f, I2C_LCD_BACKLIGHT);   // display on, cursor on and blinking
    I2C_PCF8574_LCD_Byte(0x01, I2C_LCD_BACKLIGHT);   // clear and move home
    I2C_Stop();
    return 0;
}

/**
 * Clear the LCD display (and return the cursor to the home position
 * 
 * @param addr address of the LCD display
 * @return -1 if the display doesn't respond to a selection
 */
int8_t LCD_clear(uint8_t addr) {
    if (!(I2C_Start() && I2C_SLA(addr, I2C_WRITE))) {
        return -1;
    }
    I2C_PCF8574_LCD_Byte(0x01, I2C_LCD_BACKLIGHT);   // clear screen command
    I2C_Stop();
    return 0;
}

/**
 * Set the cursor position on the LCD display
 * See the data sheet for mappings of position values to screen locations.
 * (0x00 top left, 0x40 second row left)
 * 
 * @param addr address of the LCD display
 * @param posn Location to where the cursor should be positioned
 * @return -1 if the display doesn't respond to a selection
 */
int8_t LCD_Position(uint8_t addr, uint8_t posn) {
    if (!(I2C_Start() && I2C_SLA(addr, I2C_WRITE))) {
        return -1;
    }
    I2C_PCF8574_LCD_Byte(0x80 | posn, I2C_LCD_BACKLIGHT);   // set DRAM address
    I2C_Stop();
    return 0;
}

/**
 * Write a string to the LCD display
 * 
 * @param addr address of the LCD display
 * @param str pointer to a character string to display
 * @param len length of the string to display
 * @return -1 if the display doesn't respond to a selection
 */
int8_t LCD_Write(uint8_t addr, char *str, uint8_t len) {
    if (!(I2C_Start() && I2C_SLA(addr, I2C_WRITE))) {
        return -1;
    }
    while (len--) {
        I2C_PCF8574_LCD_Byte(*str++, I2C_LCD_BACKLIGHT | I2C_LCD_RS);
    }
    I2C_Stop();
    return 0;
}

/**
 * Write a character to the LCD display
 * 
 * @param addr address of the LCD display
 * @param chr character to display
 * @return -1 if the display doesn't respond to a selection
 */
int8_t LCD_Write_Chr(uint8_t addr, char chr) {
    if (!(I2C_Start() && I2C_SLA(addr, I2C_WRITE))) {
        return -1;
    }
    I2C_PCF8574_LCD_Byte(chr, I2C_LCD_BACKLIGHT | I2C_LCD_RS);
    I2C_Stop();
    return 0;
}

/**
 * Setup the LCD display
 * 
 * @param addr address of the LCD display
 * @return -1 if the display doesn't respond to a selection
 */
int8_t setup_LCD(uint8_t addr) {
    if (LCD_PCF8574_Setup(addr) != 0) {
        return -1;
    }
    LCD_clear(addr);
    return 0;
}

// Helper Function to return string value of integer passed
char numCharHelper(int integer) {
    return "0123456789"[integer];
}

// Function to display current time period elapsed on LCD
void LCD_Timer( uint32_t timePeriod) {
    char timerArray[] = "00000"; // Character Array for time period
    
    LCD_Position(LCD_Addr, 0x0B); // Setting Position of Timer
    timerArray[0] = numCharHelper(((timePeriod/10000)%10)); // Takes in time period, returns string value of each digit of the time Period
    timerArray[1] = numCharHelper(((timePeriod/1000)%10));
    timerArray[2] = numCharHelper(((timePeriod/100)%10));
    timerArray[3] = numCharHelper(((timePeriod/10)%10));
    timerArray[4] = numCharHelper((timePeriod%10));
    LCD_Write(LCD_Addr, timerArray, 5); // Writing to display
}

void LCD_TP_Display(float tpScalerVal){ // Converting the scaler value from scaler function, to the time period in ms
    char timePeriodArray[] = "0000MS"; // The additional diagnostic on LCD, returns the time period
    int tp = 1000/tpScalerVal; // Taking scaler and turning it into ms of each time period
    timePeriodArray[0] = numCharHelper(((tp/1000)%10)); // Displaying each digit of time period
    timePeriodArray[1] = numCharHelper(((tp/100)%10));
    timePeriodArray[2] = numCharHelper(((tp/10)%10));
    timePeriodArray[3] = numCharHelper((tp%10));
    
    LCD_Position(LCD_Addr, 0x45); // Selecting position
    LCD_Write(LCD_Addr, timePeriodArray, 6); // Writing to display
    
    
}

void sensor_LCD() { // Displaying the sensor values of each button
    int sensorFlags[8]; // Creating integer array for each value
     char XO[] = "0X"; // Character array for outcomes
    
    sensorFlags[0] = BS_Flag; //  Writing each value to array
    sensorFlags[1] = BST_Flag;
    sensorFlags[2] = LS_Flag;
    sensorFlags[3] = BNT_Flag;
    sensorFlags[4] = BN_Flag;
    sensorFlags[5] = BP_Flag;
    sensorFlags[6] = LSP_Flag;
    sensorFlags[7] = HAZARD_Flag;
     
     for(int i = 0; i < 8; i++){ // Iterating over previous array and checking for 1 or 0, converting to X or O
         if(sensorFlags[i] == 1){
        sensorDisp[i] = XO[1];
    }else {
       sensorDisp[i] = XO[0]; 
    }
     }
    LCD_Position(LCD_Addr, 0);
    LCD_Write(LCD_Addr, sensorDisp, 8); // Writing to display
    
}

void LCD_Phase_Colour (char PhaseColour[]){ // Function to handle current phase and color display
    LCD_Position(LCD_Addr, 0x40);
    LCD_Write(LCD_Addr, PhaseColour, 4);
    
}


void timer0_setup(uint8_t val) { // System timer for ms time period
    TCCR0B = 0; // Turn off clock 
    TCNT0 = 0;
    OCR0A = val; // Passed value for timer counter ticks (Set at 249)
    TIFR0 = 0xff; // Clear flags
    TIMSK0 = _BV(OCIE0A); // CTC Match mode
    TCCR0A = 0b00000010; // WGM01 = 1 mode CTC
    TCCR0B = 0b00000011; // CS02 = 0, CS01 = 1, CS00 = 1 
}





void setup_interrupts() { 
    EICRA = 0b00000001; // set interrupt to occur on logical change - Mirror mode handles A and B Ports
    EIMSK = 0b00000001; // enable interrupt
    PCICR = _BV(PCIE2) | _BV(PCIE1); // enable interrupt for PORTD and C
    PCMSK2 = 0b10011100; // enable interrupt on PCINT2 select pins
    PCMSK1 = 0b00000010; // enable interrupts on PCINT1 select pins
}





void buttonHandler() { // Handler to check the button state
    if (btnFlag == 1){
        btnFlag = 0; // reset button flag on initiation
        uint8_t expanderA = SPI_Read_Command(0x12); // Reading SPI Ports
        uint8_t expanderB = SPI_Read_Command(0x13);
        // Reading all buttons, those that are to be held, clear when released
        if((expanderA & BNS_S0) == 0){
            BS_Flag = 1;
        } else {
            BS_Flag = 0;
        }
        if((expanderA & BST_S1) == 0){
            BST_Flag = 1;
        } else {
            BST_Flag = 0;
        }
        if((expanderB & LS_S2) == 0){
            LS_Flag = 1;
        } else {
            LS_Flag = 0;
        }
        if((expanderB & BN_S4) == 0){
            BN_Flag = 1;
        } else {
            BN_Flag = 0;
        }
        if((inputsD & 0b00010000 ) == 0) {
            BP_Flag = 1;
        } 
        if((inputsD & 0b00001000) == 0) {
            HAZARD_Flag = 1;   
        } 
        if((inputsD & 0b10000000) == 0){
            BNT_Flag = 1;
        } else {
            BNT_Flag = 0;
        }
        if((inputsC  & 0b00000010) == 0){ 
            LSP_Flag = 1;
           
        }        
    }
}

void hazardChecker() { // Function to check if hazard is pressed and when released, function is called in ISR
    if((inputsD & 0b00001000) == 0) {                     
        main_state = HAZARD;
        HazardPhase = 1;
        prev_ms = current_ms;
    } else{
        HAZARD_Flag = 0;
        Hazard_Timer = current_ms;
    }
    
}

// Three ISR routines, initiate other functions when set off
ISR(INT0_vect) {
    btnFlag = 1;
    
}

ISR(PCINT2_vect){
    inputsD = PIND;
    btnFlag = 1;
    hazardChecker();
    
}

ISR(PCINT1_vect) {
    inputsC = PINC;
    btnFlag = 1;
    
}

// Incrementing the current ms variable every time ISR is run
ISR(TIMER0_COMPA_vect) {
    current_ms++;
}

void setupAtoD(){  // Setting up AtoD conversion Pins
  DIDR0 = (1 << ADC0D); // Disable buffer
    ADMUX = 0b01100000; // Reference AVCC, Left adjusting and channel ADC0 on
    ADCSRA = 0b10000111; // ADC enabled, Start mode off, disable auto triggering, disable interrupt, pre-scaler 64  
    ADCSRB = 0b00000000; // Free running mode enabled
    ADCSRA |= _BV(ADSC);  // Writing bit to start conversion
}

uint16_t readAtoD() { // Reading the AtoD value
    ADCSRA |= (1 << ADSC); // Taking the ADSC High to initiate Read
    while(ADCSRA & (1 << ADSC)); // Taking conversion while true
    return ADCH; // Returning the MSB's of the reading
}



float convertAtoD(float val) {
    return (-(11.5*val)/255)+ 12.5; // Linear relationship between 255 and 0 and 80ms and 1000ms / 1s
}


int main(void) {
    // Calling all setup functions
    setup();
    setup_SPI();
    setup_interrupts();
    setupAtoD();
    setup_I2C();
    sei(); /// Enable global interrupts
    main_state = BNS;
    timer0_setup(249);
    setup_Timer1();

   
    if (setup_LCD(LCD_Addr) == -1) { // LCD Check procedure
        LCD_Addr = 0x3f;
        if (setup_LCD(LCD_Addr) == -1) {
            LCD_Addr = 0x20;
            if (setup_LCD(LCD_Addr) == -1) {
                while(1) {
                    PORTB ^= 0x20;  
                }
            }
        }
    }
    
    
    
    while(1) { 
        // Functions to display all LCD variables
       sensor_LCD(); 
      float tpScaler = convertAtoD(readAtoD()); // Function Call for time period scaler
      LCD_TP_Display(tpScaler);
      LCD_Timer((current_ms*tpScaler)/1000);
      buttonHandler(); // Calling the buttonHandler in the while loop so it is always updating flags
        
        switch(Speaker_Phase) { // Speaker Phases switch case
            case 1: // Alternating 5 tp buzzer
                OCR1A = 0;
                if(current_ms > buzzer_timer + 5000/tpScaler) {
                    OCR1A = 100;
                    buzzer_timer = current_ms;
                    Speaker_Phase = 2;
                }
                break;
                
            case 2: // Alternating 5 tp buzzer
                if(current_ms > buzzer_timer + 100/tpScaler) {
                    OCR1A = 0;
                    Speaker_Phase = 1;
                    buzzer_timer = current_ms;
                }
                
                    
                
                break;
                
            case 3: // Case 3 - 6 is the descending tone
                OCR1A = 100;
                if(current_ms > buzzer_timer + 200/tpScaler){
                    buzzer_timer = current_ms;
                    Speaker_Phase = 4;
                }
                
           break;   
            case 4:
                OCR1A = 1000;
                if(current_ms > buzzer_timer + 200/tpScaler){
                    
                    buzzer_timer = current_ms;
                    Speaker_Phase = 5;
                }
                
           break;   
            case 5:
                OCR1A = 3000;
                if(current_ms > buzzer_timer + 200/tpScaler){
                    buzzer_timer = current_ms;
                    Speaker_Phase = 6;
                }
                
           break;   
            case 6:
                OCR1A = 5000;
                if(current_ms > buzzer_timer + 200/tpScaler){
                    buzzer_timer = current_ms;
                    PORTB &= ~_BV(1);
                    Speaker_Phase = 7;
                }
           break;   
           
            case 7: // Case 7 and 8 are rapid chirp tone
                
                    OCR1A = 100;
                    PORTB |= _BV(1);
                    if(current_ms > buzzer_timer + 200/tpScaler) {
                        buzzer_timer = current_ms;
                        Speaker_Phase = 8;
                    }
                    break;
                    
            case 8: 
                OCR1A = 0;
               
                if(current_ms > buzzer_timer + 200/tpScaler){
                    buzzer_timer = current_ms;
                    Speaker_Phase = 7;
                }
                break;
                
        }
        
        switch(main_state){ // Main state machine, individual states encapsulated inside
            case HAZARD: 
                switch(HazardPhase) { // Hazard State Machine
                    prev_ms = current_ms;
                    case 1: // Lights on State
                        PORTD |= _BV(5);
                        PORTC |= _BV(2);
                        PORTB &= ~_BV(0);
                        PORTD &= ~_BV(6);
                        PORTC &= ~_BV(3);
                        SPI_Send_Command(0x12, 0x44);
                        SPI_Send_Command(0x13, 0x44);
                        char hzdStr[] = "HZDX";
                        LCD_Phase_Colour(hzdStr);
                        if(current_ms >= prev_ms + 1000) {
                            prev_ms = current_ms;
                            HazardPhase = 2;
                        }
                        break;
                        
                    case 2: // Lights off State
                        SPI_Send_Command(0x12, 0x00);
                        SPI_Send_Command(0x13, 0x00);
                        PORTD &= ~_BV(5);
                        PORTC &= ~_BV(2);
                        if(current_ms > 1000 + prev_ms){
                        HazardPhase = 1;
                        prev_ms = current_ms;
                        }
                        break;
                }
                
                if((current_ms >= Hazard_Timer + 10000) && HAZARD_Flag == 0){
                    BNS_Phase = 1;
                    main_state = BNS;
                    prev_ms = current_ms;
                   
                }
                break;
                
            case BNS: 
                
                switch(BNS_Phase) { // Broadway North South Traffic State Machine
                    
                    case 1: // Green
                        SPI_Send_Command(0x12, 0x28);
                        SPI_Send_Command(0x13, 0x82);
                        PORTD |= _BV(5);
                        PORTC |= _BV(2);
                        char bnsgStr[] = "BNSG";
                        LCD_Phase_Colour(bnsgStr);
                        
                        if((current_ms >= prev_ms + 8000/tpScaler) && (BS_Flag == 1 || BN_Flag == 0 || BNT_Flag == 0)) { // Minimum TP check maximum = 10 if other sensors, otherwise cycle BNS
                            if(current_ms >= (prev_ms + 10000/tpScaler)){ 
                                if(BST_Flag == 1 && BP_Flag == 0){ // Condition to move to BST
                                BNS_Phase = 4;
                                prev_ms = current_ms; 
                            } else if(BP_Flag == 1 || LS_Flag == 1 || LSP_Flag == 1){
                                    prev_ms = current_ms;
                                    BNS_Phase = 2;
                                }  
                                
                            }
                            }
                        
                        else if((current_ms >= prev_ms + 8000/tpScaler) && (BS_Flag == 0 && BN_Flag == 0 && BNT_Flag == 0)){
                                if(BST_Flag == 1 && BP_Flag == 0){ // Condition to move to BST
                                BNS_Phase = 4;
                                prev_ms = current_ms; 
                            } else if(BP_Flag == 1 || LS_Flag == 1 || LSP_Flag == 1){
                                    prev_ms = current_ms;
                                    BNS_Phase = 2;
                                }  
                        }
                        
                        break;
                        
                    case 2: // 2B From Phase Diagram - Yellow transition to states other than BST
                        SPI_Send_Command(0x12,0x24);
                        SPI_Send_Command(0x13, 0x42);
                        char bnsyStr[] = "BNSY";
                        LCD_Phase_Colour(bnsyStr);
                        
                        if(current_ms >= prev_ms + 3000/tpScaler){
                        BNS_Phase = 3;
                        prev_ms = current_ms;
                        }
                        break;
                        
                    case 3: // Red - To move to other Phases than BST
                        SPI_Send_Command(0x12, 0x22);
                        SPI_Send_Command(0x13, 0x22);
                        char bnsrStr[] = "BNSR";
                        LCD_Phase_Colour(bnsrStr);
                         if(current_ms >= prev_ms + 3000/tpScaler) {
                             if(LS_Flag == 1 && BP_Flag == 0 && BST_Flag == 0 ){
                                 main_state = LS;
                                 prev_ms = current_ms;
                                 LS_Phase = 1;
                             } else if(BP_Flag == 1 && BST_Flag == 1 ){
                                 BP_Phase = 2;
                                 main_state = BP;
                                 prev_ms = current_ms;
                                 
                                 Speaker_Phase = 3;
                                 
                             } else if (BP_Flag == 1 && BST_Flag == 0 ){
                                 BP_Phase = 1;
                                 main_state = BP;
                                 prev_ms = current_ms;
                                 Speaker_Phase = 3;

                                  // Below is a fringe case of constantly being in BNS due to no other cars and someone needing to cross LSP
                             } else if (LSP_Flag == 1 && BP_Flag == 0 && LS_Flag == 0 && BNT_Flag == 0) {
                                 main_state = LSP;
                                 prev_ms = current_ms;
                                 LSP_Phase = 1;
                                 Speaker_Phase = 3;
                             } 
                         }
                        break;
                        
                    case 4: // Yellow - To move to BST - Phase 2B from Phase Diagram
                        SPI_Send_Command(0x13, 0x42);
                        LCD_Phase_Colour(bnsyStr);
                        if(current_ms >= prev_ms + 3000/tpScaler) {
                            BNS_Phase = 5;
                            prev_ms = current_ms;
                        }
                        break;
                        
                    case 5: // Red - Phase 3B from Phase Diagram - To move to BST
                        LCD_Phase_Colour(bnsrStr);
                        SPI_Send_Command(0x13, 0x22);
                        if(current_ms >= prev_ms + 3000/tpScaler){
                        prev_ms = current_ms;
                        BST_Phase = 1;
                        main_state = BST;
                } 
                        
                        break;
                }
                
                        break;
                
                        
                    case BST:
                        switch (BST_Phase){ // BST State Machine
                            case 1: // Green
                                SPI_Send_Command(0x12, 0x88);
                                SPI_Send_Command(0x13, 0x22);
                                char bstgStr[] = "BSTG";
                                LCD_Phase_Colour(bstgStr);
                                PORTD |= _BV(5);
                                PORTC |= _BV(2);
                                if(current_ms >= prev_ms + 2000/tpScaler && BST_Flag == 1){
                                    if(current_ms >= prev_ms + 4000/tpScaler){
                                        if(LS_Flag == 1){
                                    prev_ms = current_ms;
                                    BST_Phase = 2;
                                } else {
                                            prev_ms = current_ms;
                                        BST_Phase = 4;
                                }
                                    }    
                                } else if(current_ms >= prev_ms + 2000/tpScaler && BST_Flag == 0){
                                        if(LS_Flag == 1){
                                    prev_ms = current_ms;
                                    BST_Phase = 2;
                                } else {
                                            prev_ms = current_ms;
                                        BST_Phase = 4;
                                }
                                }
                                break;

                                
                            case 2: // Yellow - 2A from Phase Diagram - To move to LS
                                SPI_Send_Command(0x12, 0x44);
                                char bstyStr[] = "BSTY";
                                LCD_Phase_Colour(bstyStr);
                                if(current_ms >= prev_ms + 3000/tpScaler) {
                                    prev_ms = current_ms;
                                    BST_Phase = 3;
                                }
                                
                              break; 
                              
                            case 3: // Red - 3A from Phase Diagram - Move to LS
                                SPI_Send_Command(0x12, 0x22);
                                char bstrStr[] = "BSTR";
                                LCD_Phase_Colour(bstrStr);
                                 if(current_ms >= prev_ms + 3000/tpScaler){
                                    prev_ms = current_ms;
                                    main_state = LS;
                                }
                                break;
                                
                                
                            case 4: // Yellow Moving back to BNS, Phase 2B from Phase Diagram
                                LCD_Phase_Colour(bstyStr);
                                SPI_Send_Command(0x12, 0x48);
                                if(current_ms >= prev_ms + 3000/tpScaler){
                                    prev_ms = current_ms;
                                    BST_Phase = 5;
                                }
                                break;
                                
                            case 5: // Red - Phase 3B from phase diagram - Back to BNS
                                LCD_Phase_Colour(bstrStr);
                                SPI_Send_Command(0x12, 0x28);
                                if(current_ms >= prev_ms + 3000/tpScaler){
                                    prev_ms = current_ms;
                                    BNS_Phase = 1;
                                    main_state = BNS;
                                }
                                
                              break;
                        }
                        break;
                        
            case LS:
                switch(LS_Phase){ // Little Street State machine
                    case 1: // Green
                        SPI_Send_Command(0x12, 0x22);
                        SPI_Send_Command(0x13, 0x28);
                        char lstgStr[] = "LSTG";
                        LCD_Phase_Colour(lstgStr);
                        PORTD |= _BV(5);
                        PORTC |= _BV(2);
                        if(current_ms >= prev_ms + 3000/tpScaler && LS_Flag == 1){
                            if(current_ms >= prev_ms + 5000/tpScaler){
                                    LS_Phase = 2;
                                    prev_ms = current_ms;
                            }
                            } else if(current_ms >= prev_ms + 3000/tpScaler && LS_Flag == 0){
                                    prev_ms = current_ms;
                                    LS_Phase = 2;
                            }
                        
                        break;
                    
                    case 2: // Yellow
                        SPI_Send_Command(0x13, 0x24);
                        char lstyStr[] = "LSTY";
                        LCD_Phase_Colour(lstyStr);
                        if(current_ms >= prev_ms + 3000/tpScaler){
                            prev_ms = current_ms;
                            LS_Phase = 3;
                        }
                        break;
                        
                    case 3: // Red - Move to other Phases
                        SPI_Send_Command(0x13, 0x22);
                        char lstrStr[] = "LSTR";
                        LCD_Phase_Colour(lstrStr);
                        if(current_ms >= prev_ms + 3000/tpScaler){
                            if(BST_Flag == 1 && BN_Flag == 0){
                                    prev_ms = current_ms;
                                    main_state = BST;
                                    BST_Phase = 1;
                                } else if(LSP_Flag == 1) { // Moving to Little Street Pedestrian to clear cars from main road and pedestrians
                                    main_state = LSP;
                                    prev_ms = current_ms;
                                    Speaker_Phase = 3;
                                } else { // Default returning to BNS when no sensors are pressed
                                    main_state = BNS;
                                    BNS_Phase = 1;
                                    prev_ms = current_ms;
                                }
                        }
                        break;
                }
                
                break;
                
            case LSP:
                if((PINC  & 0b00000010) != 0){ 
                            LSP_Flag = 0;
                             }  
                switch(LSP_Phase){ // Little Street Pedestrian State Machine
                    
                    case 1: // Green 

                        SPI_Send_Command(0x12, 0x28);
                        SPI_Send_Command(0x13, 0x82);
                        char lspgStr[] = "LSPG";
                    LCD_Phase_Colour(lspgStr);
                        PORTD |= _BV(5);
                        PORTB |= _BV(0);
                        PORTC |= _BV(3);
                        PORTC &= ~_BV(2);
                        
                        if(current_ms >= prev_ms + 5000/tpScaler ) { 
                            if(LS_Flag == 1){
                            prev_ms = current_ms;
                            LSP_Timer = current_ms;
                            LSP_Phase = 5;
                            } else {
                                prev_ms = current_ms;
                                LSP_Timer = current_ms;
                                LSP_Phase = 2;
                            }
                        }
                        
                        break;
                        
                    case 2: // Moving to BNS - Alternating Green Flashes
                        LCD_Phase_Colour(lspgStr);
                        PORTC &= ~_BV(3);
                        if(current_ms > prev_ms + 500/tpScaler){
                            prev_ms = current_ms;
                            LSP_Phase = 3;
                        }
                        break;
                        
                    case 3:  // Alternating Green Flashes
                        LCD_Phase_Colour(lspgStr);
                         PORTC |= _BV(3);
                         if(current_ms >= LSP_Timer + 3000/tpScaler){
                          PORTC &= ~_BV(3);
                          LSP_Phase = 4;   
                         }
                        if(current_ms > prev_ms + 500/tpScaler){
                            prev_ms = current_ms;
                            LSP_Phase = 2;
                        }
                         break;
                         
                    case 4:  // Red - Move to BNS
                        PORTC |= _BV(2);
                        char lsprStr[] = "LSPR";
                        LCD_Phase_Colour(lsprStr);
                        Speaker_Phase = 2;
                        
                        if(current_ms >= prev_ms + 3000/tpScaler) {
                            prev_ms = current_ms;
                            LSP_Timer = 0;
                            buzzer_timer = current_ms;
                            main_state = BNS;
                            BNS_Phase = 1;
                            
                        }
                        break;
                        
                    case 5: // Alternating Green Flashes - Moving to LS
                        LCD_Phase_Colour(lspgStr);
                        PORTC &= ~_BV(3);
                        SPI_Send_Command(0x12, 0x24);
                        SPI_Send_Command(0x13, 0x42);
                        
                        if(current_ms > prev_ms + 500/tpScaler){
                            prev_ms = current_ms;
                            LSP_Phase = 6;
                        }
                        break;
                        
                        
                    case 6: // Alternating Green Flashes - Moving to LS
                        LCD_Phase_Colour(lspgStr);
                        PORTC |= _BV(3);
                         if(current_ms >= LSP_Timer + 3000/tpScaler){
                          PORTC &= ~_BV(3);
                          LSP_Phase = 7;   
                         }
                        if(current_ms > prev_ms + 500/tpScaler){
                            prev_ms = current_ms;
                            LSP_Phase = 5;
                        }
                         break;
                         
                    case 7: // Red - Move to LS
                        LCD_Phase_Colour(lsprStr);
                        Speaker_Phase = 2;
                        PORTC |= _BV(2);
                        SPI_Send_Command(0x12,0x22);
                        SPI_Send_Command(0x13, 0x22);
                        if(current_ms >= prev_ms + 3000/tpScaler) {
                            prev_ms = current_ms;
                            LSP_Timer = 0;
                            main_state = LS;
                            LS_Phase = 1;
                            Speaker_Phase = 1;
                        }
                }
                break;
                
            case BP:
                if((PIND & 0b00010000) != 0) {
                            BP_Flag = 0;
                        }
                switch(BP_Phase){ // Broadway pedestrian state machine
                    case 1: // Green - Without BST On
                        PORTD |= _BV(6);
                        PORTC |= _BV(2);
                        PORTD &= ~_BV(5);
                        SPI_Send_Command(0x12, 0x22);
                        SPI_Send_Command(0x13, 0x22);
                        char bpdgStr[] = "BPDG";
                        LCD_Phase_Colour(bpdgStr);
                        if(current_ms  >= prev_ms + 7000/tpScaler) {
                           prev_ms = current_ms;
                           BP_Timer = current_ms;
                           BP_Phase = 3;
                        }
                        
                        break;
                        
                    case 2: // Green - With BST On
                        LCD_Phase_Colour(bpdgStr);
                        PORTD |= _BV(6);
                        PORTC |= _BV(2);
                        PORTD &= ~_BV(5);
                        SPI_Send_Command(0x12, 0x82);
                        SPI_Send_Command(0x13, 0x22);
                        if(current_ms  >= prev_ms + 7000/tpScaler) {
                           prev_ms = current_ms;
                           BP_Timer = current_ms;
                           BP_Phase = 5;
                        }
                        break;
                        
                    case 3: // Alternating Green Flashes 
                        LCD_Phase_Colour(bpdgStr);
                        PORTD &= ~_BV(6);
                        if(current_ms > prev_ms + 500/tpScaler){
                            prev_ms = current_ms;
                            BP_Phase = 4;
                        }
                        break;
                        
                    case 4: // Alternating Green Flashes
                        LCD_Phase_Colour(bpdgStr);
                         PORTD |= _BV(6);
                         if(current_ms >= BP_Timer + 3000/tpScaler){
                          PORTD &= ~_BV(6);
                          BP_Phase = 7;   
                         }
                        if(current_ms > prev_ms + 500/tpScaler){
                            prev_ms = current_ms;
                            BP_Phase = 3;
                        }
                         break;
                         
                    case 5: // Alternating Green Flashes
                        LCD_Phase_Colour(bpdgStr);
                        PORTD &= ~_BV(6);
                        SPI_Send_Command(0x12, 0x42);
                        if(current_ms > prev_ms + 500/tpScaler){
                            prev_ms = current_ms;
                            BP_Phase = 6;
                        }
                        break;
                                                
                        
                    case 6:  // Alternating Green Flashes
                        LCD_Phase_Colour(bpdgStr);
                         PORTD |= _BV(6);
                         if(current_ms >= BP_Timer + 3000/tpScaler){
                          PORTD &= ~_BV(6);
                          BP_Phase = 7;   
                         }
                        if(current_ms > prev_ms + 500/tpScaler){
                            prev_ms = current_ms;
                            BP_Phase = 5;
                        }
                        
                        break;
                         
                         
                    case 7: // Common end Case - Move to state based on buttons pressed
                        PORTD |= _BV(5);
                        SPI_Send_Command(0x12, 0x22);
                        char bpdrStr[] = "BPDR";
                        LCD_Phase_Colour(bpdrStr);
                        Speaker_Phase = 2;
                        
                        if(current_ms >= prev_ms + 3000/tpScaler){
                            if(LS_Flag == 1 && LSP_Flag == 0){
                                main_state = LS;
                                LS_Phase = 1;
                                prev_ms = current_ms;
                                BP_Timer = 0;
                                Speaker_Phase = 1;
                            } else if (LSP_Flag == 1){
                                main_state = LSP;
                                LSP_Phase = 1;
                                prev_ms = current_ms;
                                BP_Timer = 0;
                                Speaker_Phase = 1;
                            } else {
                            prev_ms = current_ms;
                            BP_Timer = 0;
                            main_state = BNS;
                            BNS_Phase = 1;
                            Speaker_Phase = 1;
                            }
                        }
                        break;
                }
                break;
                  
        }             
    }
}


