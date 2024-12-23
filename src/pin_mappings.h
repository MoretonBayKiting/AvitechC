// Comment in each line below is from BASCOM (after Code Pilot converted it to C).  Although Code Pilot confirmed appropriateness
// for atmega328PB, I'm not convinced.  I did the mapping based on C:\D\NonSML\AviTech\Atmega328PB.png copies from BASCOM reference.
// First attempt here replaced (courtesy of code pilot) below - assumes using avr DDR registers.
// #define X_ENABLEPIN 9   //PORTD5
// #define Y_ENABLEPIN 32  //PORTD2
// #define X_DIR 11        //PORTD7
// #define Y_DIR 2         //PORTD4
// #define X_STEP 10       //PORTD6
// #define Y_STEP  1       //PORTD3
// #define BUZZER  2       //PORTE0
// #define TILTSTOP 13     //PINB1
// #define PANSTOP 14      //PINB2
// #define LASER2 22       //PORTE3
// #define FAN 6           //PORTE1

#define X_ENABLEPIN PD5
#define Y_ENABLEPIN PD2
#define X_DIR PD7
#define Y_DIR PD4
#define X_STEP PD6
#define Y_STEP PD3
#define BUZZER PE0
#define TILT_STOP PB1
#define PAN_STOP PB2

#define BT_TX_PIN PD0 // Bluetooth HC-05
#define BT_RX_PIN PD1
#define SCL PC5
#define SDA PC4

#define LASER2 PE3
#define FAN PE1

// Setup pins that are not used on the board. Each pin has a 10k (for safety)resistor tied to VCC
#define PINC_3 PC3
#define PINE_1 PE1
#define PINE_2 PE2
#define PORTE_3 PE3
#define PORTE_1 PE1