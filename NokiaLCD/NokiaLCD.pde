#include <Servo.h>

#define LIB 0

#if LIB
#include <NokiaLCD.h>
#else

#define NLCD_SCE 3
#define NLCD_RES 4
#define NLCD_DC 5
#define DATAOUT 11//MOSI
#define DATAIN  12//MISO 
#define SPICLOCK  13//sck
#define SLAVESELECT 10//ss

#endif
/*
PB5=D13=SCK=#7
PB4=D12=MISO=#--
PB3=D11=MOSI=#6 
/*--------------------------------------------------------------------------------------------------
                                Private functions
--------------------------------------------------------------------------------------------------*/
#define SPIOUT(data) { SPDR = data;  while (!(SPSR & (1<<SPIF))); }

/*--------------------------------------------------------------------------------------------------
  Name         :  LcdSend
  Description  :  Sends data to display controller.
  Argument(s)  :  data -> Data to be sent
  Return value :  None.
--------------------------------------------------------------------------------------------------*/
static void LcdSend( byte data )
{
    // !D/C data vs command
    digitalWrite( NLCD_DC, 0 );
    //  Enable display controller (active low).
    digitalWrite( NLCD_SCE, 0 );
    //  Send data to display controller.
    SPDR = data;
    //  Wait until Tx register empty.
    while (!(SPSR & (1<<SPIF)));
    //  Disable display controller.
    digitalWrite( NLCD_SCE, 1 );
}


/*--------------------------------------------------------------------------------------------------
  Name         :  LcdSend
  Description  :  Sends data to display controller.
  Argument(s)  :  data -> Data to be sent
  Return value :  None.
--------------------------------------------------------------------------------------------------*/
static void LcdSendCmd( byte cmd )
{
    //  Enable display controller (active low).
    digitalWrite( NLCD_SCE, 0 );
    // !D/C data vs command
    digitalWrite( NLCD_DC, 1 );
    //  Send data to display controller.
    SPDR = cmd;
    //  Wait until Tx register empty.
    while (!(SPSR & (1<<SPIF)));
    //  Disable display controller.
    digitalWrite( NLCD_SCE, 1 );
}


static void LcdFill(byte n)
{
    Serial.print("fill->");Serial.println(n,DEC);  
    //  Set base address 
    LcdSendCmd( 0x80 | 0 );
    LcdSendCmd( 0x40 | 0 );
    // !D/C data vs command
    digitalWrite( NLCD_DC, 0 );
    //  Enable display controller (active low).
    digitalWrite( NLCD_SCE, 0 );
    for( byte x = 0; x < 84; x++ )
 //   for( byte y = 0; y < 48; y++ )
    {
//       SPIOUT( n );
      LcdSend(n+x);
    }
    digitalWrite( NLCD_SCE, 1 );
}  


static void LcdInit()
{
    digitalWrite( SLAVESELECT, 1 );
    //  Disable LCD controller
    digitalWrite( NLCD_SCE, 1 );
    //  Toggle display reset pin.
    digitalWrite( NLCD_RES, 0 );
    digitalWrite( NLCD_RES, 1 );
    delay(100);

//    LcdSendCmd( 0x21 );  // LCD Extended Commands.
//    LcdSendCmd( 0xC8 );  // Set LCD Vop (Contrast).
//    LcdSendCmd( 0x06 );  // Set Temp coefficent.
//    LcdSendCmd( 0x13 );  // LCD bias mode 1:48.
    LcdSendCmd( 0x20 );  // LCD Standard Commands, Horizontal addressing mode.
    LcdSendCmd( 0x0C );  // LCD in normal mode.
}


void setup()
{
  Serial.begin(9600);
  Serial.println("Setup BEG");  
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object 

#if LIB
  LcdInit();
  Serial.println("LINES");
  LcdLine(0,40,0,40,PIXEL_ON);
  LcdLine(40,0,40,0,PIXEL_ON);
  LcdChr(FONT_1X,'X');
  LcdStr(FONT_1X,"Hello world!");
  LcdGotoXY(0,20);
  LcdChr(FONT_2X,'X');
  LcdStr(FONT_2X,"Hello world!");
#else
	
	pinMode( NLCD_RES, OUTPUT);
	pinMode( NLCD_SCE, OUTPUT);
	pinMode( NLCD_DC, OUTPUT);
	pinMode(DATAOUT, OUTPUT);
	pinMode(DATAIN, INPUT);
	pinMode(SPICLOCK,OUTPUT);
	pinMode(SLAVESELECT,OUTPUT);

	//interrupt disabled,spi enabled,msb 1st,master,clk low when idle,
	//sample on leading edge of clk,system clock/4 rate
	//SPCR = (1<<SPE)|(1<<MSTR)|(1<<CPOL)|(1<<CPHA);

	// Set the SPCR register to 01010000
	//  Enable SPI port: No interrupt, MSBit first, Master mode, CPOL->0, CPHA->0, Clk/4
	SPCR = 0x50;

    LcdInit();
    	
    delay(100);
    LcdFill(255);
    delay(1000);
    LcdFill(0);
    delay(1000);
    LcdFill(0xAA);
    delay(1000);
    LcdFill(0x55);
    delay(1000);

#endif
  Serial.println("Setup END");  
}


byte n = 1;
char dir = -10;
int pos = 90;
#define SERVO 1

void loop()
{    
  Serial.println(2L*(3300L*analogRead(0))/1024L,DEC);
  Serial.println((3300L*analogRead(3))/1024L,DEC);
  Serial.println(pos);
#if SERVO
  myservo.write(pos);
  pos+=dir;
  if(pos>180) { pos=180; dir = -dir; }
  if(pos<0) { pos=0; dir = -dir; }
#endif
#if !LIB 
//  Serial.println(n,DEC);    
//  LcdInit();
  LcdFill(n);
  delay(1000);
  n++;
  if(n==0)
  {
    LcdInit();
    n=1;
  }
#else
  LcdClear();
  LcdGotoXY(n,n);
  LcdChr(FONT_2X,'X');
  LcdStr(FONT_2X,"Hello world!");
  n++;
  if(n<48)n=0;
#endif
}
 
