// This is a tester for Nokia 5110/3310 LCD display

#define NLCD_SCE 3
#define NLCD_RES 4
#define NLCD_DC 5
#define DATAOUT 11//MOSI
#define DATAIN  12//MISO 
#define SPICLOCK  13//sck
#define SLAVESELECT 10//ss

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
    //  Enable display controller (active low).
    digitalWrite( NLCD_SCE, 0 );
    //  Send data to display controller.
    SPIOUT(data);
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
    // !D/C data vs command
    digitalWrite( NLCD_DC, 1 );
    //  Enable display controller (active low).
    digitalWrite( NLCD_SCE, 0 );
    //  Send data to display controller.
    SPIOUT(cmd);
    //  Disable display controller.
    digitalWrite( NLCD_SCE, 1 );
    // !D/C leave as data
    digitalWrite( NLCD_DC, 0 );
}

/*--------------------------------------------------------------------------------------------------
  Name         :  LcdFill
  Description  :  Fills all display with the same byte n
  Argument(s)  :  n -> byte to be filled with
  Return value :  None.
--------------------------------------------------------------------------------------------------*/

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
#if NORMAL    
    const byte f = n;
#else    
    // as long as LSB of data is not 0 the communication/display will work
    // with 0 on lsb data it will hang (or the display of rows with 0 hangs the display).
    const byte f = n|1;
#endif    
    for( byte x = 0; x < 84; x++ )
    for( byte r = 0; r < 48/8; r++ )
    {
       SPIOUT( f );
    }
    digitalWrite( NLCD_SCE, 1 );
}  

/*--------------------------------------------------------------------------------------------------
  Name         :  LcdInit
  Description  :  Initialize the controller
  Argument(s)  :  None.
  Return value :  None.
--------------------------------------------------------------------------------------------------*/

static void LcdInit()
{
    digitalWrite( SLAVESELECT, 1 );
    //  Disable LCD controller
    digitalWrite( NLCD_SCE, 1 );
    //  Toggle display reset pin.
    digitalWrite( NLCD_RES, 0 );
    digitalWrite( NLCD_RES, 1 );
    delay(20);
    // Initialize
    LcdSendCmd( 0x21 );  // LCD Extended Commands.
    LcdSendCmd( 0xC8 );  // Set LCD Vop (Contrast).
    LcdSendCmd( 0x06 );  // Set Temp coefficent.
    LcdSendCmd( 0x13 );  // LCD bias mode 1:48.
    LcdSendCmd( 0x20 );  // LCD Standard Commands, Horizontal addressing mode.
    LcdSendCmd( 0x0C );  // LCD in normal mode.
}

//////////////////////////////////////////////////////////////////////////////////////////////////

void setup()
{
  Serial.begin(9600);
  Serial.println("Setup BEG");  

  pinMode( NLCD_RES, OUTPUT);
  pinMode( NLCD_SCE, OUTPUT);
  pinMode( NLCD_DC, OUTPUT);
  pinMode(DATAOUT, OUTPUT);
  pinMode(DATAIN, INPUT);
  pinMode(SPICLOCK,OUTPUT);
  pinMode(SLAVESELECT,OUTPUT);
  // setup SPI engine
  //interrupt disabled,spi enabled,msb 1st,master,clk low when idle,
  //sample on leading edge of clk,system clock/4 rate
  //SPCR = (1<<SPE)|(1<<MSTR)|(1<<CPOL)|(1<<CPHA);

  // Set the SPCR register to 01010000
  //  Enable SPI port: No interrupt, MSBit first, Master mode, CPOL->0, CPHA->0, Clk/4
  SPCR = 0x50;

  LcdInit();
  Serial.println("Setup END");  
}

//////////////////////////////////////////////////////////////////////////////////////////////////

byte n = 1;

void loop()
{
  LcdFill(n);
  if( n == 0 )
    n = 1;    
  else if( n & 128 )
    n = n<<1;
  else
    n = (n<<1) + 1;
  delay(1000);
}
