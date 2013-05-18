

#define DATAOUT 11//MOSI
#define DATAIN  12//MISO 
#define SPICLOCK  13//sck
#define SLAVESELECT 10//ss for /LIS3LV02DQ, active low
#define RTC_CHIPSELECT 9// chip select (ss/ce) for RTC, active high

byte clr;

char spi_transfer(volatile char data)
{
  /*
  Writing to the SPDR register begins an SPI transaction
  */
  SPDR = data;
  /*
  Loop right here until the transaction is complete. the SPIF bit is 
  the SPI Interrupt Flag. When interrupts are enabled, and the 
  SPIE bit is set enabling SPI interrupts, this bit will set when
  the transaction is finished.
  */
  while (!(SPSR & (1<<SPIF)))     
  {};
  // received data appears in the SPDR register
  return SPDR;                    
}

void setup()
{
  char in_byte;
  clr = 0;
  in_byte = clr;
  Serial.begin(9600);
  
  // set direction of pins
  pinMode(DATAOUT, OUTPUT);
  pinMode(DATAIN, INPUT);
  pinMode(SPICLOCK,OUTPUT);
  pinMode(SLAVESELECT,OUTPUT);
  digitalWrite(SLAVESELECT,HIGH); //disable device
  pinMode(10, OUTPUT);
  // SPCR = 01010000
  // Set the SPCR register to 01010000
  //interrupt disabled,spi enabled,msb 1st,master,clk low when idle,
  //sample on leading edge of clk,system clock/4 rate
  SPCR = (1<<SPE)|(1<<MSTR)|(1<<CPOL)|(1<<CPHA);
  clr=SPSR;
  clr=SPDR;
  // query the WHO_AM_I register of the LIS3LV02DQ
  // this should return 0x3A, a factory setting
  in_byte = read_register(15);
  Serial.print("WHO_AM_I [");
  Serial.print(in_byte, HEX);
  Serial.println("]");
  
  // start up the device
  // this essentially activates the device, powers it on, enables all axes, and turn off the self test
  // CTRL_REG1 set to 10000111
  write_register(0x20, 135);
   // query the WHO_AM_I register of the LIS3LV02DQ
  // this should return 0x3A, a factory setting
  in_byte = read_register(15);
  Serial.print("WHO_AM_I [");
  Serial.print(in_byte, HEX);
  Serial.println("]"); 
  Serial.println("----");
  delay(250);    
}

// reads a register
char read_register(char register_name)
{
   char in_byte;
   // need to set bit 7 to indicate a read
   register_name |= 128;
   // SS is active low
   digitalWrite(SLAVESELECT, LOW);
   // send the address of the register we want to read first
   spi_transfer(register_name);
   // send nothing, but here's when the device sends back the register's value as an 8 bit byte
   in_byte = spi_transfer(0);
   // deselect the device
   digitalWrite(SLAVESELECT, HIGH); 
   return in_byte;
}

// write to a register
void write_register(char register_name, byte data)
{
  // char in_byte;
   // clear bit 7 to indicate we're doing a write
   register_name &= 127;
   // SS is active low
   digitalWrite(SLAVESELECT, LOW);
   // send the address of the register we want to write
   spi_transfer(register_name);
   // send the data we're writing
   spi_transfer(data);
   digitalWrite(SLAVESELECT, HIGH);
   //return in_byte;
}

void loop()
{
  byte in_byte;
  /*
  int x_val, y_val, z_val;
  byte x_val_l, x_val_h, y_val_l, y_val_h, z_val_l, z_val_h;
  
  // read the outx_h register
  x_val_h = read_register(0x29); //Read outx_h 
  // high four bits are just the sign in 12 bit mode
  if((x_val_h & 0xF0) > 0) {
    Serial.print("NEG_X");
  }
  // comment this if you care about the sign, otherwise we're getting absolute values
  x_val_h &= 0X0F;
  //Serial.print("x_h="); Serial.print(x_val_h, DEC); Serial.print(", ");
  
  // read the outy_h register
  x_val_l  = read_register(0x28);
  
  //Serial.print("x_l="); Serial.print(x_val_l, DEC); Serial.print(", ");
  x_val = x_val_h;
  x_val <<= 8;
  x_val += x_val_l;

  // the LIS3LV02DQ according to specs, these values are:
  // 2g = 2^12/2 = 2048
  // 1g = 1024
  // if you use the sign, that gives a range of +/-2g should output +/-2048 
    
  Serial.print("x_val="); Serial.print(x_val, DEC);

  y_val_h = read_register(0x2B); //Read outx_h 
  y_val_l = read_register(0x2A); //Read outx_l
  y_val = y_val_h;
  y_val <<= 8;
  y_val += y_val_l;
 Serial.print(" y_val="); Serial.print(y_val, DEC);


  z_val_h = read_register(0x2D); //Read outz_h
 // Serial.print("z_h="); Serial.print(z_val_h, DEC); Serial.print(", ");
  z_val_l = read_register(0x2C); //Read outz_l
 // Serial.print("z_l="); Serial.print(z_val_l, DEC); Serial.print(", ");
  z_val = z_val_h;
  z_val <<= 8;
  //Serial.print("z_h_<<8="); Serial.print(z_val_h, DEC); Serial.print(", ");
  //z_val += z_val_l; 
  
  //long g_z; // say approx 100 cm/s/s
  //g_z = z_val * 10 / 1024;
  
  //Serial.print(" z_val="); Serial.println(z_val, DEC);

  //delay(500);
  */
  digitalWrite(10, HIGH);
  in_byte = read_register(15);
  Serial.print("WHO_AM_I [");
  Serial.print(in_byte, HEX);
  Serial.println("]"); 
  delay(10);
    
}
