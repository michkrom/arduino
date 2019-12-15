/*
 * Solar Charge Controller
 * by Michal Krombholz
 *
 */

void setup()
{
  // begin the serial communication
  Serial.begin(9600);
  Serial.println("Solar Charge Controller");
}

int readBatteryVoltage_mV()
{
  int32_t dac = 0;
  const int32_t N = 256;
  for(auto n = N; n > 0; --n) dac += analogRead(A0);
  // empiricly measured 243 counts and 12.25V (via 100k/10k divider)
  int32_t mV = (dac*1225)/(N*243);
  return mV;
}


void loop()
{
  // check if data has been sent from the computer
  if (Serial.available()) {
    // read the most recent byte (which will be from 0 to 255)
    byte in = Serial.read();
  }
  auto vbat_mV = readBatteryVoltage_mV();
  Serial.print(vbat_mV);
  Serial.println("mV");
}
