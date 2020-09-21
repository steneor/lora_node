#include <Arduino.h>
/************************** Configuration ***********************************/
#define EU863 ///< Used in Europe

//#define DEBUG 1

#include <TinyLoRa.h>
#include <SPI.h>
#include <LowPower.h>

#include <TinyGPS++.h>
#include <AltSoftSerial.h>
static const int RXPin = 8, TXPin = 9;
static const uint32_t GPSBaud = 9600;

uint8_t coords[11];
TinyGPSPlus gps;
AltSoftSerial ss(RXPin, TXPin); // RX, TX

// Données à envoyer vers TTN (payload)
unsigned char loraData[19] = {0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20,0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20,0x20, 0x20, 0x20, 0x20};


void get_coords () {
  bool newData = false;
  // unsigned long chars;
  // unsigned short sentences, failed;
  float flat,flon,faltitudeGPS,fhdopGPS; 
  // unsigned long age;

  // For one second we parse GPS data and report some key values

  for (unsigned long start = millis(); millis() - start < 1000;) {
    while (ss.available()) {
      char c = ss.read();
      Serial.write(c); // uncomment this line if you want to see the GPS data flowing
      if (gps.encode(c)) { // Did a new valid sentence come in?
        newData = true;
      }
    }
  }

 if ( newData ) {
    flat=gps.location.lat();
    flon=gps.location.lng();
    if (gps.altitude.isValid())
         faltitudeGPS = gps.altitude.meters();
     else
        faltitudeGPS=0;
    fhdopGPS = gps.hdop.value();  
    Serial.print("  -  lat : ");Serial.println(flat);  
    Serial.print("  -  lon : ");Serial.println(flon);  
    Serial.print("  -  alt : ");Serial.println(faltitudeGPS);
    memcpy((loraData+7), &flat, sizeof(float));
    memcpy((loraData+11), &flon, sizeof(float));
    memcpy((loraData+15), &faltitudeGPS, sizeof(float));

}

  //gps.stats(&chars, &sentences, &failed);

  int32_t lat = flat * 10000;
  int32_t lon = flon * 10000;
  int16_t altitudeGPS = faltitudeGPS * 100;
  int8_t hdopGPS = fhdopGPS;
 
  // channel = 0x01;
  // coords[0] = channel; 
  // coords[1] = LPP_GPS; 

  coords[2] = lat >> 16;
  coords[3] = lat >> 8;
  coords[4] = lat;

  coords[5] = lon >> 16; 
  coords[6] = lon >> 8;
  coords[7] = lon;

  coords[8] = altitudeGPS;
  coords[9] = altitudeGPS >> 8;
  coords[10] = hdopGPS;


}

//# define LED_BUILTIN 7
const int batterie = A0; // pont diviseur tension batterie 1M 330K

// Network Session Key (MSB)
#define DefNwkSkey {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // MSB

// Application Session Key (MSB)
#define DefAppSkey {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // MSB

// Device Address (MSB)
#define DefDevAddr {0x00, 0x00, 0x00, 0x00}; // msb


// Network Session Key (MSB)
uint8_t NwkSkey[16] = DefNwkSkey ; // MSB

// Application Session Key (MSB)
uint8_t AppSkey[16] = DefAppSkey ;// MSB

// Device Address (MSB)
uint8_t DevAddr[4] = DefDevAddr; // msb

/************************** Configuration mesure batterie ***********************************/
// exemple pont diviseur: 1M, 330K divider across battery and using internal ADC ref of 1.1V
// Sense point is bypassed with 0.1 uF cap to reduce noise at that point
// ((1e6+330e3)/330e3)*1.1 = Vmax = 4.4333333 Volts
// 4.4333/1023 = Volts per bit = 0.0004333659

// (1940 / 470) * 1.1 = 4.5404255 votls  / 1023 = 0.0044383436 volts par bit

#define VOLTS_PAR_BITS 0.0044383436
struct batteryCapacity
{
  unsigned voltage;
  int capacity;
};

const batteryCapacity remainingCapacity[] = {
    unsigned(4.20 / VOLTS_PAR_BITS),    100,
    unsigned(4.10 / VOLTS_PAR_BITS),    96,
    unsigned(4.00 / VOLTS_PAR_BITS),    92,
    unsigned(3.96 / VOLTS_PAR_BITS),    89,
    unsigned(3.92 / VOLTS_PAR_BITS),    85,
    unsigned(3.89 / VOLTS_PAR_BITS),    81,
    unsigned(3.86 / VOLTS_PAR_BITS),    77,
    unsigned(3.83 / VOLTS_PAR_BITS),    73,
    unsigned(3.80 / VOLTS_PAR_BITS),    69,
    unsigned(3.77 / VOLTS_PAR_BITS),    65,
    unsigned(3.75 / VOLTS_PAR_BITS),    62,
    unsigned(3.72 / VOLTS_PAR_BITS),    58,
    unsigned(3.70 / VOLTS_PAR_BITS),    55,
    unsigned(3.66 / VOLTS_PAR_BITS),    51,
    unsigned(3.62 / VOLTS_PAR_BITS),    47,
    unsigned(3.58 / VOLTS_PAR_BITS),    43,
    unsigned(3.55 / VOLTS_PAR_BITS),    40,
    unsigned(3.51 / VOLTS_PAR_BITS),    35,
    unsigned(3.48 / VOLTS_PAR_BITS),    32,
    unsigned(3.44 / VOLTS_PAR_BITS),    26,
    unsigned(3.40 / VOLTS_PAR_BITS),    24,
    unsigned(3.37 / VOLTS_PAR_BITS),    20,
    unsigned(3.35 / VOLTS_PAR_BITS),    17,
    unsigned(3.27 / VOLTS_PAR_BITS),    13,
    unsigned(3.20 / VOLTS_PAR_BITS),    9,
    unsigned(3.10 / VOLTS_PAR_BITS),    6,
    unsigned(3.00 / VOLTS_PAR_BITS),    3,
};

const int ncell = sizeof(remainingCapacity) / sizeof(struct batteryCapacity);

unsigned int getBatteryCapacity(unsigned lecture_dac)
{
  for (int i = 0; i < ncell; i++)
  {
    if (lecture_dac > remainingCapacity[i].voltage)
    {
      return remainingCapacity[i].capacity;
    }
  }
  return 0;
}

#include <OneWire.h>
#include <DallasTemperature.h>

// GPIO where the DS18B20 is connected to
const int oneWireBus = 6; //6 or 9 ok
// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(oneWireBus);

// Pass our oneWire reference to Dallas Temperature sensor 
DallasTemperature sensors(&oneWire);

// délai entre deux émissions en seconde
// pour 10 minutes -> cad 600 s / 8 s = 75
//pour 2 minutes -> cad 120 s / 8 = 15
unsigned int sleepCounter = 7; //sleepCounter ->lowpower de 8S pendant n fois sleepCounter

//  TinyLoRa(irq,ss)
//TinyLoRa lora = TinyLoRa(26, 18); // TTGO32
TinyLoRa lora = TinyLoRa(2, 10, 5); // Ernesto cablage

/*
 *
 */
void setup()
{
  delay(2000);
  Serial.begin(115200);
  //while (! Serial);
  ss.begin(GPSBaud);

  //Configure la tension de référence 1.1V utilisée avec les entrées analogiques.
  analogReference(INTERNAL);

  // Start the DS18B20 sensor
  sensors.begin();

  // Led de la carte
  //pinMode(LED_BUILTIN, OUTPUT);

  Serial.print("lancement LoRaWAN...");

  // configurer (canal, étalement, bande passante) comme fait avec la passerelle monocanal
  lora.setChannel(CH0);
  //lora.setDatarate(SF9BW125);
  lora.setDatarate(SF7BW125);
  if (!lora.begin())
  {
    Serial.println("Echec");
    while (true)
      ;
  }
  Serial.println("OK");
}

/*
 *
 */
void loop()
{
  get_coords() ;
  sensors.requestTemperatures(); 
  float tempC = sensors.getTempCByIndex(0);
  Serial.print(tempC);
  Serial.println("ºC");
  memcpy((loraData+3), &tempC, sizeof(float));

  Serial.println("Envoi données vers TTN, Device: devicefish ... ");

  uint16_t vbatt = analogRead(batterie);
  //float battVolt = vbatt * VOLTS_PAR_BITS;
  int capacity = getBatteryCapacity(vbatt);
#if DEBUG==1
  Serial.print("Lecture vbatt : ");
  Serial.print(vbatt);
  Serial.print("  -  Lecture V Baterie : ");
  Serial.print(battVolt);
  Serial.print("V");
  Serial.print("  -  capacite : ");
  Serial.print(capacity);
  Serial.println("%");
#endif  //DEBUG

  //envoi 3 octets (paquet reformaté à convenance dans  la console TTN)
  loraData[0] = highByte(vbatt);
  loraData[1] = lowByte(vbatt);
  loraData[2] = lowByte(capacity);
  //Serial.print("loraData : "); //Serial.println(loraData);

  lora.sendData(loraData, sizeof(loraData), lora.frameCounter);

  Serial.print("lora.sendData fait ...");
  Serial.print("  -  Compteur Trames : ");
  Serial.println(lora.frameCounter);
  lora.frameCounter++;

  // del témoin d'envoi
  // pinMode(LED_BUILTIN, OUTPUT);

  // digitalWrite(LED_BUILTIN, HIGH);
  // delay(1000);
  // digitalWrite(LED_BUILTIN, LOW);
  // delay(1000);

  Serial.print("temporisation : ");
  Serial.print(sleepCounter * 8);
  Serial.println(" secondes");

  delay(10);

  unsigned int counter;
  counter = sleepCounter;
  while (counter > 0)
  {
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
    counter--;
  }
  delay(10);
}
