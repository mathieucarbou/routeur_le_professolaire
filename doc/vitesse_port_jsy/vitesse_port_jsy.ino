//  mise à jour de la vitesse du port TTL du capteur JSY - 194 de 9600 à 38400 //
//  * ESP32 + JSY-MK-194 (16 et 17)

#include <HardwareSerial.h>

#define RXD2 16
#define TXD2 17


void setup() {

  Serial.begin(115200);
  Serial2.begin(4800, SERIAL_8N1, RXD2, TXD2); //PORT DE CONNEXION AVEC LE CAPTEUR JSY-MK-194

             }


void loop() {

delay (60);

byte msg[] = {0x00,0x10,0x00,0x04,0x00,0x01,0x02,0x01,0x08,0xAA,0x12}; // passage du port TTL en 38400
// 00 10 00 04 00 01 02 01 06 2B D6 pour passer à 9600
// 00 10 00 04 00 01 02 01 08 AA 12 pour passer à 38400

 int i;
 int len=11; 

 //  envoie la requête pour passer le port de 9600 à 38400 //

 for(i = 0 ; i < len ; i++)
{
      Serial2.write(msg[i]); 
         
}
 len = 0;

 Serial.println("done.");
                   
            }     