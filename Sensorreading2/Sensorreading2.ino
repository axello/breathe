/* 
  Sensor Reading
  Breate / Air Quality Egg workshop
  Nov. 2011 - Jan. 2012
  
  Equipment: Nanode, 
  Sensors: MQ-135 (CO2), MQ7 (CO), Sensirion SHT71 (Humidity & temp)
  
  by Axel Roest, 
  Created 12 Jan. 2012

  ToDo:
  We should cycle both gas sensor heaters using a powerfet or transistor. 
  Especially the MG-811 needs a duty cycle of about 30%
  
*/

// If using a Nanode (www.nanode.eu) instead of Arduino and ENC28J60 EtherShield then
// use this define:
#define NANODE

#include "EtherShield.h"
#ifdef NANODE
#include <NanodeMAC.h>
#endif

#include "pachube.h"
#include "Sensirion.h"

/*********************************************
    Pachube constants
*********************************************/
 // change this to your API key (defined in pachube.h file)
#ifndef PACHUBE_API_KEY
#define PACHUBE_API_KEY "PASTE_YOUR_API_KEY_HERE_BETWEEN_THE_QUOTES"
#endif

// change this to the relative URL of your feed
#define HTTPFEEDPATH "/v2/feeds/44332"

// global string buffer for hostname message:
#define FEEDHOSTNAME "api.pachube.com\r\nX-PachubeApiKey: " PACHUBE_API_KEY
#define FEEDWEBSERVER_VHOST "api.pachube.com"
static char hoststr[150] = FEEDWEBSERVER_VHOST;

// change the template to be consistent with your datastreams: see http://api.pachube.com/v2/
#define FEED_POST_MAX_LENGTH 512
static char feedTemplate[] = "{\"version\":\"1.0.0\",\"datastreams\":[{\"id\":\"PL1\", \"current_value\":\"%f\"},{\"id\":\"PL2\",\"current_value\":\"%f\"},{\"id\":\"PL3\",\"current_value\":\"%f\"},{\"id\":\"PL4\",\"current_value\":\"%f\"}]}";
static char feedPost[FEED_POST_MAX_LENGTH] = {0}; // this will hold your filled out template

long lastPostTimestamp;

/*********************************************
    Sensor constants
*********************************************/
#define SensorCo2 0
#define SensorCO 1
#define ReadingLed 5
#define NanodeLed 6

// resistor values (appr.) for the ground resistors of the divider
#define RCo2  30000.0
#define RCO   5000.0
#define  R0_Co2 30000.0
#define  R0_CO -26400.0

#define dataPin  A4
#define clockPin A5

/*********************************************
    Ethernet Shield constants & global variables
*********************************************/
#define USE_DHCP

#ifdef NANODE
static uint8_t mymac[6] = { 0,0,0,0,0,0 };
#else
static uint8_t mymac[6] = { 0x41,0x58,0x45,0x4C,0x34,0x56 };
#endif


static uint8_t myip[4] = { 0,0,0,0 };
static uint8_t mynetmask[4] = { 0,0,0,0 };

// IP address of the host being queried to contact (IP of the first portion of the URL):
static uint8_t websrvip[4] = { 0, 0, 0, 0 };

// Default gateway. The ip address of your DSL router. It can be set to the same as
// websrvip the case where there is no default GW to access the 
// web server (=web server is on the same lan as this host) 
static uint8_t gwip[4] = { 0,0,0,0};

static uint8_t dnsip[4] = { 0,0,0,0 };
static uint8_t dhcpsvrip[4] = { 0,0,0,0 };

#define DHCPLED 6

// listen port for tcp/www:
#define MYWWWPORT 80

#define BUFFER_SIZE 750
static uint8_t buf[BUFFER_SIZE+1];

// Programmable delay for flashing LED
uint16_t delayRate = 0;

EtherShield es=EtherShield();
#ifdef NANODE
NanodeMAC mac( mymac );
#endif


/*********************************************
    Global variables
*********************************************/

int co2raw;
int coraw;
float co2;
float co;

float temperature;
float humidity;
float dewpoint;

Sensirion tempSensor = Sensirion(dataPin, clockPin);

void setup()
{
  pinMode(ReadingLed, OUTPUT);  // Set servo pin as an output pin
  pinMode(NanodeLed, OUTPUT);  // Set servo pin as an output pin
  // finished setup, light led
  digitalWrite(ReadingLed, HIGH);
  
  Serial.begin(2400);
  Serial.println("Ready to init ethernet shield");
  delay(500);
  initialiseEthernetShield();

}


void loop()
{
  long currentTime = millis();
  digitalWrite(ReadingLed,HIGH);            // toggle measuring led
  int plen = es.ES_enc28j60PacketReceive(BUFFER_SIZE, buf);
  es.ES_packetloop_icmp_tcp(buf,plen);

 /* tempSensor.measure(&temperature, &humidity, &dewpoint);
  serialPrintHumidTempValues();

  readGasValues();
  serialPrintGasValues();
 */ 
  if(fillOutTemplateWithSensorValues(1, temperature, humidity, co2, co)){
      es.ES_client_http_post(PSTR(HTTPFEEDPATH),PSTR(FEEDWEBSERVER_VHOST),PSTR(FEEDHOSTNAME), PSTR("PUT "), feedPost, &sensor_feed_post_callback);    
  }

  digitalWrite(ReadingLed,LOW);  
  delay(5000);
  // do something with   delay(delayRate);
    lastPostTimestamp = currentTime;

}

/* Pachube 
*/
uint8_t fillOutTemplateWithSensorValues(uint8_t node_id, float sensorValue1, float sensorValue2, float sensorValue3, float sensorValue4){
  // change this function to be consistent with your feed template, it will be passed the node id and four sensor values by the sketch
  // if you return (1) this the sketch will post the contents of feedPost to Pachube, if you return (0) it will not post to Pachube
  // you may use as much of the passed information as you need to fill out the template
  
  snprintf(feedPost, FEED_POST_MAX_LENGTH, feedTemplate, sensorValue1, sensorValue2, sensorValue3, sensorValue4); // this simply populates the current_value filed with sensorValue1
  return (1);
}

/* see Excel calculation and trendline
  y (Rs/R0) = a * x^b
  
  ==> x = pow(y/a , 1/b)
  */
  
float ppm(float y, float a, float b)
{
  return pow(y/a,1/b);
}
void readGasValues(void)
{
  float RSco2, RSco;
  
  co2raw=analogRead(SensorCo2);
  RSco2 = (1024.0 * RCo2 / (float)co2raw ) - RCo2;        // calculate voltage-divider value
  // do exponential conversion 
  // TODO compensate for RH, T and stuff
  co2 = ppm(RSco2/R0_Co2, 5.1633, -0.35);                  // see Excel and/or Datasheet for a,b values
  coraw=analogRead(SensorCO);
  RSco = (1024.0 * RCO / (float)coraw ) - RCO;
  co = ppm(RSco/R0_CO, 464.43, -0.061);
  
}

void serialPrintGasValues(void)
{
  Serial.print("CO2:");
  serialPrintFloat(co2);
  Serial.print(" ppm, CO:");
  serialPrintFloat(co);
  Serial.println(" ppm");
}

void serialPrintHumidTempValues(void)
{
  Serial.print("Temperature: ");
  serialPrintFloat(temperature);
  Serial.print(" C, Humidity: ");
  serialPrintFloat(humidity);
  Serial.print(" %, Dewpoint: ");
  serialPrintFloat(dewpoint);
  Serial.println(" C");
}

void serialPrintFloat(float f){
  Serial.print((int)f);
  Serial.print(".");
  int decplace = (f - (int)f) * 100;
  Serial.print(abs(decplace));
}

/*
  Ethershield Code
*/

void acquireIPAddress(void)
{
 uint16_t dat_p;
  long lastDhcpRequest = millis();
  uint8_t dhcpState = 0;
  Serial.println("Sending initial DHCP Discover");
  es.ES_dhcp_start( buf, mymac, myip, mynetmask,gwip, dnsip, dhcpsvrip );

  while(1) {
    // handle ping and wait for a tcp packet
    int plen = es.ES_enc28j60PacketReceive(BUFFER_SIZE, buf);

    dat_p=es.ES_packetloop_icmp_tcp(buf,plen);
    //    dat_p=es.ES_packetloop_icmp_tcp(buf,es.ES_enc28j60PacketReceive(BUFFER_SIZE, buf));
    if(dat_p==0) {
      int retstat = es.ES_check_for_dhcp_answer( buf, plen);
      dhcpState = es.ES_dhcp_state();
      // we are idle here
      if( dhcpState != DHCP_STATE_OK ) {
        if (millis() > (lastDhcpRequest + 10000L) ){
          lastDhcpRequest = millis();
          // send dhcp
          Serial.println("Sending DHCP Discover");
          es.ES_dhcp_start( buf, mymac, myip, mynetmask,gwip, dnsip, dhcpsvrip );
        }
      } 
      else {
        return;        
      }
    }
  }   
}

void initialiseEthernetShield()
{
  // Initialise SPI interface
  es.ES_enc28j60SpiInit();

  // initialize enc28j60
  Serial.println("Init ENC28J60");
#ifdef NANODE
    es.ES_enc28j60Init(mymac,8);
#else
    es.ES_enc28j60Init(mymac);
#endif

  Serial.println("Init done");
  
  Serial.print( "ENC28J60 version " );
  Serial.println( es.ES_enc28j60Revision(), HEX);
  if( es.ES_enc28j60Revision() <= 0 ) {
    Serial.println( "Failed to access ENC28J60");
 
    while(1);    // Just loop here
 }

#ifdef USE_DHCP
  acquireIPAddress();
#endif
   printNetworkParameters();

  //init the ethernet/ip layer:
  es.ES_init_ip_arp_udp_tcp(mymac,myip, 80);

  // init the web client:
  es.ES_client_set_gwip(gwip);  // e.g internal IP of dsl router
  es.ES_dnslkup_set_dnsip(dnsip); // generally same IP as router
  
  Serial.println("Awaiting Client Gateway");
  while(es.ES_client_waiting_gw()){
    int plen = es.ES_enc28j60PacketReceive(BUFFER_SIZE, buf);
    es.ES_packetloop_icmp_tcp(buf,plen);    
  }

  Serial.println("Client Gateway Complete, Resolving Host");

  resolveHost(hoststr, websrvip);
  Serial.print("Resolved host: ");
  Serial.print(hoststr);
  Serial.print(" to IP: ");
  printIP(websrvip);
  Serial.println();
  
  es.ES_client_set_wwwip(websrvip);

  lastPostTimestamp = millis();

}

// hostName is an input parameter, ipAddress is an outputParame
void resolveHost(char *hostName, uint8_t *ipAddress){
  es.ES_dnslkup_request(buf, (uint8_t*)hostName );
  while(1){
    int plen = es.ES_enc28j60PacketReceive(BUFFER_SIZE, buf);
    es.ES_packetloop_icmp_tcp(buf,plen);   
    if(es.ES_udp_client_check_for_dns_answer(buf, plen)) {
      uint8_t *websrvipptr = es.ES_dnslkup_getip();
      for(int on=0; on <4; on++ ) {
        ipAddress[on] = *websrvipptr++;
      }     
      return;
    }    
  }
}  

void printNetworkParameters(void)
{
    Serial.print( "My IP: " );
    printIP( myip );
    Serial.println();

    Serial.print( "Netmask: " );
    printIP( mynetmask );
    Serial.println();

    Serial.print( "DNS IP: " );
    printIP( dnsip );
    Serial.println();

    Serial.print( "GW IP: " );
    printIP( gwip );
    Serial.println();
   
    digitalWrite( DHCPLED, HIGH);
}

 // Output a ip address from buffer
void printIP( uint8_t *buf ) {
  for( int i = 0; i < 4; i++ ) {
    Serial.print( buf[i], DEC );
    if( i<3 )
      Serial.print( "." );
  }
}


void sensor_feed_post_callback(uint8_t statuscode,uint16_t datapos)
{
  Serial.println();
  Serial.print("Status Code: ");
  Serial.println(statuscode, HEX);
  Serial.print("Datapos: ");
  Serial.println(datapos, DEC);
  Serial.println("PAYLOAD");
  for(int i = 0; i < 100; i++){
     Serial.print(byte(buf[i]));
  }
  
  Serial.println();
  Serial.println();  
}

