#include <espDMX.h>
#include <Espanol.h>
#include <ESP8266mDNS.h>
#include <ArduinoOTA.h>

#include "networkconf.h"

#ifndef MQTT_NET_ESSID
# error Please define MQTT_NET_ESSID in networkconf.h
#endif
#ifndef MQTT_NET_PASSWORD
# error Please define MQTT_NET_PASSWORD in networkconf.h
#endif
#ifndef MQTT_SERVER
# error Please define MQTT_SERVER in networkconf.h
#endif

#ifndef OTA_PASSWORD
# error Please define OTA_PASSWORD in networkconf.h
#endif

#define MAX(x,y) ((x) > (y) ? (x) : (y))

#define HOSTNAME     "spotpole"
#define MQTT_PREFIX  "hasi/lights/spot/"

#define dmxX dmxA

char* ssid       = MQTT_NET_ESSID;
char* password   = MQTT_NET_PASSWORD;
char* broker     = MQTT_SERVER;
char* clientname = HOSTNAME;
int   port       = 1883;


uint8_t
parse_hexdigit (const char digit)
{
  if (digit >= '0' && digit <= '9')
    return digit - '0';
  else if (digit >= 'a' && digit <= 'f')
    return digit - 'a' + 10;
  else if (digit >= 'A' && digit <= 'F')
    return digit - 'A' + 10;
  else
    return 0xff;
}


bool
parse_color (const char   *parsedata,
             unsigned int  len,
             byte         *target)
{
  int i;
    
  if (len != 4 && len != 7)
    return false;

  if (parsedata[0] != '#')
    return false;

  for (i = 1; i < len; i++)
    if (parse_hexdigit (parsedata[i]) >= 16)
      return false;

  if (len == 4)
    {
      target[0] = parse_hexdigit (parsedata[1]) * 17;
      target[1] = parse_hexdigit (parsedata[2]) * 17;
      target[2] = parse_hexdigit (parsedata[3]) * 17;
    }
  else
    {
      target[0] = parse_hexdigit (parsedata[1]) * 16 +
                  parse_hexdigit (parsedata[2]);
      target[1] = parse_hexdigit (parsedata[3]) * 16 +
                  parse_hexdigit (parsedata[4]);
      target[2] = parse_hexdigit (parsedata[5]) * 16 +
                  parse_hexdigit (parsedata[6]);
    }

  return true;
}


void
parse_dmx_string (const char   *parsedata,
                  unsigned int  len,
                  byte         *target)
{
  int cur_chan = 0;
  int cur_value = 0;
  int pos = 0;
  int base = 10;
  int digitcount = 0;

  pos = 0;
  while (pos <= len)
    {
      char c;
      
      if (pos < len)
        c = parsedata[pos];
      else
        c = '\0';
        
      switch (c)
        {
          case '0':
          case '1':
          case '2':
          case '3':
          case '4':
          case '5':
          case '6':
          case '7':
          case '8':
          case '9':
          case 'A':
          case 'B':
          case 'C':
          case 'D':
          case 'E':
          case 'F':
          case 'a':
          case 'b':
          case 'c':
          case 'd':
          case 'e':
          case 'f':
            if (base == 16 && digitcount == 2)
              {
                if (cur_chan < 0 || cur_chan >= 512)
                  return;
            
                target[cur_chan] = cur_value;
                cur_value = 0;
                cur_chan += 1;
                digitcount = 0; 
              }

            if (c >= 'a')
              c = c - 'a' + 10;
            else if (c >= 'A')
              c = c - 'A' + 10;
            else if (c >= '0')
              c = c - '0' + 0;
              
            if (c >= base)
              return;

            cur_value = cur_value * base + c;
            digitcount += 1;
              
            break;

          case 'x':
          case '#':
            if (cur_value != 0)
              return;

            base = 16;
            digitcount = 0;
            break;

          case ':':
            if (cur_value <= 0 || cur_value > 512)
              return;

            cur_chan = cur_value - 1;
            cur_value = 0;
            digitcount = 0;
            base = 10;
            break;

          case '\0':
          case ',':
            if (cur_value < 0 || cur_value >= 256)
              return;
            if (cur_chan < 0 || cur_chan >= 512)
              return;
            
            target[cur_chan] = cur_value;
            cur_value = 0;
            cur_chan += 1;
            digitcount = 0;
            base = 10;
            
            break;
            
          default:
            return;
            break;
        }

      pos ++;
    }
}


void
handle_request (char *topic, byte *payload, unsigned int len)
{
  char *payload_c = (char *) payload;
  char *subtopic;
  byte rgb[3] = { 0, 0, 0 };
  int ch_offset = 1;

  if (strncmp (MQTT_PREFIX, topic, strlen (MQTT_PREFIX)) != 0)
    return;

  subtopic = topic + strlen (MQTT_PREFIX);

  if (strncmp (subtopic, "channel", 7) == 0 &&
      subtopic[7] >= '0' && subtopic[7] <= '3' &&
      parse_color (payload_c, len, rgb) == true)
    {
      ch_offset = (subtopic[7] - '0') * 3 + 1;
      dmxX.setChans (rgb, 3, ch_offset);
    }
  else if (strcmp (subtopic, "dmx_raw") == 0)
    {
      dmxX.setChans (payload, MAX (len, 512), 1);
    }
  else if (strcmp (subtopic, "dmx") == 0)
    {
      byte *dmxdata;

      dmxdata = dmxX.getChans ();
      
      parse_dmx_string (payload_c, len, dmxdata);
      dmxX.setChans (dmxdata);
    }
}


void
setup ()
{
  byte zero = 0;
  
  // Start dmxX, status LED on pin 12 with full intensity
  dmxX.begin (12);
  dmxX.clearChans ();
  dmxX.setChans (&zero, 1, 12);

  Espanol.begin (ssid, password, clientname, broker, port);

  ArduinoOTA.setHostname (HOSTNAME);
  ArduinoOTA.setPassword (OTA_PASSWORD);
  ArduinoOTA.begin ();

  Espanol.subscribe (MQTT_PREFIX "#");
  Espanol.setCallback (handle_request);
}


void
loop ()
{
  ArduinoOTA.handle ();
  Espanol.loop ();
}
