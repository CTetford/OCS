// -----------------------------------------------------------------------------------------------------------------
// NTP time code
// from here: https://github.com/PaulStoffregen/Time

#include "Tls_NTP.h"

#if TIME_LOCATION_SOURCE == NTP

#include "../../tasks/OnTask.h"
#include <TimeLib.h> // https://github.com/PaulStoffregen/Time/archive/master.zip

#include <Ethernet.h>
EthernetUDP Udp;

IPAddress timeServer = IPAddress TIME_IP_ADDR;

// local port to listen for UDP packets
unsigned int localPort = 8888;

void ntpPoll() {
  if (!tls.isReady()) tls.poll();
}

// initialize
bool TimeLocationSource::init() {
  VF("MSG: TLS, start NTP monitor task (rate 60s priority 7)... ");
  if (tasks.add(60000, 0, true, 7, ntpPoll, "ntpPoll")) {
    VL("success");
    active = true;
  } else {
    VL("FAILED!");
    active = false;
  }

  // flag that start time is unknown
  startTime = 0;

  ready = false;

  return active;
}

void TimeLocationSource::set(JulianDate ut1) {
  ut1 = ut1;
}

void TimeLocationSource::get(JulianDate &ut1) {
  if (!ready) return;
  if (year() >= 0 && year() <= 3000 && month() >= 1 && month() <= 12 && day() >= 1 && day() <= 31 &&
      hour() <= 23 && minute() <= 59 && second() <= 59) {
    GregorianDate greg; greg.year = year(); greg.month = month(); greg.day = day();
    ut1 = calendars.gregorianToJulianDay(greg);
    ut1.hour = hour() + minute()/60.0 + second()/3600.0;
  }
}

void TimeLocationSource::poll() {
  // discard any previously received packets
  unsigned long tOut = millis() + 3000L;
  while ((Udp.parsePacket() > 0) && ((long)(millis() - tOut) < 0)) Y;

  VLF("Transmit NTP Request");
  sendNTPpacket(timeServer);

  uint32_t beginWait = millis();
  while (millis() - beginWait < 1500) {
    int size = Udp.parsePacket();
    if (size >= NTP_PACKET_SIZE) {
      VLF("Receive NTP Response");
      // read packet into the buffer
      Udp.read(packetBuffer, NTP_PACKET_SIZE);
      unsigned long secsSince1900;
      // convert four bytes starting at location 40 to a long integer
      secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
      secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
      secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
      secsSince1900 |= (unsigned long)packetBuffer[43];
      time_t ntpTime = secsSince1900 - 2208988800UL;
      setTime(ntpTime);
      ready = true;
      return;
    }
    Y;
  }
  DLF("No NTP Response :-(");
}

// send an NTP request to the time server at the given address
void TimeLocationSource::sendNTPpacket(IPAddress &address) {
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;
  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:                 
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}

TimeLocationSource tls;
#endif
