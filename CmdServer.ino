// -----------------------------------------------------------------------------------
// Cmd channel server

EthernetServer cmdserver1(9999);
EthernetServer cmdserver2(9998);

// t is the timeout in ms, 0 for never
void CmdServer::init(int port, long t) {
  if ((port>=9998) && (port<=9999)) {
    thisPort=port;
    if (thisPort==9999) cmdserver1.begin();
    if (thisPort==9998) cmdserver2.begin();
    timeout=t;
#if DEBUG_CMDSERVER == ON
    Serial.print("cmd server is at ");
    Serial.print(Ethernet.localIP()); Serial.print(":"); Serial.println(thisPort);
#endif
  }
}

void CmdServer::handleClient() {
  if (thisPort==0) return;
  if (!haveClient) {
    // new client connect
    if (thisPort==9999) client=cmdserver1.available();
    if (thisPort==9998) client=cmdserver2.available();
    if (client) {
      lastAccess=millis();
      haveClient = true;
#if DEBUG_CMDSERVER == ON
      Serial.println("client connected");
#endif
    }
  } else {
    // client disconnect
    if (!client.connected() || ((timeout!=0) && (millis()-lastAccess>timeout))) {
      client.stop();
      client = EthernetClient();
      haveClient = false;
#if DEBUG_CMDSERVER == ON
      Serial.print("client disconnected");
      if (millis()-lastAccess>timeout) Serial.println(" (timed out)"); else Serial.println("");
#endif
    }
  }
}

int CmdServer::available() {
  if (thisPort==0) return 0;
  if (!haveClient) return 0;
  if (!client.connected()) return 0;
  return client.available();
}

char CmdServer::read() {
  if (thisPort==0) return 0;
  lastAccess=millis();
  return client.read();
}

void CmdServer::print(char s[]) {
  if (thisPort==0) return;
  if (!client.connected()) return;
  lastAccess=millis();
  client.write(s);
}
