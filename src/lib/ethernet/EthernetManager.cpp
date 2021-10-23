// ethernet manager, used by the webserver and ethernet serial IP
#include "EthernetManager.h"

#if defined(OPERATIONAL_MODE) && (OPERATIONAL_MODE == ETHERNET_W5100 || OPERATIONAL_MODE == ETHERNET_W5500)

void EthernetManager::init() {
  if (!active) {
    #ifdef NV_ETHERNET_SETTINGS_BASE
      if (EthernetSettingsSize < sizeof(EthernetSettings)) { nv.readOnly(true); DL("ERR: EthernetManager::init(); EthernetSettingsSize error NV subsystem writes disabled"); }

      if (!nv.isKeyValid()) {
        VLF("MSG: EthernetManager, writing defaults to NV");
        nv.writeBytes(NV_ETHERNET_SETTINGS_BASE, &settings, sizeof(EthernetSettings));
      }

      nv.readBytes(NV_ETHERNET_SETTINGS_BASE, &settings, sizeof(EthernetSettings));
    #endif

    VF("MSG: Ethernet DHCP En = "); VL(settings.dhcp_enabled);
    VF("MSG: Ethernet IP = "); V(settings.ip[0]); V("."); V(settings.ip[1]); V("."); V(settings.ip[2]); V("."); VL(settings.ip[3]);
    VF("MSG: Ethernet GW = "); V(settings.gw[0]); V("."); V(settings.gw[1]); V("."); V(settings.gw[2]); V("."); VL(settings.gw[3]);
    VF("MSG: Ethernet SN = "); V(settings.sn[0]); V("."); V(settings.sn[1]); V("."); V(settings.sn[2]); V("."); VL(settings.sn[3]);

    #if defined(ETHERNET_CS_PIN) && ETHERNET_CS_PIN != OFF
      VF("MSG: Ethernet device ETHERNET_CS_PIN ("); V(ETHERNET_CS_PIN); VL(")");
      Ethernet.init(ETHERNET_CS_PIN);
    #endif

    if (ETHERNET_RESET_PIN != OFF) {
      VF("MSG: Ethernet device ETHERNET_RESET_PIN ("); V(ETHERNET_RESET_PIN); VL(")");
      pinMode(ETHERNET_RESET_PIN, OUTPUT); 
      digitalWrite(ETHERNET_RESET_PIN, LOW);
      delay(1000);
      digitalWrite(ETHERNET_RESET_PIN, HIGH);
      delay(1000);
    }

    Ethernet.begin(settings.mac, settings.ip, settings.dns, settings.gw, settings.sn);

    VLF("MSG: Ethernet initialized");
    active = true;
  }
}

void EthernetManager::restart() {
  Ethernet.begin(settings.mac, settings.ip, settings.dns, settings.gw, settings.sn);
}

void EthernetManager::writeSettings() {
  #ifdef NV_ETHERNET_SETTINGS_BASE
    VLF("MSG: Mount, writing settings to NV");
    nv.writeBytes(NV_ETHERNET_SETTINGS_BASE, &settings, sizeof(EthernetSettings));
  #endif
}

EthernetManager ethernetManager;

#endif
