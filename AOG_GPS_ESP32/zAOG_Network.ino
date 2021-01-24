#if HardwarePlatform == 0
//---------------------------------------------------------------------
// start WiFi in Workstation mode = log to existing WiFi

// WIFI handling 15. Jan 2021 for ESP32  -------------------------------------------

void WiFi_Start_STA() {
    unsigned long timeout, timeout2;
    now = millis();
    timeout = now + (Set.timeoutRouter * 1000);
    timeout2 = timeout - (Set.timeoutRouter * 500);

    while (millis() < timeout) {
        //scanning for known networks in reach, MUST be done first, to get network #
        scan_WiFi_connections();
        if (WiFi_netw_nr > 0) { break; }
        delay(1000);
    }    
    
    delay(10);
    WiFi.mode(WIFI_STA);   //  Workstation
    delay(50);
    switch (WiFi_netw_nr) {
    case 1: WiFi.begin(Set.ssid1, Set.password1); break;
    case 2: WiFi.begin(Set.ssid2, Set.password2); break;
    case 3: WiFi.begin(Set.ssid3, Set.password3); break;
    case 4: WiFi.begin(Set.ssid4, Set.password4); break;
    case 5: WiFi.begin(Set.ssid5, Set.password5); break;
    }
    WiFi.config(0U, 0U, 0U);  //set IP to DHCP call immediately after begin!
    delay(300);
    timeout = millis() + (Set.timeoutRouter * 1000);
    timeout2 = timeout - (Set.timeoutRouter * 500);
    while (WiFi.status() != WL_CONNECTED && millis() < timeout) {
        delay(300);
        Serial.print(".");
        //give a 2. try, sometimes WiFi router doesn't let in at first time
        if ((millis() > timeout2) && (WiFi.status() != WL_CONNECTED)) {
            WiFi.disconnect();
            Serial.print("-");
            delay(200);
            switch (WiFi_netw_nr) {
            case 1: WiFi.begin(Set.ssid1, Set.password1); break;
            case 2: WiFi.begin(Set.ssid2, Set.password2); break;
            case 3: WiFi.begin(Set.ssid3, Set.password3); break;
            case 4: WiFi.begin(Set.ssid4, Set.password4); break;
            case 5: WiFi.begin(Set.ssid5, Set.password5); break;
            }
            timeout2 = timeout + 100;
        }
        //WIFI LED blink in double time while connecting
        if (!LED_WIFI_ON) {
            if (millis() > (LED_WIFI_time + (LED_WIFI_pause >> 2)))
            {
                LED_WIFI_time = millis();
                LED_WIFI_ON = true;
                digitalWrite(Set.LEDWiFi_PIN, !Set.LEDWiFi_ON_Level);
            }
        }
        if (LED_WIFI_ON) {
            if (millis() > (LED_WIFI_time + (LED_WIFI_pulse >> 2))) {
                LED_WIFI_time = millis();
                LED_WIFI_ON = false;
                digitalWrite(Set.LEDWiFi_PIN, Set.LEDWiFi_ON_Level);
            }
        }
    }  //connected or timeout  

    Serial.println();  
    if (WiFi.status() == WL_CONNECTED)
    {
        delay(200);
        Serial.println();
        Serial.println("WiFi Client successfully connected");
        Serial.print("Connected IP - Address : ");
        IPAddress myIP = WiFi.localIP();
        Serial.println(myIP);
        //after connecting get IP from router -> change it to x.x.x.IP Ending (from settings)
        myIP[3] = Set.WiFi_myip[3]; //set ESP32 IP to x.x.x.myIP_ending
        Serial.print("changing IP to: ");
        Serial.println(myIP);        
        IPAddress gwIP = WiFi.gatewayIP();
        if (!WiFi.config(myIP, gwIP, Set.mask, gwIP)) { Serial.println("Network failed to configure"); }
        delay(200);
        Serial.print("Connected IP - Address : ");
        myIP = WiFi.localIP();
        WiFi_ipDestination = myIP;
        WiFi_ipDestination[3] = 255;
        Serial.println(myIP);
        Serial.print("Gateway IP - Address : ");
        Serial.println(gwIP);
        Set.WiFi_ipDestination[0] = myIP[0];
        Set.WiFi_ipDestination[1] = myIP[1];
        Set.WiFi_ipDestination[2] = myIP[2];
        Set.WiFi_ipDestination[3] = 255;//set IP to x.x.x.255 according to actual network
        LED_WIFI_ON = true;
        digitalWrite(Set.LEDWiFi_PIN, Set.LEDWiFi_ON_Level);
        my_WiFi_Mode = 1;// WIFI_STA;
    }
    else
    {
        Serial.println("WLAN-Client-Connection failed");
        Serial.println();
        LED_WIFI_ON = false;
        digitalWrite(Set.LEDWiFi_PIN, !Set.LEDWiFi_ON_Level);
    }
    delay(20);
}


//---------------------------------------------------------------------
// scanning for known WiFi networks. Logic by Franz Husch

void scan_WiFi_connections()
{
    Serial.println("scanning for WiFi networks");
    // WiFi.scanNetworks will return the number of networks found
    int WiFi_num_netw_inReach = WiFi.scanNetworks();
    Serial.print("scan done: ");
    if (WiFi_num_netw_inReach == 0) {
        Serial.println("no networks found");
        WiFi_netw_nr = 0;
    }
    else
    {
        Serial.print(WiFi_num_netw_inReach);
        Serial.println(" network(s) found");
        for (int i = 0; i < WiFi_num_netw_inReach; ++i) {
            Serial.println("#" + String(i + 1) + " network : " + WiFi.SSID(i));
        }
        delay(2000);
        for (int i = 0; i < WiFi_num_netw_inReach; ++i) {
            if (WiFi.SSID(i) == Set.ssid1) {
                // network found in list
                Serial.println("Connecting to: " + WiFi.SSID(i));
                WiFi_netw_nr = 1;
                break;
            }
            if (WiFi.SSID(i) == Set.ssid2) {
                // network found in list
                Serial.println("Connecting to: " + WiFi.SSID(i));
                WiFi_netw_nr = 2;
                break;
            }
            if (WiFi.SSID(i) == Set.ssid3) {
                // network found in list
                Serial.println("Connecting to: " + WiFi.SSID(i));
                WiFi_netw_nr = 3;
                break;
            }
            if (WiFi.SSID(i) == Set.ssid4) {
                // network found in list
                Serial.println("Connecting to: " + WiFi.SSID(i));
                WiFi_netw_nr = 4;
                break;
            }            
            if (WiFi.SSID(i) == Set.ssid5) {
                // network found in list
                Serial.println("Connecting to: " + WiFi.SSID(i));
                WiFi_netw_nr = 5;
                break;
            }
        }
    }
}  //end scan_WiFi_connections()


//---------------------------------------------------------------------
// start WiFi Access Point = only if no existing WiFi

void WiFi_Start_AP() {
  WiFi.mode(WIFI_AP);   // Accesspoint
  WiFi.softAP(Set.ssid_ap, "");
  while (!SYSTEM_EVENT_AP_START) // wait until AP has started
   {
    delay(100);
    Serial.print(".");
   }   
  delay(100);//right IP adress only with this delay 
  WiFi.softAPConfig(Set.WiFi_gwip, Set.WiFi_gwip, Set.mask);  // set fix IP for AP  
  delay(300);
  IPAddress myIP = WiFi.softAPIP();
  delay(300);

  //AP_time = millis();
  Serial.print("Accesspoint started - Name : ");
  Serial.println(Set.ssid_ap);
  Serial.print( " IP address: ");
  WiFi_ipDestination = myIP;
  Serial.println(WiFi_ipDestination);
  WiFi_ipDestination[3] = 255;
  LED_WIFI_ON = true;
  digitalWrite(Set.LEDWiFi_PIN, Set.LEDWiFi_ON_Level);
  my_WiFi_Mode = WIFI_AP;
}

//-------------------------------------------------------------------------------------------------

void Eth_Start() {
    Ethernet.init(Set.Eth_CS_PIN);
    delay(50);
    Ethernet.begin(mac);
    delay(200);
    if (Ethernet.hardwareStatus() == EthernetNoHardware) {
        Serial.println("no Ethernet hardware");
    }
    else {
        Serial.println("Ethernet hardware found, checking for connection");
        if (Ethernet.linkStatus() == LinkOFF) {
            Serial.println("Ethernet cable is not connected.");
        }
        else {
            Serial.println("Ethernet status OK");
            Serial.print("Got IP ");
            Serial.println(Ethernet.localIP());
            for (byte n = 0; n < 3; n++) {
                Set.Eth_myip[n] = Ethernet.localIP()[n];
                Eth_ipDestination[n] = Ethernet.localIP()[n];
            }
            Eth_ipDestination[3] = 255;
            Ethernet.setLocalIP(Set.Eth_myip);
            delay(100);
            Serial.print("changed IP to ");
            Serial.println(Ethernet.localIP());
            Ethernet_running = true;
            //init UPD Port sending to AOG //don't init 2 different Eth UDP ports, won't work!
            if (Eth_udpRoof.begin(Set.portMy))
            {
                Serial.print("Ethernet UDP sending from port: ");
                Serial.println(Set.portMy);
            }
            delay(50);
        }
    }
    Serial.println();
}


//-------------------------------------------------------------------------------------------------

void doEthUDPNtrip() {
    Eth_udpRoof.stop();
    Eth_udpNtrip.begin(Set.portAOG);
    packetLenght = Eth_udpNtrip.parsePacket();
    if (packetLenght)
		{
			if (Set.debugmode) { Serial.println("got NTRIP data via Ethernet"); }
            Eth_udpNtrip.read(packetBuffer, packetLenght);
			for (unsigned int i = 0; i < packetLenght; i++)
			{
				Serial1.print(packetBuffer[i]);
			}
			NtripDataTime = millis();
		}  // end of Packet
    Eth_udpNtrip.stop();
    Eth_udpRoof.begin(Set.portMy);
	if ((NtripDataTime + 30000) < millis())
	{
		NtripDataTime = millis();
		Serial.println("no NTRIP from AOG for more than 30s via Ethernet");
	}
}

#endif

//-------------------------------------------------------------------------------------------------
/* from autosteer code, not for async udp
void UDP_Start()
{
    if (UDPToAOG.begin(steerSet.portMy))
    {
        Serial.print("UDP sendig to IP: ");
        for (byte n = 0; n < 4; n++) {
            Serial.print(steerSet.WiFi_ipDestination[n]);
            Serial.print(".");
        }
        Serial.print(" from port: ");
        Serial.print(steerSet.portMy);
        Serial.print(" to port: ");
        Serial.println(steerSet.portDestination);
    }
    delay(300);
    if (UDPFromAOG.begin(steerSet.portAOG))
    {
        Serial.print("UDP listening for AOG data on IP: ");
        Serial.println(WiFi.localIP());
        Serial.print(" on port: ");
        Serial.println(steerSet.portAOG);
        getDataFromAOG();
    }
}*/