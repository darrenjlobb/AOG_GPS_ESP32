#if HardwarePlatform == 0
// WIFI handling 31. Jan 2021 for ESP32  -------------------------------------------

void WiFi_handle_connection() {
    if (WiFi_connect_step > 0) {
        IPAddress gwIP, myIP;
        now = millis();

        if (now > (WiFi_connect_timer + 500)) {
            //do every half second
            if (Set.debugmode) { Serial.print("WiFi_connect_step: "); Serial.println(WiFi_connect_step); }
            switch (WiFi_connect_step) {
            case 1:
                if (Ping.ping(Set.WiFi_gwip)) { //retry to connect NTRIP, WiFi is available
                    WiFi_connect_timer = 0;
                    Ntrip_restart = 1;
                    WiFi_connect_step = 0;
                    if ((Set.NtripClientBy == 2) && (!task_NTRIP_running)) {
                        {
                            xTaskCreatePinnedToCore(NTRIPCode, "Core1", 3072, NULL, 1, &Core1, 1);
                            delay(500);
                        }
                    }
                }
                else { WiFi_connect_timer = now; WiFi_connect_step++; }//no network
                break;
            case 2:
                if (!task_NTRIP_running) { WiFi_connect_step++; }
                WiFi_connect_timer = now;
                break;

            case 3:
                WiFi_netw_nr = 0;
                WebIORunning = false;
                WiFi_UDP_running = false;
                WiFi_Ntrip_cl.stop();
                WiFi_connect_step++;
                WiFi_connect_timer = now;
                break;
            case 4:
                WiFi.mode(WIFI_OFF);
                WiFi_connect_step = 10;
                WiFi_connect_timer = now;
                break;

            case 10:
                WiFi_netw_nr = 0;
                WiFi_network_search_timeout = 0;
                WiFi_connect_tries = 0;
                WebIORunning = false;
                WiFi_UDP_running = false;
                WiFi.mode(WIFI_STA);   //  Workstation
                WiFi_connect_step++;
                WiFi_connect_timer = now;
                break;
            case 11:
                if (WiFi_network_search_timeout == 0) {   //first run                 
                    WiFi_network_search_timeout = now + (Set.timeoutRouter * 1000);
                }
                WiFi_scan_networks();
                //timeout?
                if (now > WiFi_network_search_timeout) { WiFi_connect_step = 50; }
                else {
                    if (WiFi_netw_nr > 0) {
                        //found network
                        WiFi_connect_step++;
                        WiFi_network_search_timeout = 0;//reset timer
                    }
                }
                WiFi_connect_timer = now;
                break;

            case 12:
                WiFi_STA_connect_network();
                if (WiFi_network_search_timeout == 0) {   //first run                   
                    WiFi_network_search_timeout = now + (Set.timeoutRouter * 500);//half time
                }
                if (WiFi.status() != WL_CONNECTED) {
                    Serial.print(".");
                    if (now > WiFi_network_search_timeout) {
                        //timeout
                        WiFi_connect_step = 19;//close WiFi and try again
                        WiFi_connect_tries++;
                        WiFi_network_search_timeout += (Set.timeoutRouter * 500);//add rest of time
                    }
                }
                else {
                    //connected
                    WiFi_connect_step++;
                    WiFi_network_search_timeout = 0;//reset timer
                }
                WiFi_connect_timer = now;
                break;

            case 13:
                WiFi_connect_step++; //need to wait to get correct IP
                WiFi_connect_timer = now;
                break;

            case 14:
                //connected
                Serial.println();
                Serial.println("WiFi Client successfully connected");
                Serial.print("Connected IP - Address : ");
                myIP = WiFi.localIP();
                Serial.println(myIP);
                //after connecting get IP from router -> change it to x.x.x.IP Ending (from settings)
                myIP[3] = Set.WiFi_myip[3]; //set ESP32 IP to x.x.x.myIP_ending
                Serial.print("changing IP to: ");
                Serial.println(myIP);
                gwIP = WiFi.gatewayIP();
                if (!WiFi.config(myIP, gwIP, Set.mask, gwIP)) { Serial.println("STA Failed to configure"); }
                WiFi_connect_step++;
                WiFi_connect_timer = now;
                break;

            case 15:
                Serial.print("Connected IP - Address : ");
                myIP = WiFi.localIP();
                WiFi_ipDestination = myIP;
                WiFi_ipDestination[3] = 255;
                Serial.println(myIP);
                gwIP = WiFi.gatewayIP();
                Serial.print("Gateway IP - Address : ");
                Serial.println(gwIP);
                Set.WiFi_ipDestination[0] = myIP[0];
                Set.WiFi_ipDestination[1] = myIP[1];
                Set.WiFi_ipDestination[2] = myIP[2];
                Set.WiFi_ipDestination[3] = 255;//set IP to x.x.x.255 according to actual network
                my_WiFi_Mode = 1;// WIFI_STA;
                WiFi_connect_step = 20;
                WiFi_connect_timer = now;
                break;

            case 19://no connection at first try, try again
                WiFi.disconnect();
                if (WiFi_connect_tries > 1) {//start access point
                    WiFi_connect_step = 50;
                    Serial.println();
                    Serial.println("error connecting to WiFi network");
                }
                else { WiFi_connect_step = 12; Serial.print("-"); }
                WiFi_connect_timer = now;
                break;

            case 20://init WiFi UDP sending to AOG
                if (WiFi_udpRoof.listen(Set.PortGPSToAOG))
                {
                    Serial.print("UDP writing to IP: ");
                    Serial.println(WiFi_ipDestination);
                    Serial.print("UDP writing to port: ");
                    Serial.println(Set.PortDestination);
                    Serial.print("UDP writing from port: ");
                    Serial.println(Set.PortGPSToAOG);
                }
                if (Set.NtripClientBy == 0) { WiFi_connect_step = 100; WiFi_UDP_running = true; }
                else {
                    if (Set.NtripClientBy == 2) { WiFi_connect_step = 30; }
                    else {
                        if ((Set.DataTransVia > 5) && (Set.DataTransVia < 10)) { WiFi_connect_step++; }
                        else { WiFi_connect_step = 100; }
                    }
                }
                WiFi_connect_timer = now;
                break;

            case 21:
                //init WiFi UPD listening to AOG 
                if (WiFi_udpNtrip.listen(Set.AOGNtripPort))
                {
                    Serial.print("NTRIP WiFi UDP Listening to port: ");
                    Serial.println(Set.AOGNtripPort);
                    Serial.println();
                }
                delay(2);
                WiFi_connect_step++;
                WiFi_connect_timer = now;
                break;

            case 22:
                // UDP NTRIP packet handling
                WiFi_udpNtrip.onPacket([](AsyncUDPPacket packet)
                    {
                        if (Set.debugmode) { Serial.print("got NTRIP data via WiFi. packet lenght: "); Serial.println(packet.length()); }
                        for (unsigned int i = 0; i < packet.length(); i++)
                        {
                            Serial1.write(packet.data()[i]);
                            //  Serial1.print(packet.data()[i]);
                        }
                        NtripDataTime = millis();
                    });  // end of onPacket call
                WiFi_UDP_running = true;
                WiFi_connect_step = 100;
                WiFi_connect_timer = now;
                break;

            case 30://ESP32 NTRIP client
                //create a task that will be executed in the NTRIPcode() function, with priority 1 and executed on core 1
                xTaskCreatePinnedToCore(NTRIPCode, "Core1", 3072, NULL, 1, &Core1, 1);
                delay(500);
                NtripDataTime = millis();
                WiFi_connect_step = 100;
                WiFi_connect_timer = now;
                break;

            case 50://start access point
                WiFi_Start_AP();
                WiFi_connect_step = 100;
                WiFi_connect_timer = now;
                break;

            case 100:
                //start Server for Webinterface
                WiFi_StartServer();
                WiFi_connect_step++;
                WiFi_connect_timer = now;
                break;

            case 101:
                WebIORunning = true;
                WebIOTimeOut = millis() + (long(Set.timeoutWebIO) * 60000);
                WiFi_connect_step = 0;
                WiFi_connect_timer = 0;
                LED_WIFI_ON = true;
#if useLED_BUILTIN
                digitalWrite(LED_BUILTIN, HIGH);
#endif
                digitalWrite(Set.LEDWiFi_PIN, Set.LEDWiFi_ON_Level);
                break;

            default:
                WiFi_connect_step++;
                break;
            }
        }
    }
}





//---------------------------------------------------------------------
// scanning for known WiFi networks

void WiFi_scan_networks()
{
    Serial.println("scanning for WiFi networks");
    // WiFi.scanNetworks will return the number of networks found
    int WiFi_num_netw_inReach = WiFi.scanNetworks();
    Serial.print("scan done: ");
    if (WiFi_num_netw_inReach == 0) {
        Serial.println("no networks found");
    }
    else
    {
        Serial.print(WiFi_num_netw_inReach);
        Serial.println(" network(s) found");
        for (int i = 0; i < WiFi_num_netw_inReach; ++i) {
            Serial.println("#" + String(i + 1) + " network : " + WiFi.SSID(i));
        }
        delay(800);//.SSID gives no value if no delay
        delay(500);

        for (int i = 0; i < WiFi_num_netw_inReach; ++i) {
            if (WiFi.SSID(i) == Set.ssid1) {
                // network found in list
                Serial.println("Connecting to: " + WiFi.SSID(i));
                WiFi_netw_nr = 1;
                break;
            }
        }
        if ((WiFi_netw_nr == 0) && (Set.ssid2 != "")) {
            for (int i = 0; i < WiFi_num_netw_inReach; ++i) {
                if (WiFi.SSID(i) == Set.ssid2) {
                    // network found in list
                    Serial.println("Connecting to: " + WiFi.SSID(i));
                    WiFi_netw_nr = 2;
                    break;
                }
            }
        }
        if ((WiFi_netw_nr == 0) && (Set.ssid3 != "")) {
            for (int i = 0; i < WiFi_num_netw_inReach; ++i) {
                if (WiFi.SSID(i) == Set.ssid3) {
                    // network found in list
                    Serial.println("Connecting to: " + WiFi.SSID(i));
                    WiFi_netw_nr = 3;
                    break;
                }
            }
        }
        if ((WiFi_netw_nr == 0) && (Set.ssid4 != "")) {
            for (int i = 0; i < WiFi_num_netw_inReach; ++i) {
                if (WiFi.SSID(i) == Set.ssid4) {
                    // network found in list
                    Serial.println("Connecting to: " + WiFi.SSID(i));
                    WiFi_netw_nr = 4;
                    break;
                }
            }
        }
        if ((WiFi_netw_nr == 0) && (Set.ssid5 != "")) {
            for (int i = 0; i < WiFi_num_netw_inReach; ++i) {
                if (WiFi.SSID(i) == Set.ssid5) {
                    // network found in list
                    Serial.println("Connecting to: " + WiFi.SSID(i));
                    WiFi_netw_nr = 5;
                    break;
                }
            }
        }
    }
}  //end WiFi_scan_networks()

//-------------------------------------------------------------------------------------------------
//connects to WiFi network

void WiFi_STA_connect_network() {//run WiFi_scan_networks first
    switch (WiFi_netw_nr) {
    case 1: WiFi.begin(Set.ssid1, Set.password1); break;
    case 2: WiFi.begin(Set.ssid2, Set.password2); break;
    case 3: WiFi.begin(Set.ssid3, Set.password3); break;
    case 4: WiFi.begin(Set.ssid4, Set.password4); break;
    case 5: WiFi.begin(Set.ssid5, Set.password5); break;
    }
    if (WiFi_connect_timer == 0) { WiFi.config(0U, 0U, 0U); } //set IP to DHCP on first run. call immediately after begin!
}

        //-------------------------------------------------------------------------------------------------
        // start WiFi Access Point = only if no existing WiFi or connection fails


void WiFi_Start_AP() {
    WiFi.mode(WIFI_AP);   // Accesspoint
    WiFi.softAP(Set.ssid_ap, "");
    delay(5);
    while (!SYSTEM_EVENT_AP_START) // wait until AP has started
    {
        delay(100);
        Serial.print(".");
    }
    delay(150);//right IP adress only with this delay 
    WiFi.softAPConfig(Set.WiFi_gwip, Set.WiFi_gwip, Set.mask);  // set fix IP for AP  
    delay(300);
    IPAddress myIP = WiFi.softAPIP();
    //AP_time = millis();
    Serial.print("Accesspoint started - Name : ");
    Serial.println(Set.ssid_ap);
    Serial.print(" IP address: ");
    WiFi_ipDestination = myIP;
    Serial.println(WiFi_ipDestination);
    WiFi_ipDestination[3] = 255;
    my_WiFi_Mode = WIFI_AP;
}
#endif

//=================================================================================================
//Ethernet
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
            //init UPD Port sending to AOG
            if (Eth_udpRoof.begin(Set.PortGPSToAOG))
            {
                Serial.print("Ethernet UDP sending from port: ");
                Serial.println(Set.PortGPSToAOG);
            }
            delay(50);
            //init UPD Port getting NTRIP from AOG
            if (Eth_udpNtrip.begin(Set.AOGNtripPort))
            {
                Serial.print("Ethernet NTRIP UDP listening to port: ");
                Serial.println(Set.AOGNtripPort);
            }
            delay(50);
        }
    }
    Serial.println();
}




//-------------------------------------------------------------------------------------------------

void doEthUDPNtrip() {
    unsigned int packetLenght = Eth_udpNtrip.parsePacket();
    if (packetLenght)
    {
        if (Set.debugmode) { Serial.print("got NTRIP data via Ethernet lenght: "); Serial.println(packetLenght); }
        Eth_udpNtrip.read(packetBuffer, packetLenght);
        Eth_udpNtrip.flush();
        for (unsigned int i = 0; i < packetLenght; i++)
        {
            Serial1.write(packetBuffer[i]);
        }
        //Serial1.println(); //really send data from UART buffer
        NtripDataTime = millis();
    }  // end of Packet
}

/*

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
        }
        if ((WiFi_netw_nr == 0) && (Set.ssid2 != "")) {
            for (int i = 0; i < WiFi_num_netw_inReach; ++i) {
                if (WiFi.SSID(i) == Set.ssid2) {
                    // network found in list
                    Serial.println("Connecting to: " + WiFi.SSID(i));
                    WiFi_netw_nr = 2;
                    break;
                }
            }
        }
        if ((WiFi_netw_nr == 0) && (Set.ssid3 != "")) {
            for (int i = 0; i < WiFi_num_netw_inReach; ++i) {
                if (WiFi.SSID(i) == Set.ssid3) {
                    // network found in list
                    Serial.println("Connecting to: " + WiFi.SSID(i));
                    WiFi_netw_nr = 3;
                    break;
                }
            }
        }
        if ((WiFi_netw_nr == 0) && (Set.ssid4 != "")) {
            for (int i = 0; i < WiFi_num_netw_inReach; ++i) {
                if (WiFi.SSID(i) == Set.ssid4) {
                    // network found in list
                    Serial.println("Connecting to: " + WiFi.SSID(i));
                    WiFi_netw_nr = 4;
                    break;
                }
            }
        }
        if ((WiFi_netw_nr == 0) && (Set.ssid5 != "")) {
            for (int i = 0; i < WiFi_num_netw_inReach; ++i) {
                if (WiFi.SSID(i) == Set.ssid5) {
                    // network found in list
                    Serial.println("Connecting to: " + WiFi.SSID(i));
                    WiFi_netw_nr = 5;
                    break;
                }
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
            //init UPD Port sending to AOG
            if (Eth_udpRoof.begin(Set.PortGPSToAOG))
            {
                Serial.print("Ethernet UDP sending from port: ");
                Serial.println(Set.PortGPSToAOG);
            }
            delay(50);
            //init UPD Port getting NTRIP from AOG
            if (Eth_udpNtrip.begin(Set.AOGNtripPort))
            {
                Serial.print("Ethernet NTRIP UDP listening to port: ");
                Serial.println(Set.AOGNtripPort);
            }
            delay(50);
        }
    }
    Serial.println();
}




//-------------------------------------------------------------------------------------------------

void doEthUDPNtrip() {
    unsigned int packetLenght = Eth_udpNtrip.parsePacket();
    if (packetLenght)
    {
        if (Set.debugmode) { Serial.println("got NTRIP data via Ethernet"); }
        Eth_udpNtrip.read(packetBuffer, packetLenght);
        Eth_udpNtrip.flush();
        for (unsigned int i = 0; i < packetLenght; i++)
        {
            Serial1.write(packetBuffer[i]);
        }
        Serial1.println(); //really send data from UART buffer
        NtripDataTime = millis();        
    }  // end of Packet
}

#endif

//-------------------------------------------------------------------------------------------------
/* 
//-------------------------------------------------------------------------------------------------

void doEthUDPTest() {
//    Eth_udpRoof.stop();
//    delay(1);
//    Eth_udpNtrip.begin(Set.PortFromAOG);

    unsigned int packetLenght = Eth_udpNtrip.parsePacket();
    if (packetLenght)
    {
        Serial.print(millis());
        Serial.println("got UDP data via Ethernet");
        Eth_udpNtrip.read(packetBuffer, packetLenght);
        for (unsigned int i = 0; i < packetLenght; i++)
        {
            //Serial1.print(packetBuffer[i]);
            Serial.print(packetBuffer[i]);
        }
        NtripDataTime = millis();
        // end of Packet
        Serial.println();
    }
//    Eth_udpNtrip.stop();
//    delay(1);
 //   Eth_udpRoof.begin(Set.PortGPSToAOG);
}


 from autosteer code, not for async udp
 void UDP_Start()
{
    if (UDPToAOG.begin(steerSet.PortGPSToAOG))
    {
        Serial.print("UDP sendig to IP: ");
        for (byte n = 0; n < 4; n++) {
            Serial.print(steerSet.WiFi_ipDestination[n]);
            Serial.print(".");
        }
        Serial.print(" from port: ");
        Serial.print(steerSet.PortGPSToAOG);
        Serial.print(" to port: ");
        Serial.println(steerSet.PortDestination);
    }
    delay(300);
    if (UDPFromAOG.begin(steerSet.PortFromAOG))
    {
        Serial.print("UDP listening for AOG data on IP: ");
        Serial.println(WiFi.localIP());
        Serial.print(" on port: ");
        Serial.println(steerSet.PortFromAOG);
        getDataFromAOG();
    }
}*/
