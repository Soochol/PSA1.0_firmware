#include "type1scMqttLte.h"

static const char TAG[] = __FILE__;

HardwareSerial M1Serial(1); // use ESP32 UART2
TYPE1SC TYPE1SC(M1Serial, DebugSerial, PWR_PIN, RST_PIN, WAKEUP_PIN);

void (*userCallback)(const char* response) = NULL;

void registerModemCallback(void (*callback)(const char*)) {
    userCallback = callback;
}

void modemTask(void* parameter) {
    String buffer = "";
    while (1) {
        if (M1Serial.available()) {
            char c = M1Serial.read();
            buffer += c;

            if (c == '\n') {  // End of AT response line
                if (userCallback != NULL) {
                    userCallback(buffer.c_str());
                }
                // DebugSerial.println("-------------------------");
                // DebugSerial.println(buffer);
                // DebugSerial.println("-------------------------");
                buffer = "";
            }
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);  // Light CPU load
    }
}

void myATResponseHandler(const char* response) {
    Serial.printf("[Modem] Response: %s\n", response);
    if (strstr(response, "+QMTRECV:")) {
        // Handle MQTT message
    }
}


void extAntenna() {
  if (EXT_ANT_ON) {
    pinMode(EXT_ANT, OUTPUT);
    digitalWrite(EXT_ANT, HIGH);
    delay(100);
  }
}

void setSerials() {

    /* Serial2 Initialization */
    M1Serial.begin(115200, SERIAL_8N1, ESP_U1_RXD, ESP_U1_TXD); // RXD2 : 16, TXD2 : 17
    DebugSerial.begin(115200);

    DebugSerial.println("\tTYPE1SC Module Start!!!");
}

void publishTask(void *pvParameters) {
    while (1) {
        // Wait for a signal that a report is ready to be sent
        if (xSemaphoreTake(reportReadySemaphore, portMAX_DELAY) == pdTRUE) {
            DebugSerial.println("---publishTask initiating---");
            ESP_LOGI(TAG, "Publisher task received signal to send report.");
            unsigned long start_report = millis();

            // Perform the blocking publish operation
            if (type1scReport() == true) { // type1scReport would now be called by this task
                ESP_LOGI(TAG, "6.Published report to broker");
            } else {
                ESP_LOGI(TAG, "6.Something went wrong");
            }
            ESP_LOGI(TAG, "type1scReport() completed in %lu ms", millis() - start_report);
            DebugSerial.println("---publishTask finished---");
        }
    }
}

void initType1sc() {

    extAntenna();

    /* Board Reset */
    TYPE1SC.reset();
    delay(2000);

    /* TYPE1SC Module Initialization */
    if (TYPE1SC.init()) {
        DebugSerial.println("\tTYPE1SC Module Error!!!");
    }

    /* Network Registration Check */
    while (TYPE1SC.canConnect() != 0) {
        DebugSerial.println("\tNetwork not Ready!!!");
        delay(2000);
    }

    DebugSerial.println("\tTYPE1SC Module Ready!!!");

    /*** TYPE1SC Basic Test Code ***/
    /* SIM Card Check */
    if (!TYPE1SC.chkSIM()) {
        DebugSerial.println("\tSIM Card OK!!!");
    }
    delay(1000);

    xTaskCreate(publishTask, "publishTask", PUBLISH_TASK_STACK, NULL, PUBLISH_TASK_PRI, &TaskPublish_h);
    DebugSerial.println("\tTYPE1SC initialized...");
    
}

void type1scSetAPN() {
    char *apnAddr = "simplio.apn"; /* Vodafone Global IoT SIM APN */

    if (TYPE1SC.setAPN(apnAddr) == 0) {
        DebugSerial.println("\tTYPE1SC Set APN Address!!!");
    }
    delay(1000);


    char apn[128];
    if (TYPE1SC.getAPN(apn, sizeof(apn)) == 0) {
        DebugSerial.print("\tGET APN Address: ");
        DebugSerial.println(apn);
    }

    DebugSerial.println("\tTYPE1SC APN Setup Complete!!!");
}

void type1scSetCertificates() {

    /* DELETE Certification Profile 1-255 */
    int delProfile = 9;
    if (TYPE1SC.delCert(delProfile) == 0) {
        DebugSerial.println("\tDelete Certification Profile!!!");
    }
    delay(2000);

    /* Write root CA, Don't edit the file name */
    if (TYPE1SC.writeKEY("rootCA.pem", 0, rootCA) == 0) {
        DebugSerial.println("\tRoot CA Write!!!");
    }
    delay(2000);

    /* Write client CA, Don't edit the file name */
    if (TYPE1SC.writeKEY("cert.pem.crt", 0, clientCrt) == 0) {
        DebugSerial.println("\tClient CA Write!!!");
    }
    delay(2000);

    /* Write client KEY, Don't edit the file name */
    if (TYPE1SC.writeKEY("private.pem.key", 1, clientKey) == 0) {
        DebugSerial.println("\tClient KEY Write!!!");
    }
    delay(2000);

    /* ADD Certification Profile 1-255 */
    int addProfile = 9;
    if (TYPE1SC.addCert(addProfile) == 0) {
        DebugSerial.println("\tADD Certification Profile!!!");
    }
    delay(2000);
}

void type1scPrintStrength() {
    /* Get RSSI */
    int rssi;
    if (TYPE1SC.getRSSI(&rssi) == 0) {
        DebugSerial.print("\tRSSI : ");
        DebugSerial.println(rssi);
    }
    delay(100);

    /* Get RSRP */
    int rsrp;
    if (TYPE1SC.getRSRP(&rsrp) == 0) {
        DebugSerial.print("\tRSRP : ");
        DebugSerial.println(rsrp);
    }
    delay(100);

    /* Get RSRQ */
    int rsrq;
    if (TYPE1SC.getRSRQ(&rsrq) == 0) {
        DebugSerial.print("\tRSRQ : ");
        DebugSerial.println(rsrq);
    }
    delay(100);

    /* Get SINR */
    int sinr;
    if (TYPE1SC.getSINR(&sinr) == 0) {
        DebugSerial.print("\tSINR : ");
        DebugSerial.println(sinr);
    }
    delay(100);
}

void type1scBasicTest() {
    /* Get Phone Number */
    char szCIMI[32];
    if (TYPE1SC.getCIMI(szCIMI, sizeof(szCIMI)) == 0) {
        DebugSerial.print("\tIMSI : ");
        DebugSerial.println(szCIMI);
    }
    delay(1000);

    /* Get IMEI Number */
    char szIMEI[32];
    if (TYPE1SC.getIMEI(szIMEI, sizeof(szIMEI)) == 0) {
        DebugSerial.print("\tIMEI : ");
        DebugSerial.println(szIMEI);
    }
    delay(1000);

    /* Get ICCID Number */
    char szICCID[32];
    if (TYPE1SC.getICCID(szICCID, sizeof(szICCID)) == 0) {
        DebugSerial.print("\tICCID : ");
        DebugSerial.println(szICCID);
    }
    delay(1000);

    /* Get Fimrware version */
    char szCGMR[32];
    if (TYPE1SC.getCGMR(szCGMR, sizeof(szCGMR)) == 0) {
        DebugSerial.print("\tCGMR : ");
        DebugSerial.println(szCGMR);
    }
    delay(1000);

    /* Get Time (GMT, (+36/4) ==> Korea +9hour) */
    char szTime[32];
    if (TYPE1SC.getCCLK(szTime, sizeof(szTime)) == 0) {
        DebugSerial.print("\tTime : ");
        DebugSerial.println(szTime);
    }
    delay(1000);

    /* Get RSSI */
    int rssi;
    if (TYPE1SC.getRSSI(&rssi) == 0) {
        DebugSerial.print("\tRSSI : ");
        DebugSerial.println(rssi);
    }
    delay(1000);

    /* Get RSRP */
    int rsrp;
    if (TYPE1SC.getRSRP(&rsrp) == 0) {
        DebugSerial.print("\tRSRP : ");
        DebugSerial.println(rsrp);
    }
    delay(1000);

    /* Get RSRQ */
    int rsrq;
    if (TYPE1SC.getRSRQ(&rsrq) == 0) {
        DebugSerial.print("\tRSRQ : ");
        DebugSerial.println(rsrq);
    }
    delay(1000);

    /* Get SINR */
    int sinr;
    if (TYPE1SC.getSINR(&sinr) == 0) {
        DebugSerial.print("\tSINR : ");
        DebugSerial.println(sinr);
    }
    delay(1000);

    /* Get TX Power */
    char txPower[64];
    if (TYPE1SC.getTxPower(txPower, sizeof(txPower)) == 0) {
        DebugSerial.print("\tTX Power : ");
        DebugSerial.println(txPower);
    }
    delay(1000);
}

void type1scHttpTest() {
    /* Enter a DNS address to get an IP address */
    char IPAddr[32];

    while (1) {

        if (TYPE1SC.getIPAddr("api.thingspeak.com", IPAddr, sizeof(IPAddr)) == 0) {
        DebugSerial.print("IP Address : ");
        DebugSerial.println(IPAddr);
        break;
        } else {
        DebugSerial.println("IP Address Error!!!");
        }
        delay(2000);
    }

    int _PORT = 80;
    char sckInfo[128];
    char recvBuffer[700];
    int recvSize;

    String WApiKey = "61GNSUUGN6RAYDP8"; // Thing Speak Write API Key 16Character
    float temp = 0.0;
    float humi = 0.0;
    String fieldTemp = "field1"; // Air temperature
    String fieldHumi = "field2"; // Air humidity

    /*Get Temperature & Humidity */
    while (1) {
        /* Get DHT22 Sensor */
        temp = 100;
        humi = 56;
        if (String(temp) != "nan" && String(humi) != "nan")
        break;
        else {
        DebugSerial.println("case nan ...");
        delay(1000);
        }
    }

    /* 1 :TCP Socket Create ( 0:UDP, 1:TCP ) */
    if (TYPE1SC.socketCreate(1, IPAddr, _PORT) == 0)
        DebugSerial.println("TCP Socket Create!!!");

    INFO:

    /* 2 :TCP Socket Activation */
    if (TYPE1SC.socketActivate() == 0)
        DebugSerial.println("TCP Socket Activation!!!");

    if (TYPE1SC.socketInfo(sckInfo, sizeof(sckInfo)) == 0) {
        DebugSerial.print("Socket Info : ");
        DebugSerial.println(sckInfo);

        if (strcmp(sckInfo, "ACTIVATED")) {
        delay(3000);
        goto INFO;
        }
    }

    /* 3 :TCP Socket Send Data */
    String data = "GET /update";
    data += "?api_key=" + WApiKey + "&" + fieldTemp + "=" + String(temp) + "&" +
            fieldHumi + "=" + String(humi);
    data += " HTTP/1.1\r\n";
    data += "Host: api.thingspeak.com\r\n";
    data += "Connection: close\r\n\r\n";

    if (TYPE1SC.socketSend(data.c_str()) == 0) {
        DebugSerial.print("[HTTP Send] >>  ");
        DebugSerial.println(data);
    } else
        DebugSerial.println("Send Fail!!!");

    /* 4 :TCP Socket Recv Data */
    if (TYPE1SC.socketRecv(recvBuffer, sizeof(recvBuffer), &recvSize) == 0) {
        DebugSerial.print("[Recv] >>  ");
        DebugSerial.println(recvBuffer);
        DebugSerial.print("[RecvSize] >>  ");
        DebugSerial.println(recvSize);
    } else {
        DebugSerial.println("Recv Fail!!!");
    }

    /* 5 :TCP Socket DeActivation */
    if (TYPE1SC.socketDeActivate() == 0)
        DebugSerial.println("TCP Socket DeActivation!!!");

    if (TYPE1SC.socketInfo(sckInfo, sizeof(sckInfo)) == 0) {
        DebugSerial.print("Socket Info : ");
        DebugSerial.println(sckInfo);
    }

    /* 6 :TCP Socket DeActivation */
    if (TYPE1SC.socketClose() == 0) {
        DebugSerial.println("TCP Socket Close!!!");
    }
}

void type1scConnectAws() {
    int tlsProfile = 9;
    int conn_timeout = 1200;

    /* 1 : Configure AWS_IOT parameters (ID, Address, tlsProfile) */
    if (TYPE1SC.setAWSIOT_CONN(cfg->thing_name, MQTT_HOST, tlsProfile) == 0) {
        DebugSerial.println("\t1.Configure AWS_IOT parameter:ID, Address, tls Profile");
    }
    delay(100);

    /* 2 : Configure AWS_IOT parameters (Connection Timeout) */
    if (TYPE1SC.setAWSIOT_TIMEOUT(conn_timeout) == 0) {
        DebugSerial.println("\t2.Configure AWS_IOT parameter:Timeout");
    }
    delay(100);

    /* 3 : Enable AWS_IOT events */
    if (TYPE1SC.setAWSIOT_EV(1) == 0) {
        DebugSerial.println("\t3.Enable AWS_IOT events");
    }
    delay(100);

    /* 4 : Establish connection */
    if (TYPE1SC.AWSIOT_Connect() == 0) {
        DebugSerial.println("\t4.Establish connection");
    }
    delay(100);

    /* 5 : Subscribe (register) to the topic on the endpoint */
    if (TYPE1SC.AWSIOT_SUBSCRIBE(cfg->subTopic) == 0) {
        DebugSerial.println("\t5.Subscribe to the topic on the endpoint");
    }
    delay(100);
    if (TYPE1SC.AWSIOT_SUBSCRIBE(cfg->pubReportTopic) == 0) {
        DebugSerial.println("\t5.Subscribe to the topic on the endpoint");
    }
    delay(100);
    if (TYPE1SC.AWSIOT_SUBSCRIBE(cfg->pubPowerOnTopic) == 0) {
        DebugSerial.println("\t5.Subscribe to the topic on the endpoint");
    }
    delay(100);
    if (TYPE1SC.AWSIOT_SUBSCRIBE(cfg->pubEmergencyTopic) == 0) {
        DebugSerial.println("\t5.Subscribe to the topic on the endpoint");
    }
    delay(100);

    char _message[64];
    memset(_message, 0x0, sizeof(_message));
    sprintf(_message, "Temperature/%s, Humidity/%s", "100", "56");

    /* 6 : Publish data to broker */
    // if (TYPE1SC.AWSIOT_Publish(cfg->pubPowerOnTopic, _message) == 0) {
    //     DebugSerial.println("\t6.Publish data to broker");
    // } else {
    //     DebugSerial.println("\t6.Something went wrong");
    // }
    
    cfg->isConnected = true;

    if (type1scPowerOn() == true) {
        DebugSerial.println("\t6.Publish data to broker");
    } else {
        DebugSerial.println("\t6.Something went wrong");
    }
}

void type1scRegisterCallback() {
    registerModemCallback(myATResponseHandler);
    xTaskCreatePinnedToCore(&modemTask, "modemTask", MQTT_TASK_STACK, NULL, MQTT_TASK_PRI, &TaskSubscribe_h, MQTT_TASK_CORE);
}

bool type1scPowerOn() {
    PayloadConvert payload;
    payload.reset();
    payload.addDeviceId();
    payload.setTitle(METHOD_POWERON);
    // payload.addDeviceStates();
    if (TYPE1SC.AWSIOT_Publish(cfg->pubPowerOnTopic, payload.getBuffer()) == 0) {
        return true;
    } else {
        return false;
    }
}

bool type1scReport() {
    type1scPrintStrength();
    PayloadConvert payload;
    payload.reset();
    payload.addDeviceId();
    payload.setTitle(METHOD_REPORT);
    payload.addSensorData();
    payload.addDeviceData();
    if (TYPE1SC.AWSIOT_Publish(cfg->pubReportTopic, payload.getBuffer()) == 0) {
        return true;
    } else {
        return false;
    }
}

bool type1scEmergency() {

    PayloadConvert payload;
    payload.reset();
    payload.addDeviceId();
    payload.setTitle(METHOD_EMERGENCY);
    payload.addUserId();
    if (TYPE1SC.AWSIOT_Publish(cfg->pubEmergencyTopic, payload.getBuffer()) == 0) {
        return true;
    } else {
        return false;
    }
}