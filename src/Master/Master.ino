/*
 * ESP32 Gateway (Mother Node) - MESH + SMS System
 * Collects data from nodes and communicates externally via SMS
 */

#include <painlessMesh.h>
#include <EEPROM.h>
#include <Wire.h>
#include <LiquidCrystal.h>
#include "configSIM800L.h"
#include "StatefulGSMLib.h" // using tabahi/StatefulGSMLib

// ============ MESH network settings ============
#define MESH_PREFIX "SensorMesh"
#define MESH_PASSWORD "mesh12345"
#define MESH_PORT 5555

// ============ LCD1602A settings ============
// Pins: RS, EN, D4, D5, D6, D7
#define LCD_RS 19
#define LCD_EN 23
#define LCD_D4 14
#define LCD_D5 27
#define LCD_D6 26
#define LCD_D7 25
LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

// ============ EEPROM settings ============
#define EEPROM_SIZE 512
#define NODE_TABLE_ADDR 0

// ============ Whitelist numbers ============
String allowedNumbers[] = {
    "+989023037750"};
const int allowedCount = 1;

// ============ Alert number ============
String alertNumber = "+989023037750";

// ============ Alert thresholds ============
#define TEMP_HIGH 30.0
#define TEMP_LOW 10.0
#define HUM_HIGH 80.0
#define HUM_LOW 20.0

int displayIndex = 0;

// ============ Node data structure ============
struct NodeData
{
    char nodeId[10];
    char room[15];
    char sensorType[15];
    float value1;
    float value2;
    unsigned long lastUpdate;
    bool online;
    bool alertSent;
};

// ============ Node table ============
#define MAX_NODES 10
NodeData nodeTable[MAX_NODES];
int nodeCount = 0;

// ============ Variables ============
Scheduler userScheduler;
painlessMesh mesh;

// ------------- StatefulGSMLib (SIM800L) setup -------------
HardwareSerial HSerial1(1); // UART1
SIM800L sim800(HSerial1);

bool gsmReady = false;

// ============ SMS type compatibility ============
struct SMS
{
    bool isValid;
    String number;
    String message;
    String datetime;
};

// ============ Function prototypes ============
void checkIncomingSMS();
void updateDisplay();
void checkNodeStatus();
void processSMS(SMS sms);
String processCommand(String cmd);

// ============ Tasks ============
Task taskCheckSMS(5000, TASK_FOREVER, &checkIncomingSMS);
Task taskUpdateDisplay(3000, TASK_FOREVER, &updateDisplay);
Task taskCheckNodeStatus(10000, TASK_FOREVER, &checkNodeStatus);

// ============ SMS queue (safe to send from loop) ============
struct QueuedSMS
{
    String number;
    String message;
};

#define SMS_QUEUE_LEN 8
QueuedSMS smsQueue[SMS_QUEUE_LEN];
int smsQHead = 0;
int smsQTail = 0;

bool saveNeeded = false; // deferred save flag

// Rate control for sending SMS to avoid modem overload
unsigned long lastSMSProcessTime = 0;
const unsigned long SMS_PROCESS_INTERVAL_MS = 1500; // 1.5s between SMS

// ============ Queue functions ============
bool enqueueSMS(const String &num, const String &msg)
{
    int next = (smsQTail + 1) % SMS_QUEUE_LEN;
    if (next == smsQHead)
    {
        Serial.println("Alert/SMS queue full!");
        return false;
    }
    smsQueue[smsQTail].number = num;
    smsQueue[smsQTail].message = msg;
    smsQTail = next;
    return true;
}

bool dequeueSMS(QueuedSMS &out)
{
    if (smsQHead == smsQTail)
        return false; // empty
    out = smsQueue[smsQHead];
    smsQHead = (smsQHead + 1) % SMS_QUEUE_LEN;
    return true;
}

// ============ SIM800L (StatefulGSMLib) functions ============

// Initialize SIM800L
bool initSIM800()
{
    Serial.println("Initializing SIM800L...");

    sim800.begin(MODEM_BAUD_RATE, MODEM_RX_PIN, MODEM_TX_PIN, MODEM_PWRKEY_PIN, MODEM_RST_PIN, MODEM_PWR_EXT_PIN);

    unsigned long start = millis();
    const unsigned long timeout = 30000; // 30 seconds

    while (millis() - start < timeout)
    {
        sim800.loop();

        // check signal strength (function provided by library)
        int signal = sim800.getSignalStrength();
        if (signal > 0)
        {
            Serial.println("SIM800L ready with signal: " + String(signal));
            gsmReady = true;
            return true;
        }

        Serial.print(".");
        delay(500);
    }

    Serial.println("\nError: SIM800L not responding!");
    return false;
}

// Wrapper to send SMS (actual send handled by library)
bool sendSMS(const String &number, const String &message)
{
    if (!gsmReady)
    {
        Serial.println("Error: GSM not ready!");
        return false;
    }

    Serial.println("\nSending SMS to: " + number);
    Serial.println("Message: " + message);

    // library's sendSMS may be void in some versions; call it anyway
    sim800.sendSMS(alertNumber, message);
    Serial.println("SMS queued/sent by modem (non-blocking).");

    return true;
}

// Read incoming SMS and handle it
void checkIncomingSMS()
{
    if (!gsmReady)
        return;

    // run GSM state machine
    sim800.loop();

    // library sets sim800.sms_available and receivedNumber/receivedMessage
    if (sim800.sms_available)
    {
        SMS sms;
        sms.isValid = true;
        // copy data to avoid internal pointers
        sms.number = String(sim800.receivedNumber);
        sms.message = String(sim800.receivedMessage);
        // datetime not provided by example API; use timestamp
        sms.datetime = String(millis());

        Serial.println("\nNew SMS available:");
        Serial.println("  From: " + sms.number);
        Serial.println("  Message: " + sms.message);
        Serial.println("  Date: " + sms.datetime);

        processSMS(sms);

        // clear flag to avoid re-processing
        sim800.sms_available = false;
    }
}

// Process received SMS
void processSMS(SMS sms)
{
    // whitelist check
    bool authorized = true;
    for (int i = 0; i < allowedCount; i++)
    {
        if (sms.number.indexOf(allowedNumbers[i]) >= 0)
        {
            authorized = true;
            break;
        }
    }

    if (!authorized)
    {
        Serial.println("Unauthorized number!");
        return;
    }

    // process command
    String command = sms.message;
    command.trim();
    command.toUpperCase();

    Serial.println("Processing command: " + command);

    String response = processCommand(command);

    // enqueue response to send from loop context
    if (response.length() > 0)
    {
        if (!enqueueSMS(alertNumber, response))
        {
            Serial.println("SMS queue full - cannot enqueue reply.");
        }
    }
}

// Process textual commands and produce reply
String processCommand(String cmd)
{
    cmd.trim();

    if (cmd == "HELP")
    {
        return "Commands:\nGET ROOM1\nGET ROOM2\nGET ALL\nSTATUS\nHELP";
    }

    if (cmd == "GET ALL")
    {
        String response = "All Rooms:\n";
        int count = 0;

        for (int i = 0; i < nodeCount && count < 5; i++)
        {
            if (nodeTable[i].online)
            {
                response += String(nodeTable[i].room) + ": ";
                response += String(nodeTable[i].value1, 1);

                if (String(nodeTable[i].sensorType).indexOf("hum") >= 0)
                {
                    response += "C," + String(nodeTable[i].value2, 1) + "%";
                }
                else
                {
                    response += "C";
                }

                response += "\n";
                count++;
            }
        }

        if (count == 0)
        {
            return "No online nodes!";
        }

        return response;
    }

    if (cmd.startsWith("GET ROOM"))
    {
        String roomName = cmd.substring(4);
        roomName.trim();
        roomName.toLowerCase();

        for (int i = 0; i < nodeCount; i++)
        {
            String nodeRoom = String(nodeTable[i].room);
            nodeRoom.toLowerCase();

            if (nodeRoom == roomName && nodeTable[i].online)
            {
                String response = String(nodeTable[i].room) + ":\n";
                response += "Temp: " + String(nodeTable[i].value1, 1) + "C\n";

                if (String(nodeTable[i].sensorType).indexOf("hum") >= 0)
                {
                    response += "Hum: " + String(nodeTable[i].value2, 1) + "%\n";
                }

                unsigned long secAgo = (millis() - nodeTable[i].lastUpdate) / 1000;
                response += String(secAgo) + "s ago";

                return response;
            }
        }
        return "Room not found!";
    }

    if (cmd == "STATUS")
    {
        String response = "System:\n";
        response += String(nodeCount) + " nodes\n";

        int onlineCount = 0;
        for (int i = 0; i < nodeCount; i++)
        {
            if (nodeTable[i].online)
                onlineCount++;
        }
        response += String(onlineCount) + " online\n";

        int signalQuality = sim800.getSignalStrength();
        response += "Signal: " + String(signalQuality);

        return response;
    }

    return "Invalid cmd!\nSend HELP";
}

// ============ Node management functions ============

// Find node by ID
int findNode(String nodeId)
{
    for (int i = 0; i < nodeCount; i++)
    {
        if (String(nodeTable[i].nodeId) == nodeId)
        {
            return i;
        }
    }
    return -1;
}

// Add or update a node (deferred save)
int addOrUpdateNode(String nodeId, String room, String sensorType)
{
    int idx = findNode(nodeId);

    if (idx < 0 && nodeCount < MAX_NODES)
    {
        idx = nodeCount++;
        Serial.println("\nNew node registered:");
    }
    else if (idx < 0)
    {
        Serial.println("Error: Node table full!");
        return -1;
    }
    else
    {
        Serial.println("\nNode updated:");
    }

    nodeId.toCharArray(nodeTable[idx].nodeId, 10);
    room.toCharArray(nodeTable[idx].room, 15);
    sensorType.toCharArray(nodeTable[idx].sensorType, 15);
    nodeTable[idx].online = true;
    nodeTable[idx].lastUpdate = millis();
    nodeTable[idx].alertSent = false;

    Serial.println("  ID: " + nodeId);
    Serial.println("  Room: " + room);
    Serial.println("  Sensor: " + sensorType);

    // deferred save (do not call EEPROM.commit here)
    saveNeeded = true;
    return idx;
}

// Update node sensor data
void updateNodeData(int idx, float val1, float val2)
{
    if (idx < 0 || idx >= nodeCount)
        return;

    nodeTable[idx].value1 = val1;
    nodeTable[idx].value2 = val2;
    nodeTable[idx].lastUpdate = millis();
    nodeTable[idx].online = true;

    Serial.println("\nData updated:");
    Serial.println("  " + String(nodeTable[idx].room));
    Serial.println("  Value1: " + String(val1, 1));
    if (val2 != -999)
    {
        Serial.println("  Value2: " + String(val2, 1));
    }

    // check thresholds and queue alerts
    checkAlert(idx);

    // deferred save
    saveNeeded = true;
}

// Check thresholds and queue alert if needed
void checkAlert(int idx)
{
    if (nodeTable[idx].alertSent)
        return;

    bool alert = false;
    String alertMsg = "ALERT!\n" + String(nodeTable[idx].room) + ":\n";

    float temp = nodeTable[idx].value1;
    float hum = nodeTable[idx].value2;

    if (temp > TEMP_HIGH)
    {
        alert = true;
        alertMsg += "High temp: " + String(temp, 1) + "C\n";
    }
    else if (temp < TEMP_LOW)
    {
        alert = true;
        alertMsg += "Low temp: " + String(temp, 1) + "C\n";
    }

    if (hum > HUM_HIGH)
    {
        alert = true;
        alertMsg += "High hum: " + String(hum, 1) + "%\n";
    }
    else if (hum < HUM_LOW && hum != -999)
    {
        alert = true;
        alertMsg += "Low hum: " + String(hum, 1) + "%";
    }

    if (alert)
    {
        Serial.println("Queueing alert (deferred send)!");
        if (!enqueueSMS(alertNumber, alertMsg))
        {
            Serial.println("Alert queue full — cannot enqueue!");
        }
        else
        {
            nodeTable[idx].alertSent = true;
            saveNeeded = true;
        }
    }
}

// Check node online/offline status
void checkNodeStatus()
{
    unsigned long now = millis();

    for (int i = 0; i < nodeCount; i++)
    {
        if (nodeTable[i].online)
        {
            // if no data for more than 2 minutes -> offline
            if (now - nodeTable[i].lastUpdate > 120000)
            {
                nodeTable[i].online = false;
                Serial.println("Node offline: " + String(nodeTable[i].room));
                saveNeeded = true;
            }
        }
    }
}

// Save node table to EEPROM
void saveNodeTable()
{
    EEPROM.put(NODE_TABLE_ADDR, nodeTable);
    EEPROM.put(NODE_TABLE_ADDR + sizeof(nodeTable), nodeCount);
    EEPROM.commit();
}

// Load node table from EEPROM
void loadNodeTable()
{
    EEPROM.get(NODE_TABLE_ADDR, nodeTable);
    EEPROM.get(NODE_TABLE_ADDR + sizeof(nodeTable), nodeCount);

    if (nodeCount > MAX_NODES || nodeCount < 0)
    {
        nodeCount = 0;
        Serial.println("Node table empty");
    }
    else
    {
        Serial.println(String(nodeCount) + " nodes loaded from EEPROM");
    }
}

// ============ LCD1602A display functions ============
void updateDisplay()
{
    lcd.clear();

    if (nodeCount == 0)
    {
        lcd.setCursor(0, 0);
        lcd.print("No Nodes");
        lcd.setCursor(0, 1);
        lcd.print("Connected");
        return;
    }

    // rotate through nodes
    if (displayIndex >= nodeCount)
    {
        displayIndex = 0;
    }

    NodeData &node = nodeTable[displayIndex];

    // first line: room name and status
    lcd.setCursor(0, 0);
    lcd.print(node.room);

    // pad room name to 10 chars
    int roomLen = strlen(node.room);
    if (roomLen < 10)
    {
        for (int i = roomLen; i < 10; i++)
        {
            lcd.print(" ");
        }
    }

    lcd.print(node.online ? " [ON]" : "[OFF]");

    // second line: sensor data
    lcd.setCursor(0, 1);

    if (String(node.sensorType).indexOf("hum") >= 0)
    {
        // show temperature and humidity
        lcd.print("T:");
        lcd.print(node.value1, 1);
        lcd.print("C H:");
        lcd.print(node.value2, 0);
        lcd.print("%");
    }
    else
    {
        // temperature only
        lcd.print("Temp: ");
        lcd.print(node.value1, 1);
        lcd.print("C");
    }

    displayIndex++;
}

// ============ MESH callbacks ============
void receivedCallback(uint32_t from, String &msg)
{
    Serial.println("\nMESH message received:");
    Serial.println("  From: " + String(from));
    Serial.println("  Content: " + msg);

    // parse message separated by '|'
    String parts[10];
    int idx = 0;
    int lastPos = 0;

    for (int i = 0; i < msg.length() && idx < 10; i++)
    {
        if (msg[i] == '|')
        {
            parts[idx++] = msg.substring(lastPos, i);
            lastPos = i + 1;
        }
    }
    if (idx < 10)
        parts[idx] = msg.substring(lastPos);

    if (parts[0] == "HELLO")
    {
        String nodeId = parts[1];
        String room = parts[2];
        String sensorType = parts[3];

        int nodeIdx = addOrUpdateNode(nodeId, room, sensorType);

        // send ACK
        String ack = "ACK|" + nodeId + "|registered";
        mesh.sendSingle(from, ack);
        Serial.println("ACK sent");
    }
    else if (parts[0] == "DATA")
    {
        String nodeId = parts[1];
        String room = parts[2];
        String sensorType = parts[3];
        String values = parts[4];
        String seq = parts[6];

        int nodeIdx = findNode(nodeId);
        if (nodeIdx < 0)
        {
            nodeIdx = addOrUpdateNode(nodeId, room, sensorType);
        }

        // parse values
        float val1 = -999, val2 = -999;
        int commaPos = values.indexOf(',');

        if (commaPos > 0)
        {
            val1 = values.substring(0, commaPos).toFloat();
            val2 = values.substring(commaPos + 1).toFloat();
        }
        else
        {
            val1 = values.toFloat();
        }

        updateNodeData(nodeIdx, val1, val2);

        // send ACK
        String ack = "ACK|" + nodeId + "|" + seq;
        mesh.sendSingle(from, ack);
        Serial.println("ACK sent");
    }
}

void newConnectionCallback(uint32_t nodeId)
{
    Serial.println("New connection: " + String(nodeId));
}

void changedConnectionCallback()
{
    Serial.println("Connections changed");
}

void nodeTimeAdjustedCallback(int32_t offset)
{
    Serial.println("Time adjusted");
}

// ============ Setup ============
void setup()
{
    Serial.begin(115200);
    Serial.println("\n\n========================================");
    Serial.println("ESP32 Gateway (Master)");
    Serial.println("========================================\n");

    // initialize EEPROM and load table
    EEPROM.begin(EEPROM_SIZE);
    loadNodeTable();

    // initialize LCD1602A
    lcd.begin(16, 2);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("NH3 Gateway");
    lcd.setCursor(0, 1);
    lcd.print("Starting...");
    Serial.println("LCD initialized");

    delay(2000);

    // initialize SIM800L
    lcd.setCursor(0, 1);
    lcd.print("Init GSM...     ");

    if (!initSIM800())
    {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("SIM800 Error!");
        Serial.println("Error: SIM800L not initialized");
    }
    else
    {
        lcd.setCursor(0, 1);
        lcd.print("GSM Ready       ");
        delay(1000);
    }

    // initialize MESH
    lcd.setCursor(0, 1);
    lcd.print("Init MESH...    ");

    mesh.setDebugMsgTypes(ERROR | STARTUP | CONNECTION);
    mesh.init(MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT);
    mesh.setRoot(true);
    mesh.setContainsRoot(true);

    mesh.onReceive(&receivedCallback);
    mesh.onNewConnection(&newConnectionCallback);
    mesh.onChangedConnections(&changedConnectionCallback);
    mesh.onNodeTimeAdjusted(&nodeTimeAdjustedCallback);

    Serial.println("MESH network started (Root)");

    // start tasks
    userScheduler.addTask(taskCheckSMS);
    userScheduler.addTask(taskUpdateDisplay);
    userScheduler.addTask(taskCheckNodeStatus);

    taskCheckSMS.enable();
    taskUpdateDisplay.enable();
    taskCheckNodeStatus.enable();

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("System Ready!");
    lcd.setCursor(0, 1);
    lcd.print("Nodes: 0");

    Serial.println("\nSystem ready!\n");

    delay(2000);
}

// ============ Loop ============
void loop()
{
    // mesh update + scheduler
    mesh.update();

    // run GSM state machine frequently
    sim800.loop();

    // process at most one queued SMS every SMS_PROCESS_INTERVAL_MS
    unsigned long now = millis();
    if ((now - lastSMSProcessTime) >= SMS_PROCESS_INTERVAL_MS)
    {
        QueuedSMS q;
        if (dequeueSMS(q))
        {
            Serial.println("\nProcessing queued SMS:");
            Serial.println(" To: " + q.number);
            Serial.println(" Msg: " + q.message);

            // send using library (non-blocking)
            sim800.sendSMS(q.number, q.message);
            Serial.println("SMS send invoked (deferred).");

            // short delay to give modem breathing room
            delay(200); // short delay
        }
        lastSMSProcessTime = now;
    }

    // deferred EEPROM save (avoid frequent commits)
    static unsigned long lastSaveAttempt = 0;
    const unsigned long SAVE_INTERVAL_MS = 5000;
    if (saveNeeded && (now - lastSaveAttempt) >= SAVE_INTERVAL_MS)
    {
        saveNodeTable();
        saveNeeded = false;
        lastSaveAttempt = now;
        Serial.println("Node table saved to EEPROM (deferred).");
    }

    // small yield
    delay(10);
}
