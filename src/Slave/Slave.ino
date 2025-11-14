/*
 * NodeMCU Sensor Node - MESH Network (Modified)
 * Changes:
 *  - send via broadcast instead of sendSingle to gatewayId
 *  - setContainsRoot(true)
 *  - improved seq and retry handling
 */

#include <painlessMesh.h>
#include <DHT.h>

// ============ MESH CONFIG ============
#define MESH_PREFIX "SensorMesh"
#define MESH_PASSWORD "mesh12345"
#define MESH_PORT 5555

// ============ NODE CONFIG ============
#define NODE_ID "node01"
#define ROOM_NAME "room1"
#define SENSOR_TYPE "temp"

// ============ DHT SENSOR CONFIG ============
#define DHT_PIN D4
#define DHT_TYPE DHT11
DHT dht(DHT_PIN, DHT_TYPE);

// ============ BUZZER CONFIG ============
#define BUZZER_PIN D5
#define ALERT_DURATION 10000

// ============ ALERT THRESHOLDS ============
#define TEMP_HIGH 30.0
#define TEMP_LOW 10.0
#define HUM_HIGH 80.0
#define HUM_LOW 20.0

// ============ TIMING CONFIG ============
#define SEND_INTERVAL 30000 // send every 30 seconds (adjustable)
#define ACK_TIMEOUT 8000    // ACK timeout (increased for multi-hop)
#define MAX_RETRY 3

// ============ VARIABLES ============
Scheduler userScheduler;
painlessMesh mesh;

uint16_t messageSeq = 0;
bool waitingForAck = false;
uint32_t lastAckTime = 0;
uint8_t retryCount = 0;

float lastTemp = 0;
float lastHum = 0;
unsigned long buzzerStartTime = 0;
bool buzzerActive = false;

// store last message/seq for retry
String lastMessage = "";
String lastSeqString = "";
bool lastWasHello = false;

void sendSensorData();
void checkAckTimeout();

// ============ TASKS ============
Task taskSendSensor(SEND_INTERVAL, TASK_FOREVER, &sendSensorData);
Task taskCheckAck(ACK_TIMEOUT, TASK_FOREVER, &checkAckTimeout);

// ============================================================================
// Helper Functions
// ============================================================================

// Read sensor
void readSensor(float &temp, float &hum)
{
    temp = dht.readTemperature();
    hum = dht.readHumidity();

    if (isnan(temp))
        temp = 10;
    if (isnan(hum))
        hum = 20;
}

// Check thresholds
bool checkThresholdAlert(float temp, float hum)
{
    bool alert = false;

    if (temp > TEMP_HIGH || temp < TEMP_LOW)
    {
        alert = true;
    }

    if (hum > HUM_HIGH || hum < HUM_LOW)
    {
        alert = true;
    }

    return alert;
}

// Activate buzzer
void activateBuzzer()
{
    digitalWrite(BUZZER_PIN, HIGH);
    buzzerStartTime = millis();
    buzzerActive = true;
    Serial.println("Buzzer activated!");
}

// Deactivate buzzer
void deactivateBuzzer()
{
    digitalWrite(BUZZER_PIN, LOW);
    buzzerActive = false;
    Serial.println("Buzzer deactivated.");
}

// Build HELLO message and prepare lastMessage (for potential ACK)
String buildHelloMessage()
{
    String msg = "HELLO|";
    msg += NODE_ID;
    msg += "|";
    msg += ROOM_NAME;
    msg += "|";
    msg += SENSOR_TYPE;

    Serial.println("Prepared HELLO: " + msg);
    return msg;
}

// Prepare (new) DATA message: increments seq and stores lastMessage/lastSeqString
String prepareNewDataMessage(float temp, float hum)
{
    messageSeq++; // increment only when creating a new message
    lastSeqString = "seq" + String(messageSeq);

    String msg = "DATA|";
    msg += NODE_ID;
    msg += "|";
    msg += ROOM_NAME;
    msg += "|";
    msg += SENSOR_TYPE;
    msg += "|";

    if (String(SENSOR_TYPE) == "temp")
    {
        msg += String(temp, 1);
    }
    else if (String(SENSOR_TYPE) == "humidity")
    {
        msg += String(hum, 1);
    }
    else
    {
        msg += String(temp, 1) + "," + String(hum, 1);
    }

    msg += "|";
    msg += String(millis()); // timestamp
    msg += "|";
    msg += lastSeqString;

    lastMessage = msg;
    lastWasHello = false;
    return msg;
}

// Prepare HELLO as lastMessage (registered ack returned as 'registered')
String prepareHelloAsLast()
{
    String hello = buildHelloMessage();
    lastMessage = hello;
    lastSeqString = "registered"; // gateway returns 'registered' for HELLO ACK
    lastWasHello = true;
    return hello;
}

// Send lastMessage via broadcast (used for new send or retry)
void sendLastMessage()
{
    if (lastMessage.length() == 0)
    {
        Serial.println("No lastMessage to send.");
        return;
    }
    Serial.println("Broadcasting: " + lastMessage);
    mesh.sendBroadcast(lastMessage);
    waitingForAck = true;
    lastAckTime = millis();
    taskCheckAck.enable();
}

// Send a fresh data message (build + broadcast)
void sendFreshData(float temp, float hum)
{
    prepareNewDataMessage(temp, hum);
    sendLastMessage();
}

// ============================================================================
// MESH CALLBACKS
// ============================================================================

// On message received
void receivedCallback(uint32_t from, String &msg)
{
    Serial.println("\nMessage received from: " + String(from));
    Serial.println("  Content: " + msg);

    if (msg.startsWith("ACK|"))
    {
        // parse safely
        String parts[5]; // expect at least 3 parts; 5 is sufficient
        int idx = 0;
        int lastPos = 0;

        for (int i = 0; i < msg.length() && idx < 4; i++)
        {
            if (msg[i] == '|')
            {
                parts[idx++] = msg.substring(lastPos, i);
                lastPos = i + 1;
            }
        }
        // put the remainder into parts[idx]
        if (idx < 4)
            parts[idx] = msg.substring(lastPos);

        // validate format (need parts[0], parts[1], parts[2] at minimum)
        if (parts[0] != "ACK")
        {
            Serial.println("ACK format mismatch.");
            return;
        }

        // ensure required indices exist
        // parts[1] -> nodeId, parts[2] -> seq OR 'registered'
        if (parts[1].length() == 0 || parts[2].length() == 0)
        {
            Serial.println("ACK missing fields.");
            return;
        }

        // check ACK is for this node
        if (parts[1] == NODE_ID)
        {
            // accept three cases:
            //  - gateway replies 'registered' for HELLO
            //  - or for DATA it replies with same seq (e.g., "seq3")
            if (parts[2] == lastSeqString || (parts[2] == "registered" && lastWasHello))
            {
                Serial.println("ACK valid for this node: " + parts[2]);
                waitingForAck = false;
                retryCount = 0;
                taskCheckAck.disable();
                // optionally clear last message after ACK
                lastMessage = "";
                lastSeqString = "";
                lastWasHello = false;
            }
            else
            {
                Serial.println("ACK seq didn't match. Received: " + parts[2] + " expected: " + lastSeqString);
            }
        }
        else
        {
            Serial.println("ACK not for this node (id mismatch).");
        }
    }
}

// New connection detected
void newConnectionCallback(uint32_t nodeId)
{
    Serial.println("New connection: " + String(nodeId));

    // when a new connection appears, broadcast HELLO to register
    delay(500);
    String hello = prepareHelloAsLast();
    sendLastMessage();
}

void changedConnectionCallback()
{
    Serial.println("Network connection status changed.");
    // for debugging, print node list (optional)
    auto nodeList = mesh.getNodeList();
    Serial.print("NodeList: ");
    for (const auto &nodeId : nodeList)
    {
        Serial.print(String(nodeId) + " ");
    }
    Serial.println();
}

void nodeTimeAdjustedCallback(int32_t offset)
{
    Serial.println("Network time adjusted. Offset: " + String(offset));
}

// ============================================================================
// SEND / ACK TIMEOUT CHECK
// ============================================================================

void sendSensorData()
{
    // If previous ACK not received and retries remain, skip sending a new message
    if (waitingForAck && retryCount < MAX_RETRY)
    {
        Serial.println("Waiting for previous ACK... (retry " + String(retryCount) + ")");
        return;
    }
    else if (waitingForAck && retryCount >= MAX_RETRY)
    {
        // if retries exhausted, allow sending a new message (or consider resetting gateway)
        Serial.println("Previous send failed after retries. Will attempt new send.");
        waitingForAck = false;
        retryCount = 0;
        taskCheckAck.disable();
    }

    float temp, hum;
    readSensor(temp, hum);

    Serial.println("\nSensor Reading:");
    Serial.println("  Temperature: " + String(temp, 1) + "°C");
    Serial.println("  Humidity: " + String(hum, 1) + "%");

    // Threshold check
    if (checkThresholdAlert(temp, hum))
    {
        activateBuzzer();
    }

    // store for possible retry
    lastTemp = temp;
    lastHum = hum;

    // prepare & send
    sendFreshData(temp, hum);
}

void checkAckTimeout()
{
    if (!waitingForAck)
    {
        taskCheckAck.disable();
        return;
    }

    if (millis() - lastAckTime > ACK_TIMEOUT)
    {
        retryCount++;
        Serial.println("ACK not received! Retry " + String(retryCount) + "/" + String(MAX_RETRY));

        if (retryCount >= MAX_RETRY)
        {
            Serial.println("Sending failed after max retries. Resetting state.");
            waitingForAck = false;
            retryCount = 0;
            taskCheckAck.disable();
            // clear or take other action (e.g., send HELLO again)
            lastMessage = "";
            lastSeqString = "";
            lastWasHello = false;
        }
        else
        {
            // resend same lastMessage (without incrementing seq)
            Serial.println("Resending last message: " + lastMessage);
            mesh.sendBroadcast(lastMessage);
            lastAckTime = millis();
            taskCheckAck.enable();
        }
    }
}

// ============================================================================
// SETUP
// ============================================================================
void setup()
{
    Serial.begin(115200);
    Serial.println("\n\n========================================");
    Serial.println("NodeMCU Sensor Node (fixed)");
    Serial.println("  Node ID: " + String(NODE_ID));
    Serial.println("  Room: " + String(ROOM_NAME));
    Serial.println("  Sensor: " + String(SENSOR_TYPE));
    Serial.println("========================================\n");

    // Buzzer setup
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);

    // Sensor setup
    dht.begin();
    Serial.println("DHT sensor initialized.");

    // Mesh setup
    // enable useful debug messages (ERROR | STARTUP | CONNECTION)
    mesh.setDebugMsgTypes(ERROR | STARTUP | CONNECTION);
    mesh.init(MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT);

    // Important: tell node that root exists (helps routing when root is present)
    mesh.setContainsRoot(true);

    mesh.onReceive(&receivedCallback);
    mesh.onNewConnection(&newConnectionCallback);
    mesh.onChangedConnections(&changedConnectionCallback);
    mesh.onNodeTimeAdjusted(&nodeTimeAdjustedCallback);

    Serial.println("MESH network initialized.");

    userScheduler.addTask(taskSendSensor);
    userScheduler.addTask(taskCheckAck);

    taskSendSensor.enable();

    Serial.println("System ready!\n");

    // send an initial HELLO so gateway knows this node exists
    delay(500);
    String hello = prepareHelloAsLast();
    sendLastMessage();
}

// ============================================================================
// LOOP
// ============================================================================
void loop()
{
    mesh.update();

    if (buzzerActive && (millis() - buzzerStartTime > ALERT_DURATION))
    {
        deactivateBuzzer();
    }
}
