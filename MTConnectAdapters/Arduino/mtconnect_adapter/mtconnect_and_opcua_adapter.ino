/*
 * MTConnect + OPC UA Server for Arduino Opta
 *
 * This sketch creates both:
 * - MTConnect adapter server on port 7878 (SHDR protocol)
 * - OPC UA server on port 4840
 *
 * Monitors digital inputs A0, A1 for tool status detection
 * Provides LED control via OPC UA writes
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <Arduino_OPC_UA.h>
#include <PortentaEthernet.h>
#include <OptaBlue.h>
#include <mbed_rtc_time.h>

/**************************************************************************************
 * CONSTANTS
 **************************************************************************************/

#define LED_PIN A2
#define MTCONNECT_PORT 7878

const uint16_t HEARTBEAT_TIMEOUT = 10000;
const uint16_t TRANSITION_TIME_DELAY = 3000;

const uint8_t DATAITEMS_NB = 2;
const String DATAITEM_IDS[] = {"A1ToolPlus", "A2ToolPlus"};
const uint8_t DATAITEM_PINS[] = {A0, A1};

// OPC UA namespace and device identifiers to match Python supervisor
const char* NAMESPACE_URI = "demofactory";
const char* DEVICE_BROWSE_NAME = "IVAC";

/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/

static size_t const OPC_UA_SERVER_THREAD_STACK_SIZE = 16*1024UL;
template <size_t SIZE> struct alignas(uint32_t) OPC_UA_STACK final : public std::array<uint8_t, SIZE> {};
static OPC_UA_STACK<OPC_UA_SERVER_THREAD_STACK_SIZE> OPC_UA_SERVER_THREAD_STACK;

static size_t const OPC_UA_SERVER_THREAD_HEAP_SIZE = 320*1024UL;
template <size_t SIZE> struct alignas(O1HEAP_ALIGNMENT) OPC_UA_HEAP final : public std::array<uint8_t, SIZE> {};
static OPC_UA_HEAP<OPC_UA_SERVER_THREAD_HEAP_SIZE> OPC_UA_SERVER_THREAD_HEAP;

UA_Server * opc_ua_server = nullptr;
O1HeapInstance * o1heap_ins = nullptr;
rtos::Thread opc_ua_server_thread(osPriorityNormal, OPC_UA_SERVER_THREAD_STACK.size(), OPC_UA_SERVER_THREAD_STACK.data());

opcua::Opta::SharedPtr opta_opcua;

// MTConnect variables
EthernetServer mtconnectServer(MTCONNECT_PORT);
EthernetClient mtconnect_client;
String incoming = "";
boolean alreadyConnected = false;
String currState[DATAITEMS_NB];
String ledState = "false";

// OPC UA custom namespace and device node
UA_UInt16 custom_namespace_idx = 0;
UA_NodeId device_node_id = UA_NODEID_NULL;

/**************************************************************************************
 * DEFINES
 **************************************************************************************/

REDIRECT_STDOUT_TO(Serial)

/**************************************************************************************
 * OPC UA METHOD CALLBACKS
 **************************************************************************************/

// Method callback for LED control
static UA_StatusCode ledControlMethodCallback(UA_Server *server,
                                            const UA_NodeId *sessionId, void *sessionContext,
                                            const UA_NodeId *methodId, void *methodContext,
                                            const UA_NodeId *objectId, void *objectContext,
                                            size_t inputSize, const UA_Variant *input,
                                            size_t outputSize, UA_Variant *output) {

    if (inputSize == 1 && input[0].type == &UA_TYPES[UA_TYPES_STRING]) {
        UA_String *inputString = (UA_String*)input[0].data;
        String command = String((char*)inputString->data).substring(0, inputString->length);

        UA_LOG_INFO(UA_Log_Stdout, UA_LOGCATEGORY_SERVER, "LED Control command received: %s", command.c_str());

        String result = "OK";

        // Parse command
        if (command.equalsIgnoreCase("ON") || command.equalsIgnoreCase("true") || command.equalsIgnoreCase("1")) {
            pinMode(LED_PIN, OUTPUT);
            digitalWrite(LED_PIN, HIGH);
            ledState = "true";
            UA_LOG_INFO(UA_Log_Stdout, UA_LOGCATEGORY_SERVER, "LED turned ON");
        }
        else if (command.equalsIgnoreCase("OFF") || command.equalsIgnoreCase("false") || command.equalsIgnoreCase("0")) {
            pinMode(LED_PIN, OUTPUT);
            digitalWrite(LED_PIN, LOW);
            ledState = "false";
            UA_LOG_INFO(UA_Log_Stdout, UA_LOGCATEGORY_SERVER, "LED turned OFF");
        }
        else if (command.equalsIgnoreCase("TOGGLE")) {
            pinMode(LED_PIN, OUTPUT);
            bool currentState = digitalRead(LED_PIN);
            digitalWrite(LED_PIN, !currentState);
            ledState = !currentState ? "true" : "false";
            UA_LOG_INFO(UA_Log_Stdout, UA_LOGCATEGORY_SERVER, "LED toggled to %s", ledState.c_str());
        }
        else {
            result = "Unknown command. Use: ON, OFF, TOGGLE";
            UA_LOG_WARNING(UA_Log_Stdout, UA_LOGCATEGORY_SERVER, "Unknown LED command: %s", command.c_str());
        }

        // Set output
        if (outputSize == 1) {
            UA_String resultString = UA_STRING_ALLOC(result.c_str());
            UA_Variant_setScalarCopy(&output[0], &resultString, &UA_TYPES[UA_TYPES_STRING]);
            UA_String_clear(&resultString);
        }
    }

    return UA_STATUSCODE_GOOD;
}

/**************************************************************************************
 * LOCAL FUNCTIONS
 **************************************************************************************/

static float arduino_opta_analog_read(pin_size_t const pin)
{
  static float const VOLTAGE_MAX = 3.3;
  static float const RESOLUTION  = 4096.0;
  static float const DIVIDER     = 0.3034;

  int const pin_value = analogRead(pin);
  float const pin_voltage = pin_value * (VOLTAGE_MAX / RESOLUTION) / DIVIDER;
  return pin_voltage;
}

static PinStatus arduino_opta_digital_read(pin_size_t const pin)
{
  float const pin_voltage = arduino_opta_analog_read(pin);
  if (pin_voltage > 5.f)
    return HIGH;
  else
    return LOW;
}

// Create custom namespace and device node
static UA_StatusCode createCustomNamespaceAndDevice(UA_Server *server) {
    // Add custom namespace
    UA_String namespaceUri = UA_STRING((char*)NAMESPACE_URI);
    custom_namespace_idx = UA_Server_addNamespace(server, NAMESPACE_URI);

    UA_LOG_INFO(UA_Log_Stdout, UA_LOGCATEGORY_SERVER,
                "Created namespace '%s' with index %d", NAMESPACE_URI, custom_namespace_idx);

    // Create device object node
    UA_ObjectAttributes deviceAttr = UA_ObjectAttributes_default;
    deviceAttr.displayName = UA_LOCALIZEDTEXT("en-US", (char*)DEVICE_BROWSE_NAME);
    deviceAttr.description = UA_LOCALIZEDTEXT("en-US", "Arduino Opta Device Controller");

    device_node_id = UA_NODEID_STRING(custom_namespace_idx, (char*)DEVICE_BROWSE_NAME);

    UA_StatusCode retval = UA_Server_addObjectNode(server,
                                                  device_node_id,
                                                  UA_NODEID_NUMERIC(0, UA_NS0ID_OBJECTSFOLDER),
                                                  UA_NODEID_NUMERIC(0, UA_NS0ID_ORGANIZES),
                                                  UA_QUALIFIEDNAME(custom_namespace_idx, (char*)DEVICE_BROWSE_NAME),
                                                  UA_NODEID_NUMERIC(0, UA_NS0ID_BASEOBJECTTYPE),
                                                  deviceAttr, NULL, NULL);

    if (retval == UA_STATUSCODE_GOOD) {
        UA_LOG_INFO(UA_Log_Stdout, UA_LOGCATEGORY_SERVER,
                    "Created device node '%s'", DEVICE_BROWSE_NAME);
    } else {
        UA_LOG_ERROR(UA_Log_Stdout, UA_LOGCATEGORY_SERVER,
                     "Failed to create device node: %s", UA_StatusCode_name(retval));
        return retval;
    }

    return UA_STATUSCODE_GOOD;
}

// Add methods to the device node
static UA_StatusCode addMethodsToDevice(UA_Server *server) {
    UA_StatusCode retval = UA_STATUSCODE_GOOD;

    // Add LED Control Method
    UA_MethodAttributes ledMethodAttr = UA_MethodAttributes_default;
    ledMethodAttr.description = UA_LOCALIZEDTEXT("en-US", "Control LED state");
    ledMethodAttr.displayName = UA_LOCALIZEDTEXT("en-US", "LEDControl");
    ledMethodAttr.executable = true;
    ledMethodAttr.userExecutable = true;

    // Input argument for LED method
    UA_Argument inputArgument;
    UA_Argument_init(&inputArgument);
    inputArgument.description = UA_LOCALIZEDTEXT("en-US", "LED command (ON/OFF/TOGGLE)");
    inputArgument.name = UA_STRING("command");
    inputArgument.dataType = UA_TYPES[UA_TYPES_STRING].typeId;
    inputArgument.valueRank = UA_VALUERANK_SCALAR;

    // Output argument for LED method
    UA_Argument outputArgument;
    UA_Argument_init(&outputArgument);
    outputArgument.description = UA_LOCALIZEDTEXT("en-US", "Command result");
    outputArgument.name = UA_STRING("result");
    outputArgument.dataType = UA_TYPES[UA_TYPES_STRING].typeId;
    outputArgument.valueRank = UA_VALUERANK_SCALAR;

    retval = UA_Server_addMethodNode(server,
                                    UA_NODEID_STRING(custom_namespace_idx, "LEDControl"),
                                    device_node_id,
                                    UA_NODEID_NUMERIC(0, UA_NS0ID_HASCOMPONENT),
                                    UA_QUALIFIEDNAME(custom_namespace_idx, "LEDControl"),
                                    ledMethodAttr,
                                    &ledControlMethodCallback,
                                    1, &inputArgument,
                                    1, &outputArgument,
                                    NULL, NULL);

    if (retval == UA_STATUSCODE_GOOD) {
        UA_LOG_INFO(UA_Log_Stdout, UA_LOGCATEGORY_SERVER, "Added LEDControl method");
    } else {
        UA_LOG_ERROR(UA_Log_Stdout, UA_LOGCATEGORY_SERVER, "Failed to add LEDControl method");
    }

    return retval;
}

// MTConnect SHDR Functions (unchanged)
void handleMTConnectClient(EthernetClient &client) {
  Serial.println("MTConnect agent connected");

  while(client.connected()) {
    if (!alreadyConnected) {
      // Send initial SHDR data on first connection
      if(Ethernet.linkStatus() == LinkON) {
        client.println("|avail|AVAILABLE\n");
        Serial.println("Sent: |avail|AVAILABLE\n");
      } else {
        client.println("|avail|UNAVAILABLE\n");
        Serial.println("Sent: |avail|UNAVAILABLE\n");
      }

      client.println("* shdrVersion: 2.0\n");
      Serial.println("Sent: * shdrVersion: 2.0\n");

      client.println("* adapterVersion: 2.0\n");
      Serial.println("Sent: * adapterVersion: 2.0\n");

      // Send initial tool states
      for(int i = 0; i < DATAITEMS_NB; i++) {
        currState[i] = (arduino_opta_digital_read(DATAITEM_PINS[i]) == HIGH ? "OFF" : "ON");
        sendSHDRStringData(client, DATAITEM_IDS[i], currState[i]);
      }

      alreadyConnected = true;
    } else {
      String newState[DATAITEMS_NB];

      for(int i = 0; i < DATAITEMS_NB; i++) {
        newState[i] = (arduino_opta_digital_read(DATAITEM_PINS[i]) == HIGH ? "OFF" : "ON");

        if(newState[i] != currState[i]) {
          delay(TRANSITION_TIME_DELAY);
          sendSHDRStringData(client, DATAITEM_IDS[i], newState[i]);
          currState[i] = newState[i];
        }
      }
    }

    // Handle incoming PING/PONG heartbeat
    if (client.available()) {
      char c = client.read();
      incoming += c;

      if(incoming.indexOf("* PING") >= 0) {
        client.println("* PONG " + String(HEARTBEAT_TIMEOUT) + "\n");
        Serial.println("Sent: * PONG " + String(HEARTBEAT_TIMEOUT) + "\n");
        incoming = "";
      }
    }
  }

  client.stop();
  Serial.println("MTConnect agent disconnected");
  alreadyConnected = false;
}

void sendSHDRStringData(EthernetClient &client, String dataitemId, String value) {
  client.println("|"+ dataitemId + "|" + value + "\n");
  Serial.println("Sent: |"+ dataitemId + "|" + value + "\n");
}

/**************************************************************************************
 * SETUP/LOOP
 **************************************************************************************/

void setup()
{
  Serial.begin(115200);
  auto const start = millis();
  while(!Serial)

  // Initialize pins
  pinMode(LED_PIN, OUTPUT);
  for(int i = 0; i < DATAITEMS_NB; i++){
    pinMode(DATAITEM_PINS[i], INPUT);
  }

  // Initialize Ethernet interface
  if (!Ethernet.begin()) {
    Serial.println("\"Ethernet.begin()\" failed.");
    for (;;) { }
  }

  // Setup NTP time for OPC UA
  EthernetUDP udp_client;
  auto const epoch = opcua::NTPUtils::getTime(udp_client);
  if (epoch > 0) {
    set_time(epoch);
  } else {
    set_time(opcua::timeToStr(__DATE__));
  }

  // Initialize Opta Controller
  OptaController.begin();
  OptaController.update();

  // Initialize heap memory for OPC UA
  o1heap_ins = o1heapInit(OPC_UA_SERVER_THREAD_HEAP.data(), OPC_UA_SERVER_THREAD_HEAP.size());
  if (o1heap_ins == nullptr) {
    Serial.println("\"o1heapInit\" failed.");
    for (;;) { }
  }
  UA_mallocSingleton  = o1heap_malloc;
  UA_freeSingleton    = o1heap_free;
  UA_callocSingleton  = o1heap_calloc;
  UA_reallocSingleton = o1heap_realloc;

  // Start OPC UA server in separate thread
  opc_ua_server_thread.start(
    +[]()
    {
      // Create OPC UA server
      opc_ua_server = UA_Server_new();

      UA_LOG_INFO(UA_Log_Stdout, UA_LOGCATEGORY_SERVER,
                  "Arduino Opta IP: %s", Ethernet.localIP().toString().c_str());

      // Create custom namespace and device node
      if (createCustomNamespaceAndDevice(opc_ua_server) != UA_STATUSCODE_GOOD) {
        UA_LOG_ERROR(UA_Log_Stdout, UA_LOGCATEGORY_SERVER, "Failed to create custom namespace/device");
        return;
      }

      // Add methods to device node
      if (addMethodsToDevice(opc_ua_server) != UA_STATUSCODE_GOOD) {
        UA_LOG_ERROR(UA_Log_Stdout, UA_LOGCATEGORY_SERVER, "Failed to add methods to device");
        return;
      }

      // Determine Arduino OPC UA hardware variant for standard nodes
      opcua::OptaVariant::Type opta_type;
      if (opcua::OptaVariant::getOptaVariant(opta_type)) {
        // Pre-configure analog pins
        std::list<pin_size_t> const ADC_PIN_LIST = { A0, A1, A2, A3, A4, A5, A6, A7 };
        for (auto const adc_pin : ADC_PIN_LIST)
          arduino_opta_analog_read(adc_pin);
        analogReadResolution(12);

        // Create Arduino Opta OPC UA object (for standard nodes)
        opta_opcua = opcua::Opta::create(opc_ua_server, opta_type);
        if (opta_opcua) {
          UA_LOG_INFO(UA_Log_Stdout, UA_LOGCATEGORY_SERVER, "Created standard Opta OPC UA nodes");
        }
      }

      UA_LOG_INFO(UA_Log_Stdout, UA_LOGCATEGORY_SERVER,
                  "stack: size = %d | free = %d | used = %d | max = %d",
                  opc_ua_server_thread.stack_size(),
                  opc_ua_server_thread.free_stack(),
                  opc_ua_server_thread.used_stack(),
                  opc_ua_server_thread.max_stack());

      UA_LOG_INFO(UA_Log_Stdout, UA_LOGCATEGORY_SERVER,
                  "o1heap: capacity: %d | allocated: %d | peak_allocated: %d",
                  o1heapGetDiagnostics(o1heap_ins).capacity,
                  o1heapGetDiagnostics(o1heap_ins).allocated,
                  o1heapGetDiagnostics(o1heap_ins).peak_allocated);

      UA_LOG_INFO(UA_Log_Stdout, UA_LOGCATEGORY_SERVER, "OPC UA server setup complete");

      UA_StatusCode const status = UA_Server_runUntilInterrupt(opc_ua_server);
    });

  // Start MTConnect server
  mtconnectServer.begin();

  pinMode(LED_BUILTIN, OUTPUT);

  Serial.println("Servers started:");
  Serial.print("OPC UA: opc.tcp://");
  Serial.print(Ethernet.localIP());
  Serial.println(":4840");
  Serial.print("MTConnect: http://");
  Serial.print(Ethernet.localIP());
  Serial.print(":");
  Serial.println(MTCONNECT_PORT);
  Serial.println("Custom namespace: " + String(NAMESPACE_URI));
  Serial.println("Device browse name: " + String(DEVICE_BROWSE_NAME));
}

void loop()
{
  // Update Opta controller
  OptaController.update();

  // Handle MTConnect connections
  mtconnect_client = mtconnectServer.accept();
  if (mtconnect_client) {
    handleMTConnectClient(mtconnect_client);
  }
}