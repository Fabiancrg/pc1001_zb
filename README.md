# ESP32-C6 PC1001 Pool Heat Pump Controller - Zigbee

ESP32-C6 Zigbee router device for controlling Hayward/Majestic PC1001 pool heat pump controllers via Manchester-encoded serial protocol.

## 🌟 Features

- **Zigbee End Device** - Mains powered, non-sleepy device (powered off seasonally)
- **Thermostat Cluster** - Standard Zigbee thermostat with heating/cooling/auto modes
- **PC1001 Protocol** - Full bidirectional Manchester protocol implementation
- **Real-time Monitoring** - Water inlet/outlet temperature, programmed setpoint, mode
- **Home Assistant Ready** - Works with Zigbee2MQTT and ZHA

## 📋 Hardware Requirements

- **ESP32-C6** development board (e.g., ESP32-C6-DevKitC-1)
- **PC1001 Controller** - Hayward/Majestic heat pump with PC1001 control interface
- **Single wire connection** - GPIO2 to PC1001 NET data line (bidirectional)
- **Power supply** - 5V from PC1001

## 🔌 Wiring

```
ESP32-C6          PC1001 Controller
--------          -----------------
GPIO2    <---->   DATA LINE (via level shifter - see below)
GND      <---->   GND
+5V      <---->   +5V (available on CN16 connector)
```

### ⚠️ Level Shifter Requirements (CRITICAL)

The PC1001 NET data line operates at **5V TTL** logic levels while the ESP32-C6 uses **3.3V** GPIO. A bidirectional level shifter is **mandatory** for reliable communication.

#### ❌ **NOT RECOMMENDED: TXB0102DCUR**

The **TXB0102** auto-direction sensing level shifter is **NOT suitable** for this application because:
- Auto-direction detection causes timing jitter on half-duplex Manchester-encoded signals
- Transition time (~10-30ns) can introduce errors in timing-critical protocol (1ms pulses)
- May cause communication errors or unreliable decoding

#### ✅ **RECOMMENDED Option 1: 74LVC1T45 (Push-Pull)**

**Best choice** for push-pull PC1001 output (most common configuration):

```
Component: Texas Instruments 74LVC1T45DBVR
Package: SOT-23-6
Price: ~$0.30-0.50
```

**Wiring:**
```
ESP32-C6 (3.3V)      74LVC1T45      PC1001 (5V)
-----------------    ---------      -----------
GPIO2    <--------->   A (Data)
3.3V     <--------->   VCCA
                       DIR          (controlled by GPIO or tied for fixed direction)
GND      <--------->   GND
5V       <--------->   VCCB
                       B (Data)  <-------> NET Pin
```

**Features:**
- Direction-controlled (no auto-sensing delays)
- Fast switching (3-5ns typical)
- 24mA output drive
- Low propagation delay

**DIR Pin Options:**
1. **Automatic switching**: Connect DIR to another GPIO, set HIGH for TX, LOW for RX
2. **Fixed bidirectional**: Add 10kΩ pull-up to 3.3V for auto-bidirectional operation (if supported by your board configuration)

#### ✅ **RECOMMENDED Option 2: BSS138 MOSFET (Open-Drain)**

**Alternative choice** if PC1001 uses open-drain output or for simpler circuit:

```
Component: BSS138 N-channel MOSFET + 2× 10kΩ resistors
Package: SOT-23
Price: ~$0.10-0.20
```

**Wiring:**
```
         10kΩ              10kΩ
3.3V ----/\/\----+----/\/\---- 5V
                 |
ESP32-C6         |              PC1001
GPIO2 -----------+--- BSS138 ---+------ NET Pin
                      (G-S-D)   |
                                |
GND ----------------------------|------ GND
```

**Schematic:**
```
    3.3V Rail          5V Rail
        |                 |
       10kΩ             10kΩ
        |                 |
        +---[BSS138]------+---> PC1001 NET
        |    Gate | Drain
  ESP32 GPIO     |
              Source
                 |
                GND
```

**Features:**
- Simple 2-transistor circuit
- Bidirectional without control signals
- Works with both push-pull and open-drain outputs
- ~50ns switching time (adequate for 1ms timing)

#### Additional Wiring Notes:

1. **Series Resistor**: Add 33-100Ω series resistor on DATA line for protection and signal integrity
2. **Ground Connection**: Ensure solid common ground between ESP32-C6 and PC1001
3. **Power Supply**: PC1001 CN16 connector provides 5V power for ESP32-C6
4. **Wire Length**: Keep data line < 30cm for reliable Manchester timing
5. **Shielding**: Consider twisted pair or shielded cable in electrically noisy environments

#### Determining PC1001 Output Type:

**To check if push-pull or open-drain:**
1. Measure NET pin voltage when idle with oscilloscope
2. **Push-pull**: Voltage switches between 0V and 5V cleanly
3. **Open-drain**: Requires pull-up resistor, voltage pulled high by resistor
4. Most PC1001 units use **push-pull** output → use 74LVC1T45

**Quick test without scope:**
- If PC1001 works with simple resistor divider, it's push-pull
- If it needs pull-up resistor to function, it's open-drain

## 🛠️ Building and Flashing

### Prerequisites

- ESP-IDF v5.5.1 or later
- ESP32-C6 support enabled

### Build Commands

```bash
# Set target to ESP32-C6
idf.py set-target esp32-c6

# Configure project (optional)
idf.py menuconfig

# Build
idf.py build

# Flash and monitor
idf.py -p COMx flash monitor
```

Replace `COMx` with your serial port (e.g., `COM3` on Windows, `/dev/ttyUSB0` on Linux).

## 📡 Zigbee Configuration

The device operates as a **Zigbee End Device** (mains powered, rx_on_when_idle=true) and exposes **2 endpoints**:

**Note**: Configured as an End Device rather than Router because the device is powered off during winter. End Devices can leave/rejoin the network without disrupting routing for other devices.

### Endpoint 1 - Thermostat
- **Cluster**: Thermostat (0x0201)
- **Attributes**:
  - `local_temperature` - Water inlet temperature (TEMP_IN, read-only, reportable)
  - `occupied_cooling_setpoint` - Target cooling temperature (read/write, reportable, 15-32°C)
  - `occupied_heating_setpoint` - Target heating temperature (read/write, reportable, 15-32°C)
  - `system_mode` - Operating mode (off/heat/cool/auto, read/write, reportable)
- **Also includes**: Basic cluster (manufacturer info), Identify cluster

### Endpoint 2 - Temperature Sensor (Water Outlet)
- **Cluster**: Temperature Measurement (0x0402)
- **Attributes**:
  - `measured_value` - Water outlet temperature (TEMP_OUT, read-only, reportable)
  - `min_measured_value` - -40°C
  - `max_measured_value` - 60°C
- **Also includes**: Identify cluster
- **Note**: No Basic cluster (only on EP1 per Zigbee best practice)

**ESP_ZB_ZCL_ATTR_ACCESS_REPORTING**: All key attributes have the REPORTING flag set, ensuring that reporting configuration persists across device reboots in the zb_storage partition.

### Supported Modes

| Zigbee Mode | PC1001 Mode | Description |
|-------------|-------------|-------------|
| `off` (0x00) | Power OFF | Heat pump disabled |
| `auto` (0x01) | AUTO | Automatic heating/cooling |
| `cool` (0x03) | COOL | Cooling only |
| `heat` (0x04) | HEAT | Heating only |

## 🏠 Home Assistant Integration

### Zigbee2MQTT

The device will appear as a thermostat with two temperature sensors:

**Endpoint 1 (Thermostat)**:
```yaml
climate:
  - platform: mqtt
    name: "Pool Heat Pump"
    modes:
      - "off"
      - "heat"
      - "cool"
      - "auto"
    temperature_unit: "C"
    current_temperature_topic: "zigbee2mqtt/PC1001/local_temperature"
    temperature_command_topic: "zigbee2mqtt/PC1001/occupied_heating_setpoint/set"
    mode_command_topic: "zigbee2mqtt/PC1001/system_mode/set"
```

**Endpoint 2 (Temperature Sensor)**:
```yaml
sensor:
  - platform: mqtt
    name: "Pool Water Outlet Temperature"
    state_topic: "zigbee2mqtt/PC1001/temperature_outlet"
    unit_of_measurement: "°C"
    device_class: temperature
```

### ZHA (Zigbee Home Automation)

The device will be auto-discovered with:
- **EP1**: Climate entity with current inlet temperature, target temperature, and mode selector
- **EP2**: Temperature sensor showing water outlet temperature
- Target temperature adjustable in 0.5°C steps (15-32°C)

## 📊 PC1001 Protocol Details

### Frame Structure

The PC1001 uses Manchester encoding over a single wire:
- **Timing-based** protocol (not UART)
- **Start bit**: 9ms LOW + 5ms HIGH
- **Binary 0**: 1ms LOW + 1ms HIGH
- **Binary 1**: 1ms LOW + 3ms HIGH
- **Frame sizes**: 9 bytes (short) or 12 bytes (long)
- **Repetition**: Commands sent 8-16 times depending on cycle
- **Checksum**: Last byte contains checksum of all previous bytes
- **Frame spacing**: 125ms between most frames, 1s after frame groups

### Frame Types (Extended Protocol Support)

The enhanced implementation supports multiple frame types from the PC1001 protocol:

| Frame Type | Byte 0 | Length | Contains |
|------------|--------|--------|----------|
| Temp OUT | 0x4B | 12 | Water outlet temperature |
| Temp IN | 0x8B | 12 | Water inlet temperature |
| Status | 0x81 | 12 | Programmed temp, mode, power state |
| Clock | 0xD1 | 12 | Time synchronization data (not implemented) |
| Config | 0x82 | 12 | Configuration parameters (not implemented) |
| Condition 1 | 0x83 | 12 | Additional sensor data (not implemented) |
| Condition 2 | 0x84 | 9 or 12 | Flow meter, defrost status (not implemented) |

**Note**: The driver currently implements the three essential frame types (Temp OUT, Temp IN, Status) needed for thermostat functionality. Additional frame types are recognized but not fully decoded - these could be implemented for advanced features like time sync, flow metering, or detailed diagnostics.

### Received Frames

The PC1001 broadcasts status frames continuously:

**Core Frame Types (Implemented)**:
- **0x4B (Temp OUT)**: Water outlet temperature in byte 4
- **0x8B (Temp IN)**: Water inlet temperature in byte 9  
- **0x81 (Status)**: Programmed temp (byte 4), mode/power (byte 2)

**Extended Frame Types (Recognized, Not Decoded)**:
- **0xD1 (Clock)**: Time/date synchronization
- **0x82 (Config)**: Heat pump configuration parameters
- **0x83-0x84 (Conditions)**: Additional diagnostics and sensor data

### Temperature Encoding

Temperature is encoded in 0.5°C steps from 15°C to 32°C:
- Bit 7 (0x80): Half degree flag
- Bits 6-1: Temperature value (reversed bits)
- Formula: `temp = (reversed_bits(byte) & 0x3E) >> 1) + 2`

### Mode Encoding

Modes are encoded in Byte 2:
- Bit 7 (0x80): Power state (1=ON, 0=OFF)
- Bit 3 (0x08): Heat mode
- Bit 2 (0x04): Auto mode
- Bit 0 (0x00): Cool mode (default when others not set)

## 🔧 Configuration

### GPIO Pin

Default data pin is **GPIO2**. To change:

Edit `main/esp_zb_pc1001.c`:
```c
#define PC1001_DATA_GPIO        GPIO_NUM_2  // Change to your GPIO
```

### Zigbee Channel

Default channel is **15** (to avoid WiFi interference). To change:

Edit `sdkconfig.defaults`:
```
CONFIG_ESP_ZB_PRIMARY_CHANNEL_MASK=0x4000  # Channel 15
```

Available channels: 11-26 (use mask: `1 << channel`)

### Temperature Limits

Default range is **15-32°C** in 0.5°C steps. These match PC1001 hardware limits and are set in the thermostat cluster configuration.

## 🐛 Debugging

### Enable Debug Logging

```bash
idf.py menuconfig
# Component config -> Log output -> Default log verbosity -> Debug
```

### Common Issues

**Problem**: No data received from PC1001
- Check wiring (GPIO2 to DATA line, GND to GND)
- Verify PC1001 is powered and operational
- Check voltage levels (may need level shifter)
- Monitor GPIO with oscilloscope to verify timing

**Problem**: Device not joining Zigbee network
- Ensure coordinator is in pairing mode
- Check Zigbee channel matches coordinator
- Try factory reset: Hold BOOT button for 10 seconds (if implemented)
- Check coordinator logs for pairing attempts

**Problem**: Commands not controlling PC1001
- Verify bidirectional communication (GPIO must be able to switch between input/output)
- Check timing accuracy (use logic analyzer)
- Ensure no other device is connected to PC1001 data line

## 📝 Protocol Implementation Notes

### Protocol Version

This implementation is based on extensive reverse engineering work from:
- **sle118/hayward_pool_heater** - Comprehensive ESPHome component  
- **atlas2003/njanik** - Original Arduino/MQTT implementation
- Community reverse engineering efforts documented in Arduino forums

The driver implements the **essential subset** of the full PC1001 protocol needed for thermostat operation:
- ✅ **Temperature monitoring** (inlet/outlet)
- ✅ **Mode control** (heat/cool/auto)
- ✅ **Power control** (on/off)
- ✅ **Checksum validation** for both 9-byte and 12-byte frames
- ⚠️ **Extended features** recognized but not decoded (clock sync, flow meter, detailed diagnostics)

### Frame Cycle Patterns

The full PC1001 protocol uses a **16-frame cycle pattern** with varied silence periods:
- **125ms spacing**: Between most frames within a cycle
- **1s spacing**: After specific frame groups (typically every 4th frame)

This implementation uses **8 repetitions** for commands (instead of full 16-frame cycle) to balance:
- ✅ Fast command response
- ✅ Reliable transmission (redundancy)  
- ✅ Backward compatibility with simpler protocol understanding

To enable full 16-frame cycle matching reference implementations, change `repetitions = 8` to `repetitions = 16` in `transmit_command_frame()`.

### Receiver Task

The `receiver_task` continuously monitors the data line for incoming Manchester-encoded frames:
- Polls GPIO every 1ms
- Detects edges and measures pulse durations
- Decodes timing into bits and assembles bytes
- Validates checksums before processing

### Transmitter

The `transmit_command_frame` function:
- Switches GPIO to output mode
- Generates Manchester-encoded timing
- Repeats frame 8 times as per protocol
- Returns to input mode for receiving

### Timing Accuracy

The ESP32-C6 uses `esp_rom_delay_us()` for microsecond precision timing required by the Manchester protocol. This is critical for reliable communication.

## 🔮 Future Enhancements

The driver is designed for extensibility. Potential enhancements based on external reference implementations:

### Already Recognized (Not Decoded)

The following frame types are detected and logged but not fully implemented:

1. **Clock Synchronization (0xD1)**
   - Time/date data for display and scheduling
   - Bytes 1-6 typically contain: hour, minute, second, day, month, year

2. **Configuration Frames (0x82)**
   - Heat pump configuration parameters
   - Defrost settings, temperature limits, operating modes

3. **Condition Frames (0x83, 0x84)**
   - **Type 1 (0x83)**: Additional temperature sensors
   - **Type 2 (0x84)**: Flow meter status, defrost cycles, error codes
   - Can be 9-byte or 12-byte format

### Implementation Guide

To add support for extended frame types:

```c
// In process_received_frame(), extend the switch statement:

case 0xD1:  // Clock frame
    uint8_t hour = frame[1];
    uint8_t minute = frame[2];
    uint8_t second = frame[3];
    // Update system time or display
    break;

case 0x84:  // Condition frame
    if (size == 9) {
        // Short frame: flow meter data
        bool flow_meter_active = (frame[2] & 0x10) != 0;
    } else {
        // Long frame: extended diagnostics
        uint8_t defrost_status = frame[4];
    }
    break;
```

### Benefits of Full Protocol

Implementing the complete protocol enables:
- ⏰ **Time synchronization** with heat pump controller
- 📊 **Advanced diagnostics** (defrost cycles, error codes, sensor health)
- 🌊 **Flow metering** if supported by hardware
- 🔧 **Remote configuration** of heat pump parameters
- 📈 **Energy monitoring** and efficiency tracking

**Reference**: See `sle118/hayward_pool_heater` GitHub repository for full frame definitions and decoding logic.

## 📜 License

This project is provided as-is for personal and educational use.

## 🙏 Credits

- Based on Arduino MQTT implementation by original author
- ESP-IDF and Zigbee stack by Espressif Systems

## 🔗 Related Projects

- **acw02_zb** - Toshiba HVAC controller (similar UART protocol)
- **caelum-weatherstation** - Weather station with sleep mode
- **Aeris_zb** - Air quality sensor with multiple endpoints

## 📞 Support

For issues and questions:
1. Check debug logs with `idf.py monitor`
2. Verify PC1001 protocol timing with logic analyzer
3. Test with original Arduino code first to confirm PC1001 operation
4. Review Zigbee2MQTT or ZHA logs for integration issues

---

**Version**: 1.0.0  
**Target**: ESP32-C6  
**Framework**: ESP-IDF v5.5.1+  
**Protocol**: Manchester-encoded bidirectional serial
