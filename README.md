# ESP32-C6 PC1001 Pool Heat Pump Controller - Zigbee

ESP32-C6 Zigbee router device for controlling Hayward/Majestic PC1001 pool heat pump controllers via Manchester-encoded serial protocol.

## 🌟 Features

- **Zigbee End Device** - Mains powered, non-sleepy device (powered off seasonally)
- **Thermostat Cluster** - Standard Zigbee thermostat with heating/cooling/auto modes
- **PC1001 Protocol** - Full bidirectional Manchester protocol implementation
- **Real-time Monitoring** - Water inlet/outlet temperature, programmed setpoint, mode
- **Home Assistant Ready** - Works with Zigbee2MQTT and ZHA
- **Reliable Operation** - Based on proven acw02_zb/caelum architecture

## 📋 Hardware Requirements

- **ESP32-C6** development board (e.g., ESP32-C6-DevKitC-1)
- **PC1001 Controller** - Hayward/Majestic heat pump with PC1001 control interface
- **Single wire connection** - GPIO2 to PC1001 data line (bidirectional)
- **Power supply** - 5V USB (device does not sleep)

## 🔌 Wiring

```
ESP32-C6          PC1001 Controller
--------          -----------------
GPIO2    <---->   DATA LINE
GND      <---->   GND
```

⚠️ **Important**: The PC1001 data line operates at **5V** logic levels. You may need a level shifter or ensure your ESP32-C6 GPIO is 5V tolerant. Alternatively, use a voltage divider or bidirectional level converter.

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

The device operates as a **Zigbee End Device** (mains powered, rx_on_when_idle=true) and exposes a **Thermostat cluster**:

**Note**: Configured as an End Device rather than Router because the device is powered off during winter. End Devices can leave/rejoin the network without disrupting routing for other devices.

### Endpoint 1 - Thermostat
- **Cluster**: Thermostat (0x0201)
- **Attributes**:
  - `local_temperature` - Water inlet temperature (read-only)
  - `occupied_cooling_setpoint` - Target cooling temperature (read/write, 15-32°C)
  - `occupied_heating_setpoint` - Target heating temperature (read/write, 15-32°C)
  - `system_mode` - Operating mode (off/heat/cool/auto)

### Supported Modes

| Zigbee Mode | PC1001 Mode | Description |
|-------------|-------------|-------------|
| `off` (0x00) | Power OFF | Heat pump disabled |
| `auto` (0x01) | AUTO | Automatic heating/cooling |
| `cool` (0x03) | COOL | Cooling only |
| `heat` (0x04) | HEAT | Heating only |

## 🏠 Home Assistant Integration

### Zigbee2MQTT

The device will appear as a thermostat with the following controls:

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

### ZHA (Zigbee Home Automation)

The device will be auto-discovered as a thermostat and will expose:
- Current water temperature
- Target temperature (0.5°C steps)
- Operating mode selector
- Power on/off

## 📊 PC1001 Protocol Details

### Frame Structure

The PC1001 uses Manchester encoding over a single wire:
- **Timing-based** protocol (not UART)
- **Start bit**: 9ms LOW + 5ms HIGH
- **Binary 0**: 1ms LOW + 1ms HIGH
- **Binary 1**: 1ms LOW + 3ms HIGH
- **Frame size**: 12 bytes
- **Repetition**: Each command sent 8 times
- **Checksum**: Byte 11 contains checksum of bytes 0-10

### Received Frames

The PC1001 broadcasts three types of status frames:

| Frame Type | Byte 0 | Contains |
|------------|--------|----------|
| Temp OUT | 0x4B | Water outlet temperature |
| Temp IN | 0x8B | Water inlet temperature |
| Status | 0x81 | Programmed temp, mode, power state |

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

## 📜 License

This project is provided as-is for personal and educational use.

## 🙏 Credits

- Based on Arduino MQTT implementation by original author
- Architecture inspired by acw02_zb and caelum-weatherstation projects
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
