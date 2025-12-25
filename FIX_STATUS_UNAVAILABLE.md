# Fix: "No PC1001 status available yet" Issue

## Problem
When sending Zigbee commands to the PC1001 device, you were seeing:
```
W (xxxxx) PC1001_ZB: No PC1001 status available yet
```

Commands were being rejected because the device hadn't received any status frames from the heat pump yet.

## Root Cause
The code required a valid heat pump status (frame type 0x81) before allowing any commands. This created a "chicken and egg" problem:
- If the heat pump is off or not broadcasting frames → no status received
- Without status → commands are rejected
- Without commands → can't turn the heat pump on

## Solution Applied

### 1. Allow Commands Without Prior Status
- **File**: `esp_zb_pc1001.c` line ~340
- Changed: Now uses safe default values (26°C, heat mode, off) when status isn't available
- Commands can now be sent immediately after device boot

### 2. Removed Command Blocking
- **File**: `pc1001_driver.c` `pc1001_send_command()` 
- Changed: Removed the `return ESP_ERR_INVALID_STATE` that blocked commands
- Now shows warning but sends command anyway: `"No heat pump status received yet - sending blind command"`

### 3. Enhanced Diagnostics
Added better logging to help debug frame reception:
- GPIO initial level check at startup
- Frame start detection with pulse duration
- Frame type and validity logging at INFO level (not DEBUG)
- Startup message: "Waiting for heat pump status frames (type 0x81)..."

## What to Check

### After Flashing
Look for these log messages:

1. **Initialization**:
   ```
   I (xxx) PC1001_DRV: GPIO2 initial level: 1 (expect 1/HIGH when idle)
   I (xxx) PC1001_DRV: PC1001 driver initialized on GPIO2 - listening for frames
   I (xxx) PC1001_DRV: Waiting for heat pump status frames (type 0x81)...
   ```

2. **If receiving frames**:
   ```
   I (xxx) PC1001_DRV: Frame start detected (pulse: 25ms)
   I (xxx) PC1001_DRV: Valid 12-byte frame type 0x81
   I (xxx) PC1001_DRV: Status: 26.0°C, OFF, heat
   ```

3. **When sending commands** (before receiving frames):
   ```
   W (xxx) PC1001_ZB: No PC1001 status available yet - using defaults
   W (xxx) PC1001_DRV: No heat pump status received yet - sending blind command
   I (xxx) PC1001_DRV: Sending command: 28.0°C, ON, heat
   ```

## Hardware Checks if No Frames Received

If you still see NO frame reception logs:

1. **GPIO Level Check**: Initial level should be 1 (HIGH)
   - If stuck at 0 → wiring issue or level shifter problem

2. **Is Heat Pump Powered?**: The PC1001 only broadcasts when powered
   - If OFF → you won't see frames (this is normal)
   - Try: turn heat pump ON with physical button first

3. **Wiring Verification**:
   - GPIO2 → BSS138 LV (3.3V side)
   - BSS138 HV (5V side) → PC1001 data line (single wire)
   - Common ground between ESP and PC1001
   - Pull-ups: 10K on both LV and HV sides

4. **Protocol Timing**: PC1001 uses Manchester encoding
   - Frame start: ~25ms pulse (22-28ms accepted)
   - Bit 0: ~5ms pulse, Bit 1: ~15ms pulse
   - If wrong protocol → no frames will be decoded

## Next Steps

1. Flash the updated firmware
2. Monitor the logs for frame reception
3. Try sending a Zigbee command:
   - It should now work even without prior status
   - Watch for "sending blind command" message
4. If heat pump responds, you should start seeing status frames
5. Once status frames are received, commands will use actual current state

## Build & Flash

```bash
cd /home/fabian/Repositories/pc1001_zb
idf.py build
idf.py flash monitor
```

Press Ctrl+] to exit monitor.
