# ESP32 MCP23017 LED Blink

Blinks an LED connected to GPA0 of MCP23017 I2C GPIO expander.

## Hardware Connections

### MCP23017 to ESP32
```
MCP23017 Pin    →  ESP32/Power
─────────────────────────────────
VCC             →  3.3V
GND             →  GND
SDA             →  GPIO 23
SCL             →  GPIO 22
```

### LED Connection (Sink Mode)
```
3.3V → 330Ω Resistor → LED Anode (+) → LED Cathode (-) → GPA0 on MCP23017
```

**LED Polarity:**
- Anode (+): Longer leg
- Cathode (-): Shorter leg, flat edge on plastic

## I2C Address
- Default: `0x20` (A0, A1, A2 = GND)

## Code Logic (Sink Mode)
- `0` (LOW) = LED ON (sinks current to ground)
- `1` (HIGH) = LED OFF (high impedance)

## Build & Flash
```bash
idf.py build
idf.py flash monitor
```

## Expected Output
```
Device found at 0x20
GPA0 configured as output
LED ON
LED OFF
LED ON
...
```