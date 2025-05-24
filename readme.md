# SzögFM: Distributed FM Receiver System with Remote Volume Control

## Table of Contents
- [Overview](#overview)
- [System Architecture](#system-architecture)
- [Hardware Components](#hardware-components)
- [Software Structure](#software-structure)
- [Installation and Setup](#installation-and-setup)
- [Configuration](#configuration)
- [Usage](#usage)
- [Development](#development)
- [Troubleshooting](#troubleshooting)
- [Future Enhancements](#future-enhancements)
- [Bill of Materials](#bill-of-materials)

## Overview

SzögFM is a distributed FM receiver system designed for large outdoor events and festivals. The system provides centralized audio distribution via FM broadcast with individual remote volume control for each speaker location.

### Key Features

- **Centralized FM Audio Distribution**: 5W FM transmitter broadcasting at 88.5MHz with ~3km range
- **Remote Volume Control**: Individual volume control for up to 20 receiver nodes
- **433MHz Control Network**: Separate LORA-based communication for control commands
- **Local Manual Controls**: Physical buttons on each node for local adjustments
- **OLED Display**: Visual feedback showing frequency, volume, signal strength, and status
- **Environmental Monitoring**: Temperature and humidity sensors on selected nodes
- **Power Management**: Remote relay control for speaker power switching
- **Robust Design**: Industrial-grade components for outdoor festival environments

### Problem Solved

Traditional audio distribution at large festivals requires expensive cable infrastructure. SzögFM solves this by:
1. Using FM radio for audio distribution (no audio cables needed)
2. Providing individual volume control via 433MHz wireless commands
3. Enabling centralized control from a web interface
4. Supporting emergency shutdown and zone-based control

## System Architecture

The system consists of three main components:

### 1. Central FM Transmitter
- **Location**: High position (building roof) in central festival area
- **Power**: 5W transmitter with custom ground plane antenna
- **Range**: ~3km radius coverage
- **Frequency**: 88.5MHz (configurable)
- **Audio Input**: Line level from mixing console

### 2. Central Controller
- **Hardware**: ESP32 with EBYTE E49 433MHz LORA module
- **Function**: Receives commands via USB/web interface and relays to nodes
- **Communication**: 433MHz LORA network with ~1km range
- **Interface**: Web-based control panel (planned) or command-line interface

### 3. Receiver Nodes (up to 20 units)
- **Hardware**: ESP32 + FM receiver + 433MHz LORA + OLED display
- **Function**: Receive FM audio, process volume commands, drive speakers
- **Display**: 128x64 OLED showing frequency, volume, signal strength, status
- **Controls**: Local buttons for manual frequency/volume adjustment
- **Output**: Line-level audio to powered speakers or amplifiers

## Hardware Components

### Receiver Node Specifications

#### Microcontroller
- **Model**: ESP32 WROOM module
- **Operating Voltage**: 3.3V (regulated from 5V)
- **Features**: WiFi/Bluetooth (disabled for power saving)
- **Interfaces**: I2C, UART, SPI, GPIO

#### Power Supply
- **Input**: 12V DC via 5.5mm barrel jack
- **Regulation**: MP2307-based buck converter (12V→5V→3.3V)
- **Consumption**: ~3W maximum per node
- **Source**: Mains power at speaker locations

#### FM Receiver
- **Module**: RDA5807M-based I2C FM tuner
- **Frequency Range**: 87.5-108.0 MHz
- **Interface**: I2C bus
- **Output**: Line-level stereo audio
- **Features**: RSSI monitoring, stereo detection

#### 433MHz Communication
- **Module**: EBYTE E49-400T20D LORA module
- **Frequency**: 433MHz ISM band
- **Power**: 20dBm (100mW) configurable
- **Range**: ~1km in real conditions
- **Interface**: UART
- **Reliability**: ~90% message success rate

#### Display
- **Type**: 0.96" SSD1306 OLED (128x64 monochrome)
- **Interface**: I2C (address 0x3C)
- **Information Displayed**:
    - FM frequency (MHz)
    - Volume level with progress bar
    - FM signal strength (RSSI)
    - Connection status with signal bars
    - Node ID and relay state
    - Temperature/humidity (if sensors present)

#### User Interface
- **Buttons**: 4 buttons on single analog input via resistor network
    - Frequency Up (analog range 1600-2200)
    - Frequency Down (analog range 2300-2900)
    - Volume Up (analog range 3700-4095)
    - Volume Down (analog range 1000-1400)
- **Visual Feedback**: OLED display with refresh indicator

#### Additional Components
- **Relay**: 12V relay for speaker power control (230V switching)
- **Sensors**: DHT22 temperature/humidity sensor (5 nodes only)
- **Audio Amplifier**: LM4881 (bypassable for line output)
- **Enclosure**: Custom 3D-printed weatherproof housing

### PCB Specifications
- **Dimensions**: 100mm × 100mm
- **Layers**: 2-layer PCB from JLC-PCB
- **Connectors**: Header-based modules for easy servicing
- **Mounting**: Designed for 3D-printed enclosure

## Software Structure

### Architecture
The software follows a modular design with clear separation of concerns:

```
szogfm/
├── common/                 # Shared components
│   ├── communication/     # 433MHz LORA communication
│   ├── display/          # Display interfaces and implementations
│   ├── input/            # Button handling
│   ├── radio/            # FM radio control
│   ├── messages.h        # Message protocol definitions
│   └── NodeConfig.h      # Configuration management
├── node/                 # Receiver node application
│   ├── NodeApplication.h # Main node logic
│   ├── NodeApplication.cpp
│   └── main.cpp         # Arduino entry point
└── controller/          # Central controller (future)
    └── ControllerApplication.h
```

### Key Classes

#### NodeApplication
- **Purpose**: Main application logic for receiver nodes
- **Responsibilities**:
    - Initialize and coordinate all hardware components
    - Process incoming 433MHz commands
    - Handle local button presses
    - Update display with current status
    - Send status messages and acknowledgments

#### SSD1306Display
- **Purpose**: OLED display management
- **Features**:
    - Real-time FM signal strength display
    - Volume visualization with progress bar
    - Connection status with signal bars
    - Automatic refresh and caching
    - Error message display

#### EbyteCommModule
- **Purpose**: 433MHz LORA communication
- **Features**:
    - Message sending/receiving with error handling
    - Configurable parameters (channel, address, power)
    - Debug logging and diagnostics
    - Automatic retry mechanisms

#### RDA5807Radio
- **Purpose**: FM radio control
- **Features**:
    - Frequency tuning and volume control
    - RSSI (signal strength) monitoring
    - Stereo detection
    - Mute functionality

### Communication Protocol

#### Message Types
- **COMMAND**: Volume, frequency, relay, mute commands
- **STATUS_REQUEST**: Request for node status
- **STATUS_RESPONSE**: Node status with all parameters
- **ACK**: Command acknowledgment with success/error codes
- **ERROR**: Error notifications

#### Addressing
- **Node IDs**: 1-20 for individual nodes, 0 for broadcast
- **Sequence Numbers**: For message tracking and duplicate detection
- **Checksums**: Header validation for data integrity

## Installation and Setup

### Hardware Installation

1. **Mount receiver nodes** at speaker locations
2. **Connect 12V power supply** via barrel jack
3. **Connect audio output** to powered speakers or amplifiers
4. **Connect relay** to speaker power control (optional)
5. **Position antennas** for optimal FM and 433MHz reception
6. **Install in weatherproof enclosure**

### Software Installation

#### Prerequisites
- **PlatformIO IDE** or Arduino IDE with ESP32 support
- **USB cable** for programming ESP32 modules
- **Serial monitor** for debugging and configuration

#### Library Dependencies
```ini
lib_deps =
    EBYTE=https://github.com/KrisKasprzak/EBYTE.git
    adafruit/Adafruit GFX Library @ ^1.11.5
    adafruit/Adafruit SSD1306 @ ^2.5.7
    adafruit/DHT sensor library @ ^1.4.4
    adafruit/Adafruit Unified Sensor @ ^1.1.9
    adafruit/Adafruit BusIO@^1.17.0
    ArduinoJson @ ^6.21.2
    pu2clr/PU2CLR RDA5807 @ ^1.1.9
```

#### Build and Upload
```bash
# For PlatformIO
pio run -e node --target upload

# Or use PlatformIO IDE upload button
```

### Initial Configuration

#### Node Configuration
Each node needs unique configuration:

```cpp
// Set in NodeConfig or via serial commands
nodeId = 1-20;          // Unique node identifier
radioChannel = 0x1A;    // 433MHz channel
radioAddress = 0x1234;  // Network address
fmFrequency = 8850;     // 88.5 MHz
volume = 8;             // Initial volume (0-15)
```

#### Pin Configuration
Default pin assignments (modify in NodeApplication constructor):
```cpp
_pinUserButton = 34;      // Analog input for buttons
_pinRelayControl = 27;    // Relay control output
_pinSDASensor = 21;       // I2C SDA (FM radio + display)
_pinSCLSensor = 22;       // I2C SCL (FM radio + display)
_pinM0 = 4;              // EBYTE M0 control
_pinM1 = 32;             // EBYTE M1 control
_pinAUX = 33;            // EBYTE AUX status
```

## Configuration

### Node Settings

#### Basic Parameters
- **Node ID**: Unique identifier (1-20)
- **FM Frequency**: Default 88.5 MHz (8850 in 0.1 MHz units)
- **Volume**: Default level 8 (range 0-15)
- **Radio Channel**: 433MHz channel (default 0x1A)
- **Radio Address**: Network address (default 0x1234)

#### Communication Settings
- **Air Data Rate**: 2.4k baud (for range vs. speed balance)
- **UART Baud Rate**: 9600 baud (for module communication)
- **Transmission Power**: 20dBm (maximum range)

#### Button Ranges
Adjust these values based on your resistor network:
```cpp
FREQUENCY_UP:   1600-2200  // Analog range
FREQUENCY_DOWN: 2300-2900
VOLUME_UP:      3700-4095
VOLUME_DOWN:    1000-1400
```

### Sensor Configuration
Enable DHT22 sensors by uncommenting in code:
```cpp
#define ENABLE_DHT_SENSOR
```

## Usage

### Normal Operation

1. **Power on** all receiver nodes and central controller
2. **Verify display** shows correct frequency and connection status
3. **Start FM transmitter** on configured frequency (88.5 MHz)
4. **Control remotely** via central controller or web interface
5. **Use local buttons** for manual adjustments if needed

### Local Controls

#### Button Functions
- **Frequency Up**: Increase FM frequency by 0.1 MHz
- **Frequency Down**: Decrease FM frequency by 0.1 MHz
- **Volume Up**: Increase volume by 1 level (max 15)
- **Volume Down**: Decrease volume by 1 level (min 0)

#### Display Information
- **Top Line**: FM frequency and signal strength (RSSI)
- **Second Line**: Volume level with progress bar
- **Third Line**: Connection status with signal bars
- **Bottom Line**: Node ID and relay state
- **Additional**: Temperature/humidity if sensors enabled

### Remote Commands

#### Available Commands
- **SET_VOLUME**: Set volume level (0-15)
- **SET_FREQUENCY**: Set FM frequency (8750-10800, 0.1 MHz units)
- **TOGGLE_RELAY**: Turn speaker power on/off
- **MUTE/UNMUTE**: Mute or unmute audio
- **RESET**: Restart the node
- **GET_STATUS**: Request current status

#### Command Format (via serial/web interface)
```
Command examples (implementation specific):
volume <nodeId> <level>     # Set volume
frequency <nodeId> <freq>   # Set frequency
relay <nodeId> <on/off>     # Control relay
mute <nodeId>              # Mute node
status <nodeId>            # Get status
```

### Status Monitoring

#### Serial Monitor Output
The system provides detailed logging:
- **Button presses**: Shows which button and resulting action
- **Command processing**: Detailed command execution logs
- **Radio status**: RSSI, stereo detection, tuning status
- **Communication**: Message sending/receiving with hex dumps
- **Error handling**: Detailed error descriptions

#### Display Status
- **Refresh indicator**: Small rotating icon shows active updates
- **Signal strength**: Both 433MHz (bars) and FM (RSSI number)
- **Connection timeout**: Display shows disconnected after 60 seconds
- **Error display**: Full-screen error messages when problems occur

## Development

### Building the Project

#### Environment Setup
1. Install **PlatformIO** IDE or extension
2. Clone the repository
3. Open project in PlatformIO
4. Install dependencies automatically

#### Compilation
```bash
# Build node firmware
pio run -e node

# Build controller firmware (future)
pio run -e controller

# Clean build
pio run --target clean
```

#### Upload and Monitor
```bash
# Upload to ESP32
pio run -e node --target upload

# Monitor serial output
pio device monitor --baud 115200
```

### Code Structure

#### Adding New Features
1. **Display additions**: Extend SSD1306Display class
2. **Commands**: Add to Command enum and message handlers
3. **Sensors**: Implement in sensor initialization functions
4. **Communication**: Extend message protocol

#### Debug Levels
```cpp
// Set debug levels for different components
_commModule->setDebugLevel(2);  // 0=none, 1=basic, 2=verbose
```

### Testing

#### Hardware Testing
1. **I2C Bus**: Scan for devices on startup
2. **Button response**: Monitor analog values and press detection
3. **FM reception**: Check RSSI values and stereo detection
4. **433MHz communication**: Verify message success rate
5. **Display**: Test pattern on startup

#### Communication Testing
1. **Message delivery**: ~90% success rate expected
2. **Range testing**: Verify 1km range in open areas
3. **Interference**: Test with multiple nodes active
4. **Recovery**: Test automatic reconnection after failures

## Troubleshooting

### Common Issues

#### No Display Output
- **Check I2C connections**: SDA (pin 21), SCL (pin 22)
- **Verify display address**: Default 0x3C (some use 0x3D)
- **Check power supply**: Ensure 3.3V stable
- **I2C scan**: Look for device detection in startup logs

#### No FM Audio
- **Check antenna connection**: Proper FM antenna attached
- **Verify frequency**: Correct tuning to transmitter
- **Signal strength**: RSSI should be >30 for good reception
- **Audio connections**: Line-level output properly connected

#### Button Not Working
- **Check analog values**: Monitor serial output for button presses
- **Adjust ranges**: Modify analog ranges in button setup
- **Wiring**: Verify resistor network and connections
- **Debouncing**: Check for multiple rapid presses

#### 433MHz Communication Failure
- **AUX pin status**: Should be HIGH when ready
- **Module configuration**: Verify channel and address match
- **Antenna**: Check 433MHz antenna connection
- **Range**: Test at closer distance first
- **Interference**: Check for other 433MHz devices

#### Power Issues
- **Voltage levels**: Check 12V input, 5V and 3.3V regulation
- **Current draw**: Ensure power supply can handle load
- **Brown-out**: Check for voltage drops during transmission
- **Ground loops**: Ensure proper grounding

### Diagnostic Procedures

#### Serial Monitor Diagnostics
1. **Boot sequence**: Check initialization of all components
2. **I2C scan**: Verify all devices detected
3. **Radio status**: Monitor RSSI and tuning status
4. **Communication**: Watch for message send/receive
5. **Button presses**: Verify analog readings and actions

#### Module Diagnostics
```cpp
// Communication module diagnostics
_commModule->performDiagnostics();
_commModule->printParameters();

// Radio module status
int rssi = _radio->getRssi();
bool stereo = _radio->isStereo();
bool tuned = _radio->isTunedToStation();
```

#### Display Test Pattern
The system shows test patterns on startup to verify display functionality.

### Error Codes
- **Radio initialization failed**: Check I2C connections
- **Communication module failed**: Check UART and control pins
- **Display initialization failed**: Check I2C address and connections
- **Configuration load failed**: EEPROM/flash issues, will use defaults

## Future Enhancements

### Planned Features (Roadmap)

#### Phase 1: Core Improvements
- **Web interface**: Complete web-based control panel
- **WiFi control**: Direct node control via smartphone
- **Improved reliability**: Better error recovery and retry mechanisms
- **Configuration interface**: Web-based node configuration

#### Phase 2: Advanced Features
- **Zone control**: Group nodes for synchronized control
- **Scheduled operations**: Time-based volume changes
- **Audio streaming**: WiFi backup for FM distribution
- **Battery backup**: UPS functionality for critical nodes

#### Phase 3: Integration
- **Mesh networking**: Extended range via node relaying
- **Integration APIs**: External system integration
- **Mobile app**: Dedicated smartphone control application
- **Analytics**: Usage monitoring and reporting

### Security Considerations

#### Current Status
- **No authentication**: Control access not restricted
- **No encryption**: 433MHz communication unencrypted
- **Physical security**: Depends on enclosure and placement

#### Planned Improvements
- **Basic authentication**: Password-protected control interface
- **Command verification**: Checksums and sequence numbers
- **Message encryption**: Basic encryption for control commands
- **Access logging**: Track all system changes

### Performance Optimizations
- **Message reliability**: Improved error correction
- **Power efficiency**: Sleep modes and power management
- **Range extension**: Mesh networking capabilities
- **Display optimization**: Faster updates and less flicker

## Bill of Materials

### Per Node Cost Breakdown
| Component | Quantity | Unit Cost | Total |
|-----------|----------|-----------|-------|
| ESP32 WROOM module | 1 | $3.50 | $3.50 |
| RDA5807M FM receiver | 1 | $2.00 | $2.00 |
| EBYTE E49 LORA module | 1 | $4.50 | $4.50 |
| SSD1306 OLED display | 1 | $2.50 | $2.50 |
| Custom PCB (JLC-PCB) | 1 | $6.15 | $6.15 |
| Electronic components | - | $3.00 | $3.00 |
| 3D printed enclosure | 1 | $3.00 | $3.00 |
| Antennas (FM + 433MHz) | 2 | $2.00 | $4.00 |
| **Total per node** | | | **$28.65** |

### Project Total (20 nodes)
| Item | Cost |
|------|------|
| 20 × Receiver nodes | $573.00 |
| Central controller | $50.00 |
| FM transmitter + antenna | $200.00 |
| Development and testing | $100.00 |
| **Project Total** | **$923.00** |

### Additional Costs
- **Power supplies**: $10-20 per location (may be existing)
- **Cables and connectors**: $5-10 per node
- **Weatherproofing**: $10-15 per node
- **Installation labor**: Variable

### Bulk Pricing Notes
- PCB costs decrease significantly with larger quantities
- Electronic components benefit from volume discounts
- 3D printing costs can be reduced with own equipment

---

## License and Credits

### Project Information
- **Name**: SzögFM Distributed FM Receiver System
- **Purpose**: Festival audio distribution and control
- **Author**: Csányi Balázs
- **Status**: Work in progress, functional prototype

### Libraries Used
- **EBYTE**: Arduino library for EBYTE modules
- **Adafruit GFX/SSD1306**: Display libraries
- **PU2CLR RDA5807**: FM radio control library
- **DHT sensor library**: Temperature/humidity sensors
- **Arduino JSON**: Configuration management

### Hardware References
- **ESP32**: Espressif Systems ESP32 microcontroller
- **RDA5807M**: RDA Microelectronics FM tuner chip
- **EBYTE E49**: 433MHz LORA communication module
- **SSD1306**: Solomon Systech OLED display controller

---

*This project is designed for educational and festival use. Ensure compliance with local radio regulations for both FM transmission and 433MHz operation.*