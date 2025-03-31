# SzögFM: Remote Controlled FM Receiver System

This project implements a distributed FM radio receiver system with remote volume control for festival applications. It consists of a central controller and multiple receiver nodes that can be placed throughout a festival area.

## System Overview

### Key Features
- Central control of multiple FM receivers
- Remote volume control via 433MHz wireless communication
- Power control for speakers via relay
- OLED display on each receiver node
- Manual controls on each node
- Web interface for central control
- Environmental monitoring (temperature/humidity) on selected nodes

### Hardware Components
- ESP32 microcontrollers for nodes and controller
- RDA5807M FM receiver modules
- EBYTE E49 433MHz LORA communication modules
- 0.96" OLED displays (I2C)
- Relays for speaker power control
- Optional DHT22 temperature/humidity sensors

## Project Structure

```
szogfm/
├── common/                 # Shared code between nodes and controller
│   ├── messages.h          # Message structures for communication
│   ├── NodeConfig.h/cpp    # Configuration storage
│   ├── communication/      # Communication modules
│   │   ├── ICommModule.h   # Interface for communication modules
│   │   └── EbyteCommModule.h/cpp # EBYTE implementation
│   ├── radio/              # FM radio modules
│   │   ├── IFmRadio.h      # Interface for FM radio modules
│   │   └── RDA5807Radio.h/cpp # RDA5807 implementation
│   ├── display/            # Display modules
│   │   ├── IDisplay.h      # Interface for display modules
│   │   └── OledDisplay.h/cpp # OLED implementation
│   └── input/              # Input handling
│       └── ButtonHandler.h/cpp # Button handling
├── node/                   # Node application
│   ├── NodeApplication.h/cpp # Node application
│   └── main.cpp            # Node main entry point
├── controller/             # Controller application
│   ├── ControllerApplication.h/cpp # Controller application
│   └── main.cpp            # Controller main entry point
└── platformio.ini          # PlatformIO configuration
```

## Building the Project

This project uses PlatformIO for building and uploading. To build the project:

1. Install PlatformIO (as an extension for VS Code or as a CLI tool)
2. Clone this repository
3. Open the project in PlatformIO

### Building the Node Application

```bash
pio run -e node
```

This will build the node application. To upload to an ESP32 device:

```bash
pio run -e node -t upload
```

### Building the Controller Application

```bash
pio run -e controller
```

This will build the controller application. To upload to an ESP32 device:

```bash
pio run -e controller -t upload
```

## Configuration

### Node Configuration

Nodes store their configuration in EEPROM, including:
- Node ID
- FM frequency
- Volume level
- Relay state
- Radio communication parameters

The first time a node is run, it will initialize with default configuration. You can change this configuration through the remote control or by using the button interface on the node.

### Controller Configuration

The controller establishes a WiFi access point with the following default settings:
- SSID: SzogFM_Controller
- Password: GombaSzog24
- IP Address: 192.168.4.1

You can access the web interface by connecting to this WiFi network and navigating to [http://192.168.4.1](http://192.168.4.1) in a web browser.

## Usage

### Node Operation

Once powered on, a node will:
1. Initialize all components
2. Connect to the RDA5807M FM receiver
3. Load its configuration
4. Display the current status on the OLED screen
5. Start listening for commands from the controller

Nodes have physical buttons for manual control:
- Volume up/down
- Frequency up/down

### Controller Operation

The controller will:
1. Initialize the communication module
2. Set up a WiFi access point
3. Start the web server
4. Begin polling for node status updates

The web interface allows you to:
- View the status of all nodes
- Control volume for individual nodes or all nodes
- Set FM frequency for individual nodes or all nodes
- Toggle relays on/off for individual nodes or all nodes
- Mute/unmute nodes
- Reset nodes

## Troubleshooting

### Common Issues

- **Node not appearing in controller**: Make sure the node is powered on and has a valid configuration with a unique node ID. Check the communication module configuration.
- **No FM reception**: Verify that the FM module is properly connected and that the antenna is attached. Check the set frequency.
- **Display not working**: Check I2C connections and power supply to the display.
- **Button controls not responding**: Verify the resistor network connections to the analog input pin.

## Future Enhancements

Planned features for future versions:
- Scheduled volume changes
- Zone-based control
- Enhanced environmental monitoring
- WiFi connectivity for nodes
- Firmware updates over the air
- Backup control systems
- Battery status monitoring

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgments

- EBYTE library by Kris Kasprzak
- RDA5807 library by PU2CLR
- Adafruit libraries for display