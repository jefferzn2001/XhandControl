# XHandControl

Comprehensive Python wrapper for controlling XHAND robotic hands via RS-485 and EtherCAT.  
Tested on **Ubuntu 22.04** and **Windows 10/11** with **Python 3.10+**.

---



- Clean API in `xhandlib/xhand.py` (removed `exam_` prefixes). Use:
  - `open_device(...)`, `close()`
  - `read_state(hand_id, force_update=True)`
  - `get_sdk_version()`, `read_version(hand_id)`, `get_info(hand_id)`, `get_hand_type(hand_id)`, `get_serial_number(hand_id)`
  - `send_command()` to send the prebuilt `_hand_command`, or `set_hand_mode(...)` to set modes
- Unified scripts and naming:
  - `demo.py` (mode=3) dual-hand motions
  - `encoder.py` (mode=0) live joint angles GUI (degrees)
  - `tactile.py` (mode=0) live tactile GUI (5 fingers per hand)
- Standard mapping used across scripts:
  - Left hand: `/dev/ttyUSB1`, ID=2
  - Right hand: `/dev/ttyUSB0`, ID=1
- After installing the SDK wheel, you can delete the `xhand_control_sdk_py/` folder.

---

## ⚙️ Installation

### Prerequisites
- Python 3.10 or higher
- USB-to-RS485 adapters (for multiple hands)
- XHAND robotic hands

### Setup Environment

```bash
git clone git@github.com:jefferzn2001/XhandControl.git
cd XhandControl

# Create conda environment (recommended)
conda create -n xhand python=3.10 -y
conda activate xhand

# OR create virtual environment
python -m venv xhand_env
source xhand_env/bin/activate  # Linux/Mac
# xhand_env\Scripts\activate    # Windows

# Install SDK wheel (choose matching Python version)
cd xhand_control_sdk_py
pip install ./xhand_controller-1.1.7-cp310-cp310-linux_x86_64.whl  # Python 3.10 on Linux x86_64
# pip install ./xhand_controller-1.1.7-cp312-cp312-linux_x86_64.whl  # Python 3.12
cd ..

# After successful install, the .whl is not required to run. You can delete it
# and optionally remove the xhand_control_sdk_py/ folder if you want a lean repo.

# Install additional dependencies
pip install numpy matplotlib

# (Optional) Add current directory to PYTHONPATH if running from cloned repo
export PYTHONPATH=$(pwd):$PYTHONPATH
```

---

## 🔧 Hardware Setup for Dual Hands

### USB Device Configuration

When using **two USB-to-RS485 adapters** for dual hand control:

1. **Connect each hand to separate USB ports**:
   ```
   Left Hand  → USB Port 2 → /dev/ttyUSB1 (Linux)
   Right Hand → USB Port 1 → /dev/ttyUSB0 (Linux)
   ```

2. **Identify your USB devices**:
   ```bash
   # Linux - List all USB serial devices
   ls /dev/ttyUSB*
   
   # Check device details
   dmesg | grep ttyUSB
   
   # Windows - Check Device Manager or use PowerShell
   Get-WmiObject -Class Win32_SerialPort | Select-Object Name, DeviceID
   ```

3. **Set unique hand IDs** (critical for dual control):
   ```bash
   # Connect ONLY the left hand first (ID=2)
   python -m xhandlib.ID --port /dev/ttyUSB1 --target 2
   # Power cycle the left hand
   
   # Connect ONLY the right hand (ID=1)
   python -m xhandlib.ID --port /dev/ttyUSB0 --target 1  
   # Power cycle the right hand
   ```

### Device ID Management

The `xhandlib/ID.py` tool helps manage hand IDs:

```bash
# List all hands on a specific port
python -m xhandlib.ID --port /dev/ttyUSB0 --list

# Auto-assign ID to single connected hand
python -m xhandlib.ID --port /dev/ttyUSB0 --target 1

# Manual ID change (if you know current ID)
python -m xhandlib.ID --port /dev/ttyUSB0 --old 0 --new 1

# For Windows, use COM ports:
python -m xhandlib.ID --port COM3 --target 1
```

**Important**: Always power-cycle the hand after changing its ID to persist the change.

---

## 🚀 Usage Examples

### Basic Single Hand Control

```python
from xhandlib.xhand import XHandControl
import time

# Initialize hand controller
hand = XHandControl(hand_id=1, position=0.1, mode=3)

# Setup device connection
device_config = {
    "protocol": "RS485",
    "serial_port": "/dev/ttyUSB0",  # or "COM3" on Windows
    "baud_rate": 3000000
}

# Open device
if hand.open_device(device_config):
    print("✅ Connected successfully!")
    
    # Get device information
    print("SDK:", hand.get_sdk_version())
    ok, ver = hand.read_version(hand._hand_id)
    info = hand.get_info(hand._hand_id)
    
    # Read sensor state
    hand.read_state(hand._hand_id, force_update=True)
    
    # Send preset actions (BE CAREFUL - MOVES THE HAND!)
    # hand.send_command()  # Send current command (dangerous)
    
    # Close connection
    hand.close()
else:
    print("❌ Failed to connect")
```

### Dual Hand Control Setup

```python
from xhandlib.xhand import XHandControl
import time

# Initialize both hands
LEFT_SERIAL_PORT = "/dev/ttyUSB1"  # Left
RIGHT_SERIAL_PORT = "/dev/ttyUSB0" # Right
LEFT_ID = 2
RIGHT_ID = 1

left_hand = XHandControl(hand_id=LEFT_ID, position=0.1, mode=3)
right_hand = XHandControl(hand_id=RIGHT_ID, position=0.1, mode=3)

# Device configurations
left_config = {"protocol": "RS485", "serial_port": LEFT_SERIAL_PORT, "baud_rate": 3000000}
right_config = {"protocol": "RS485", "serial_port": RIGHT_SERIAL_PORT, "baud_rate": 3000000}

# Connect both hands
left_connected = left_hand.open_device(left_config)
right_connected = right_hand.open_device(right_config)

if left_connected and right_connected:
    print("✅ Both hands connected!")
    
    # Synchronize actions
    # WARNING: This will move both hands simultaneously
    # left_hand.send_command()
    # right_hand.send_command()
    
    # Close both connections
    left_hand.close()
    right_hand.close()
```

<!-- EtherCAT configuration omitted for brevity; focus of this repo is RS-485 usage. -->

### Advanced Control with Preset Actions

```python
from xhandlib.xhand import XHandControl
import time
import math

hand = XHandControl(hand_id=1)
device_config = {
    "protocol": "RS485",
    "serial_port": "/dev/ttyUSB0",
    "baud_rate": 3000000
}

if hand.open_device(device_config):
    # Set hand to position control mode
    hand.set_hand_mode(mode=3)  # 0=powerless, 3=position, 5=powerful
    
    # Define preset actions (angles in degrees)
    actions = {
        'fist': [11.85, 74.58, 40, -3.08, 106.02, 110, 109.75, 107.56, 107.66, 110, 109.1, 109.15],
        'palm': [0, 80.66, 33.2, 0.00, 5.11, 0, 6.53, 0, 6.76, 4.41, 10.13, 0],
        'v': [38.32, 90, 52.08, 6.21, 2.6, 0, 2.1, 0, 110, 110, 110, 109.23],
        'ok': [45.88, 41.54, 67.35, 2.22, 80.45, 70.82, 31.37, 10.39, 13.69, 16.88, 1.39, 10.55]
    }
    
    # Execute action sequence
    for action_name, angles in actions.items():
        print(f"Executing: {action_name}")
        
        # Convert degrees to radians and set positions
        for i in range(12):
            hand._hand_command.finger_command[i].position = angles[i] * math.pi / 180
        
        # Send command (BE CAREFUL!)
        # hand.send_command()
        time.sleep(2)  # Hold pose for 2 seconds
    
    hand.close()
```

---

## 📋 API Reference

### XHandControl Class

The main control class located in `xhandlib/xhand.py`.

#### Constructor
```python
XHandControl(hand_id=0, position=0.1, mode=3)
```
- `hand_id` (int): Hand identifier (0-125)
- `position` (float): Default finger position (0.0-1.0)
- `mode` (int): Control mode (0=powerless, 3=position, 5=powerful)

#### Device Management Methods

| Method | Description |
|--------|-------------|
| `enumerate_devices(protocol)` | List available ports |
| `open_device(device_identifier)` | Open device connection |
| `close()` | Close device connection |
| `list_ids()` | List connected hand IDs |

#### Information Methods

| Method | Description |
|--------|-------------|
| `get_sdk_version()` | Get software SDK version |
| `read_version(hand_id)` | Get hardware SDK version |
| `get_info(hand_id)` | Get device information |
| `get_hand_type(hand_id)` | Get hand type (left/right) |
| `get_serial_number(hand_id)` | Get serial number |
| `get_hand_name(hand_id)` | Get hand name |
| `set_hand_name(hand_id, new_name)` | Set hand name |

#### Control Methods

| Method | Description | ⚠️ Warning |
|--------|-------------|-----------|
| `send_command()` | Send prebuilt `_hand_command` | **MOVES HAND** |
| `send_command(hand_id, HandCommand_t)` | Send explicit command | **MOVES HAND** |
| `set_hand_mode(mode, hand_id=...)` | Set control mode | **MOVES HAND** |
| `read_state(hand_id, force_update)` | Read hand state | Safe |

#### Sensor Methods

| Method | Description |
|--------|-------------|
| `read_state(hand_id, force_update=True)` | Returns sensor/tactile if available |
| `reset_sensor(hand_id)` | May be unsupported on some firmware |

### Device Configuration Examples

#### RS485 Configuration
```python
device_config = {
    "protocol": "RS485",
    "serial_port": "/dev/ttyUSB0",  # Linux
    # "serial_port": "COM3",        # Windows
    "baud_rate": 3000000
}
```

#### EtherCAT Configuration
```python
device_config = {
    "protocol": "EtherCAT"
}
```

---

## 🔧 Troubleshooting

### Common Issues

1. **"Permission denied" on Linux**:
   ```bash
   sudo usermod -a -G dialout $USER
   # Logout and login again
   ```

2. **Device not found**:
   ```bash
   # Check USB connections
   lsusb
   ls /dev/ttyUSB*
   
   # Try different baud rates
   python -m xhandlib.ID --port /dev/ttyUSB0 --baud 115200 --list
   ```

3. **Multiple hands with same ID**:
   ```bash
   # Connect ONE hand at a time and set unique IDs
   python -m xhandlib.ID --port /dev/ttyUSB0 --target 1
   # Power cycle, then connect second hand
   python -m xhandlib.ID --port /dev/ttyUSB1 --target 2
   ```

