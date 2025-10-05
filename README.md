# XHandControl

Comprehensive Python wrapper for controlling XHAND robotic hands via RS-485 and EtherCAT.  
Tested on **Ubuntu 22.04** and **Windows 10/11** with **Python 3.10+**.

---

## ✨ Features

- ✅ **Multi-protocol support**: RS-485 and EtherCAT
- ✅ **Device discovery & enumeration**
- ✅ **Hand ID assignment & management**
- ✅ **Dual-hand control** (Left ID=1, Right ID=2)
- ✅ **Preset actions** (fist, palm, peace sign, OK gesture)
- ✅ **Real-time state monitoring**
- ✅ **Tactile sensor support**
- ✅ **Interactive control examples**
- ✅ **Comprehensive device information**

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

# Install SDK wheel
cd xhand_control_sdk_py
pip install ./xhand_controller-1.1.7-cp310-cp310-linux_x86_64.whl  # Linux
# pip install ./xhand_controller-1.1.7-cp312-cp312-linux_x86_64.whl  # Python 3.12
cd ..

# Install additional dependencies if needed
pip install numpy  # Optional for advanced control
```

---

## 🔧 Hardware Setup for Dual Hands

### USB Device Configuration

When using **two USB-to-RS485 adapters** for dual hand control:

1. **Connect each hand to separate USB ports**:
   ```
   Left Hand  → USB Port 1 → /dev/ttyUSB0 (Linux) or COM3 (Windows)
   Right Hand → USB Port 2 → /dev/ttyUSB1 (Linux) or COM4 (Windows)
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
   # Connect ONLY the left hand first
   python -m xhandlib.ID --port /dev/ttyUSB0 --target 1
   # Power cycle the left hand
   
   # Connect ONLY the right hand
   python -m xhandlib.ID --port /dev/ttyUSB1 --target 2  
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
if hand.exam_open_device(device_config):
    print("✅ Connected successfully!")
    
    # Get device information
    hand.exam_get_sdk_version()
    hand.exam_read_device_info()
    hand.exam_get_hand_type()
    
    # Read sensor state
    hand.exam_read_state(finger_id=5, force_update=True)
    
    # Send preset actions (BE CAREFUL - MOVES THE HAND!)
    # hand.exam_send_command()  # Send current command
    
    # Close connection
    hand.exam_close_device()
else:
    print("❌ Failed to connect")
```

### Dual Hand Control Setup

```python
from xhandlib.xhand import XHandControl
import time

# Initialize both hands
left_hand = XHandControl(hand_id=1, position=0.1, mode=3)
right_hand = XHandControl(hand_id=2, position=0.1, mode=3)

# Device configurations
left_config = {
    "protocol": "RS485", 
    "serial_port": "/dev/ttyUSB0",
    "baud_rate": 3000000
}

right_config = {
    "protocol": "RS485",
    "serial_port": "/dev/ttyUSB1", 
    "baud_rate": 3000000
}

# Connect both hands
left_connected = left_hand.exam_open_device(left_config)
right_connected = right_hand.exam_open_device(right_config)

if left_connected and right_connected:
    print("✅ Both hands connected!")
    
    # Synchronize actions
    # WARNING: This will move both hands simultaneously
    # left_hand.exam_send_command()
    # right_hand.exam_send_command()
    
    # Close both connections
    left_hand.exam_close_device()
    right_hand.exam_close_device()
```

### EtherCAT Connection

```python
from xhandlib.xhand import XHandControl

hand = XHandControl(hand_id=0)

# EtherCAT configuration
ethercat_config = {"protocol": "EtherCAT"}

if hand.exam_open_device(ethercat_config):
    print("✅ EtherCAT connected!")
    # ... your control code ...
    hand.exam_close_device()
```

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

if hand.exam_open_device(device_config):
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
        # hand.exam_send_command()
        time.sleep(2)  # Hold pose for 2 seconds
    
    hand.exam_close_device()
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

| Method | Description | Parameters |
|--------|-------------|------------|
| `exam_enumerate_devices(protocol)` | List available ports | `protocol`: "RS485" or "EtherCAT" |
| `exam_open_device(device_identifier)` | Open device connection | `device_identifier`: Dict with connection params |
| `exam_close_device()` | Close device connection | None |
| `exam_list_hands_id()` | List connected hand IDs | None |

#### Information Methods

| Method | Description | Returns |
|--------|-------------|---------|
| `exam_get_sdk_version()` | Get software SDK version | Prints version |
| `exam_read_version()` | Get hardware SDK version | Prints version |
| `exam_read_device_info()` | Get device information | Prints serial, ID, etc. |
| `exam_get_hand_type()` | Get hand type (left/right) | Prints hand type |
| `exam_serial_number()` | Get serial number | Prints serial |
| `exam_get_hand_name()` | Get hand name | Prints name |
| `exam_set_hand_name(new_name)` | Set hand name | `new_name`: String |

#### Control Methods

| Method | Description | Parameters | ⚠️ Warning |
|--------|-------------|------------|-----------|
| `exam_send_command()` | Send movement command | None | **MOVES HAND** |
| `set_hand_mode(mode)` | Set control mode | `mode`: 0/3/5 | **MOVES HAND** |
| `exam_read_state(finger_id, force_update)` | Read hand state | `finger_id`: 0-11, `force_update`: bool | Safe |

#### Sensor Methods

| Method | Description | Parameters |
|--------|-------------|------------|
| `exam_read_state(finger_id=2, force_update=True)` | Read detailed finger state | `finger_id`: Finger to read (2,5,7,9,11 have sensors) |
| `exam_reset_sensor()` | Reset fingertip sensor | None |

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

4. **Windows COM port issues**:
   - Check Device Manager for COM port numbers
   - Install USB-to-RS485 drivers if needed
   - Use `COM3`, `COM4`, etc. (not `/dev/ttyUSB0`)

### Debug Commands

```bash
# List all hands on all detected ports
for port in /dev/ttyUSB*; do
    echo "Checking $port:"
    python -m xhandlib.ID --port $port --list 2>/dev/null || echo "  No response"
done

# Test connection with verbose output
python -c "
from xhandlib.xhand import XHandControl
import sys
hand = XHandControl()
try:
    ports = hand.exam_enumerate_devices('RS485')
    print('Available ports:', ports)
except Exception as e:
    print('Error:', e)
"
```

---

## ⚠️ Safety Notes

- **Always test with small movements first**
- **Keep emergency stop accessible**
- **Power-cycle hands after ID changes**
- **Use `mode=0` (powerless) for safe testing**
- **Verify hand IDs before dual-hand operations**

---

## 📝 License

This project is provided as-is for educational and research purposes. Please refer to the XHAND SDK license terms for commercial usage.
