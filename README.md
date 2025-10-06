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
  - Left hand: `/dev/ttyUSB0`, ID=1
  - Right hand: `/dev/ttyUSB1`, ID=0
- **Note**: IDs don't persist after power cycle. Set manually each time using `xhandlib/ID.py`
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
   Left Hand  → USB Port 1 → /dev/ttyUSB0 (Linux)

   python -m xhandlib.ID --port /dev/ttyUSB0 --target 1

   Then put righthand
   ```

**Important**: IDs do NOT persist after power cycling. You must set IDs each time you power on the hands. Do NOT power-cycle after setting IDs - they are active immediately.



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

| Method | Description | Warning |
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
