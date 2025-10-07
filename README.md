# XHandControl

Comprehensive Python wrapper for controlling XHAND robotic hands via RS-485 and EtherCAT.  
Tested on **Ubuntu 22.04** and **Windows 10/11** with **Python 3.10+**.

---

## 🚀 Quick Start

The system now uses **automatic device detection** via `/dev/serial/by-id` mapping:
- **Left hand**: `FTA7NRXW` → ID 1
- **Right hand**: `FTA6HQ26` → ID 0

**No more manual ID setting required!** Just plug in USB devices in any order.

### Available Scripts:
- `demo.py` - Dual-hand motion demonstration (different action orders for each hand)
- `encoder.py` - Live joint angle monitoring GUI
- `tactile.py` - Live tactile sensor visualization with reset functionality
- `sim2real.py` - Isaac Sim GUI controlling both simulation and real robots
- `real2sim.py` - Real robot teleoperation mirrored in Isaac Sim

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

## 🔧 Hardware Setup

### Automatic Device Detection

The system automatically detects devices using `/dev/serial/by-id` mapping:

```bash
# Check your device mapping
ls -l /dev/serial/by-id
# Should show:
# FTA7NRXW -> /dev/ttyUSB0 (Left hand)
# FTA6HQ26 -> /dev/ttyUSB1 (Right hand)
```

**No manual configuration needed!** Just:
1. Connect both hands to USB ports
2. Run any script - device detection is automatic
3. IDs are automatically assigned based on hardware serial numbers



## 📋 Usage Examples

### Basic Usage
```python
from xhandlib.xhand import get_device_configs, XHandControl

# Get automatic device configurations
configs = get_device_configs()
left_config, right_config = configs['left'], configs['right']

# Initialize hands
xhand_left = XHandControl(hand_id=1, mode=3)  # Position control
xhand_right = XHandControl(hand_id=0, mode=3)

# Connect to devices
xhand_left.open_device(left_config)
xhand_right.open_device(right_config)

# Read state
state_left = xhand_left.read_state(xhand_left._hand_id, force_update=True)
state_right = xhand_right.read_state(xhand_right._hand_id, force_update=True)

# Send commands (moves the hands!)
xhand_left.send_command()
xhand_right.send_command()

# Close connections
xhand_left.close()
xhand_right.close()
```

### Running Scripts
```bash
# Motion demonstration (different orders for each hand)
python demo.py

# Live encoder monitoring
python encoder.py

# Tactile sensor visualization
python tactile.py

# Isaac Sim integration (requires Isaac Sim)
python sim2real.py    # GUI controls both sim and real
python real2sim.py    # Real robot teleoperation
```
