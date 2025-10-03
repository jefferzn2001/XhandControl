README.md: |
  # XHandControl

  Unofficial Python wrapper + examples for controlling XHAND robotic hands via RS-485.  
  Tested on **Ubuntu 22.04** with **Anaconda Python 3.10**.

  ---

  ## ✨ Features

  - ✅ RS-485 device discovery  
  - ✅ Hand ID assignment (设备ID)  
  - ✅ Basic pose control (normalized [0–1] or degrees)  
  - ✅ Encoder readout demo  
  - ✅ Multi-pose interactive demo (`example.py`)  
  - ❌ Tactile sensors — not supported in current SDK build

  Left hand is configured as **ID=1**, right hand as **ID=2**.

  ---

  ## ⚙️ Installation

  Clone and set up environment:

  ```bash
  git clone git@github.com:jefferzn2001/XhandControl.git
  cd XhandControl

  # Create conda env
  conda create -n xhand python=3.10 -y
  conda activate xhand

  # Install SDK wheel
  cd xhand_control_sdk_py
  pip install ./xhand_controller-1.1.7-cp310-cp310-linux_x86_64.whl
  cd ..
