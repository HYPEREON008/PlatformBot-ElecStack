# PlatformBot-ElecStack

Full electrical ROS 2 stack for PlatformBot.

**Repo structure (checked on 2026-04-19):**
- `dev_ws/` is the ROS 2 workspace in this repo
  - `dev_ws/src/base/` is a ROS 2 Python package (`ament_python`)
- `src/` at repo root contains standalone scripts for reference and learning purpose.

> If you are working with ROS 2 packages, use `dev_ws/` as your workspace.

---

## 1) Prerequisites (skip this section)

### Install Git
```bash
sudo apt update
sudo apt install -y git
```

### Configure Git
```bash
git config --global user.name "Your Name"
git config --global user.email "your_email@example.com"
```

### (Recommended) GitHub SSH setup (will save time in pushing)
```bash
ssh-keygen -t ed25519 -C "your_email@example.com"
cat ~/.ssh/id_ed25519.pub
ssh -T git@github.com
```
Add the printed public key to GitHub: **Settings → SSH and GPG keys → New SSH key**.

---

## 2) Clone / Pull
### Clone
Clone anywhere you want (example: home directory):

change ~ to any other folder you want the repo in
```bash
cd ~
git clone git@github.com:HYPEREON008/PlatformBot-ElecStack.git
```

### Pull latest changes
```bash
cd PlatformBot-ElecStack
git pull
```

---

## 3) ROS 2 workspace: source + dependencies + build

### Go to the workspace
This repo contains the ROS 2 workspace under `dev_ws/`:
```bash
cd PlatformBot-ElecStack/dev_ws
```

### Source ROS 2
Use .zsh or any other shell if you dont use bash:
```bash
source /opt/ros/jazzy/setup.bash
```

### Install dependencies with rosdep (skip this section)
```bash
sudo apt update
sudo apt install -y python3-rosdep
sudo rosdep init 2>/dev/null || true
rosdep update

# from dev_ws/
rosdep install --from-paths src --ignore-src -r -y
```

### Build
```bash
# from dev_ws/
colcon build --symlink-install
```

### Source the overlay (every new terminal)
```bash
# from dev_ws/
source install/setup.bash
```

---

## 4) Build a single package
Example for the `base` package:
```bash
cd PlatformBot-ElecStack/dev_ws
colcon build --symlink-install --packages-select base
source install/setup.bash
```

---

## 5) Create a new package in this workspace (no need as of now)

### Create a Python package 
```bash
cd PlatformBot-ElecStack/dev_ws/src
ros2 pkg create my_py_pkg --build-type ament_python --dependencies rclpy
```

### Create a C++ package
```bash
cd PlatformBot-ElecStack/dev_ws/src
ros2 pkg create my_cpp_pkg --build-type ament_cmake --dependencies rclcpp
```

Then rebuild:
symlink-install is not needed but will help in long term development, you can remove it for now
```bash
cd PlatformBot-ElecStack/dev_ws
colcon build --symlink-install
source install/setup.bash
```

---

## 6) Create your node inside a package (example: Python)

This repo already has a Python package at: `dev_ws/src/base` (package name: `base`).

### Add a node file
Create a file:
- `PlatformBot-ElecStack/dev_ws/src/base/base/talker.py`

Example content:
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class Talker(Node):
    def __init__(self):
        super().__init__('talker')
        self.get_logger().info('Hello from base.talker!')

def main():
    rclpy.init()
    node = Talker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Make it executable:
```bash
chmod +x PlatformBot-ElecStack/dev_ws/src/base/base/talker.py
```

### Register the node in `setup.py`
Edit: `dev_ws/src/base/setup.py` and add an entry under `console_scripts`:

```python
entry_points={
    'console_scripts': [
        'talker = base.talker:main',
    ],
},
```

### Build + run
symlink-install is not needed but will help in long term development, you can remove it for now

```bash
cd PlatformBot-ElecStack/dev_ws
colcon build --symlink-install --packages-select base
source install/setup.bash
ros2 run base talker
```

---

## 7) Git push rules for this repo

### What to commit
Commit **only source code** (packages + scripts). Do **not** commit build outputs.

If you build inside `dev_ws/`, these are generated and must not be pushed:
- `dev_ws/build/`
- `dev_ws/install/`
- `dev_ws/log/`

### Create a branch (once per feature)

```bash
cd PlatformBot-ElecStack

# make sure main is up to date
git checkout main
git pull

# create and switch to a new branch
git checkout -b my_branch_name

# push existence of this branch to origin
# for first time using push to new branch use -u (Set upstream)
git push -u origin my_branch_name
```


### Typical workflow
```bash
cd PlatformBot-ElecStack

# check which branch you're on
git branch

# if NOT on your branch, switch to it
git checkout my_branch_name

# check changes
git status

# stage
git add dev_ws/src

# commit
git commit -m "modified xxx nodes"

# push (no need to mention branch anymore)
git push
```
