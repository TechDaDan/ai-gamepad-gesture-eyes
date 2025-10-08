# 🤖 Interactive Robot Eyes Controller

<div align="center">

![Robot Eyes](https://img.shields.io/badge/Status-Active-success?style=for-the-badge)
![Arduino](https://img.shields.io/badge/Arduino-00979D?style=for-the-badge&logo=Arduino&logoColor=white)
![Python](https://img.shields.io/badge/Python-3776AB?style=for-the-badge&logo=python&logoColor=white)
![OpenCV](https://img.shields.io/badge/OpenCV-5C3EE8?style=for-the-badge&logo=opencv&logoColor=white)

**An expressive animatronic eye system with multiple control modes - from autonomous behavior to hand-tracking gestures!**

[Features](#-features) • [Demo](#-demo) • [Hardware](#-hardware-requirements) • [Installation](#-installation) • [Usage](#-usage) • [Contributing](#-contributing)

</div>

---

## 📖 About The Project

**Developed by TechDaDan**

This project brings life to robotic eyes through smooth, natural movements controlled by multiple input methods. Whether running autonomously with lifelike behaviors, controlled via gamepad, or tracked through hand gestures using a webcam - these eyes respond with personality and precision.

Built on Arduino for real-time servo control and Python for intelligent input processing, this system features:
- 🎭 **Autonomous Mode**: Natural idle behaviors, curiosity, alertness, and even sleepy states
- 🎮 **Gamepad Control**: Precise manual control with position locking and eye expressions
- 👋 **Hand Tracking**: OpenCV + MediaPipe powered gesture recognition for intuitive control

### 🎯 What Makes It Special?

- **Smooth Motion**: Advanced linear interpolation for natural, fluid movements
- **Realistic Blinking**: Dynamic, randomized blink patterns that mimic organic behavior
- **State Machine AI**: Autonomous mode with distinct personality states
- **Multiple Control Modes**: Seamlessly switch between auto, gamepad, and hand tracking
- **Optimized Performance**: Efficient 50Hz servo updates with smart smoothing algorithms

---

## ✨ Features

### 🤖 Autonomous Mode
Watch the eyes come alive on their own! The system intelligently cycles through different behavioral states:

| State | Behavior | Description |
|-------|----------|-------------|
| 🟢 **IDLE** | Calm, occasional glances | Neutral, relaxed state with minimal movement |
| 🔵 **CURIOUS** | Quick, exploratory movements | Rapid eye movements scanning the environment |
| 🟡 **ALERT** | Sharp, focused positioning | Direct, attentive gaze with minimal drift |
| 🟣 **SLEEPY** | Slow movements, droopy eyelids | Heavy-lidded, slow blinks, and downward gaze |

**Plus:**
- Natural blinking with randomized intervals (2-8 seconds)
- Smooth state transitions every 5-15 seconds
- Dynamic action timing within each state

### 🎮 Gamepad Control

Full manual control with Xbox/PlayStation-style controllers:

**Controls:**
- **Left Stick**: Control left eye vertical movement
- **Right Stick**: Control right eye vertical movement
- **Left Trigger**: Rotate base counterclockwise
- **Right Trigger**: Rotate base clockwise
- **D-Pad**: Vertical eye adjustment only in Dinput
- **L2/R2**: Eye tilt (left/right lean)
- **A Button**: Close eyes (hold)
- **X Button**: Toggle eyes closed (persistent)
- **LB/RB**: Lock/unlock position

### 👁️ Hand Tracking (OpenCV + MediaPipe)

Control the eyes with natural hand gestures in front of your webcam:

| Gesture | Control | Effect |
|---------|---------|--------|
| ✋ **Open Palm** | General control | Base rotation + vertical + horizontal movement |
| 🤏 **Pinch** | Horizontal only | Fine-tuned left-right eye movement |
| ✌️ **Victory/Peace** | Base + tilt | Rotation with dynamic eye tilting |
| ✊ **Fist** | Eyes closed | Close eyes + horizontal control |

**Hand Position Mapping:**
- Horizontal hand position → Base servo rotation
- Vertical hand position → Eye vertical angle
- Hand tilt → Eye tilt angle

---

## 🔧 Hardware Requirements

### Electronics

| Component | Quantity | Notes |
|-----------|----------|-------|
| Arduino Uno/Nano | 1 | Main controller |
| Servo Motors (SG90 or similar) | 7 | See servo layout below |
| 5V Power Supply | 1 | 2A+ recommended for 7 servos |
| Push Button | 1 | Mode switching (optional) |
| Jumper Wires | ~20 | For connections |
| USB Cable | 1 | Arduino to PC communication |

### Servo Layout

```
Pin 2: Base Rotation Servo
Pin 3: Left Eye Vertical Servo
Pin 4: Right Eye Vertical Servo
Pin 5: Left Eye Horizontal Servo
Pin 6: Right Eye Horizontal Servo
Pin 7: Left Eyelid Servo
Pin 8: Right Eyelid Servo
```

### 3D Printed Parts

**A huge thank you to [James Bruton](https://www.youtube.com/user/jamesbruton) for the incredible 3D printable eye mechanism!**

🔗 Download the STL files here: [XRobots/ServoSmoothing](https://github.com/XRobots/ServoSmoothing)

**Required Prints:**
- Eye mechanism base
- Left and right eye assemblies
- Eyelid mechanisms
- Servo mounting brackets

### Optional Hardware

- 🎮 **Gamepad**: Xbox 360/One, PlayStation 4/5, or any compatible controller
- 📷 **Webcam**: For hand tracking mode (720p+ recommended)

---

## 💻 Software Requirements

### Arduino IDE
- Version 1.8.x or newer
- `Servo.h` library (included with Arduino IDE)

### Python Dependencies
```bash
Python 3.7+
pygame
opencv-python
mediapipe
pyserial
```

---

## 🚀 Installation

### Step 1: Clone the Repository
```bash
git clone https://github.com/yourusername/robot-eyes-controller.git
cd robot-eyes-controller
```

### Step 2: Arduino Setup

1. Open `robot_eyes_controller.ino` in Arduino IDE
2. Select your Arduino board: **Tools → Board → Arduino Uno/Nano**
3. Select the correct COM port: **Tools → Port → (Your Arduino Port)**
4. Upload the code: Click the **Upload** button (→)
5. Wait for "Done uploading" message

### Step 3: Python Environment Setup

**⚠️ Important: If you encounter any errors during installation or runtime, using a virtual environment (venv) is highly recommended!**

```bash
# Create virtual environment (recommended)
python -m venv venv

# Activate virtual environment
# On Windows:
venv\Scripts\activate
# On macOS/Linux:
source venv/bin/activate

# Install required packages
# Option 1: Copy and paste this pip command
pip install pygame opencv-python mediapipe pyserial

# Option 2: Use the requirements.txt file
pip install -r requirements.txt
```

**Create a `requirements.txt` file with:**
```
pygame>=2.0.0
opencv-python>=4.5.0
mediapipe>=0.8.0
pyserial>=3.5
```

### Step 4: Hardware Assembly

1. **Connect servos to Arduino** following the pin layout above
2. **Connect power supply** to servo rail (5V, GND)
3. **Optional**: Connect mode button to pin A1 and GND
4. **Connect Arduino** to PC via USB
5. **Test servo connections** before attaching to mechanism

---

## 🎯 Usage

### Starting the Controller

1. **Power on** Arduino and ensure servos are connected
2. **Run the Python controller**:
```bash
python servo_controller.py
```

3. The system will **auto-detect** your Arduino and initialize all components

### Command Interface

Once running, you'll see this menu:

```
--- Servo Controller Interface ---
 (1) Auto Mode
 (2) Gamepad Mode
 (3) OpenCV Hand Tracking
 (b) Manual Blink
 (s) System Status
 (q) Quit
----------------------------------
```

**Type commands and press Enter:**

| Command | Action |
|---------|--------|
| `1` | Switch to Autonomous Mode |
| `2` | Switch to Gamepad Mode |
| `3` | Switch to Hand Tracking Mode |
| `b` | Trigger a manual blink |
| `s` | Display current system status |
| `q` | Quit the application |

### Mode Switching

**Three ways to change modes:**

1. **Physical Button**: Press the button connected to pin A1 (cycles through modes)
2. **Python Commands**: Type `1`, `2`, or `3` in the console
3. **Programmatic**: Send `MODE:X` via serial (X = 0, 1, or 2)

---

## 🎮 Control Guides

### Gamepad Tips
- Start with **position lock OFF** to get comfortable with controls
- Use **L2/R2 triggers** for dramatic tilting effects
- **Lock position** when you find the perfect expression
- Experiment with **B button toggle** for sustained winks

### Hand Tracking Tips
- Ensure **good lighting** for best tracking results
- Keep your **hand flat** and **fingers extended** for open palm control
- **Pinch gesture**: Touch thumb and index fingertip together
- Stay **0.5-1.5 meters** from the webcam
- Press **'q'** in the OpenCV window to exit hand tracking

### Optimal Settings

**For Smooth Motion:**
- Smoothing factor: `0.04` (general movement)
- Eye smoothing: `0.06-0.07` (faster response)
- Update rate: `50Hz` (20ms intervals)

**For Responsive Control:**
- Increase smoothing factors to `0.08-0.10`
- Reduce servo update delay

---

## 🛠️ Customization

### Adjusting Servo Limits

Edit `servo_limits` in `servo_controller.py`:

```python
self.servo_limits = {
    'base': (0, 180, 90),           # (min, max, neutral)
    'left_vert': (0, 180, 90),
    'right_vert': (0, 180, 90),
    'left_horiz': (50, 130, 90),    # Limited range
    'right_horiz': (50, 130, 90),
    'left_eye': (20, 90, 90),       # Eyelid range
    'right_eye': (90, 170, 90)
}
```

### Modifying Autonomous Behavior

Change state durations and transitions in the `handle_auto_mode()` function:

```python
# Adjust state duration
self.state_duration = random.uniform(5, 15)  # seconds

# Adjust blink frequency
self.next_blink_time = current_time + random.uniform(2, 8)

# Add new states to the state machine
possible_states = ["IDLE", "CURIOUS", "ALERT", "SLEEPY", "YOUR_STATE"]
```

### Tuning Smoothing

Modify these values for different motion characteristics:

```python
self.smoothing_factor = 0.04        # Lower = smoother but slower
self.eye_smoothing_factor = 0.06    # Eyes can be more responsive
```

---

## 🐛 Troubleshooting

### Arduino Not Found
```
❌ Solution: Check USB connection, try different ports, ensure CH340 drivers are installed
```

### Servos Jittering
```
✅ Solution: 
1. Check power supply (needs 2A+ for 7 servos)
2. Reduce smoothing factor
3. Adjust eye closing limits in Arduino code (LEFT_EYE_CLOSE, RIGHT_EYE_CLOSE)
```

### Gamepad Not Detected
```
✅ Solution:
1. Ensure gamepad is connected before running script
2. Test gamepad with pygame: python -m pygame.examples.joystick
3. Try a different USB port
```

### Hand Tracking Not Working
```
✅ Solution:
1. Check webcam permissions
2. Ensure good lighting conditions
3. Update OpenCV: pip install --upgrade opencv-python
4. Test webcam: cv2.VideoCapture(0).read()
```

### Eyes Moving Erratically
```
✅ Solution:
1. Calibrate servo limits in code
2. Check for loose mechanical connections
3. Verify servo horn alignment
4. Increase smoothing factor
```

---

## 📊 Technical Details

### Communication Protocol

**Arduino ← → Python Communication:**

| Command | Format | Description |
|---------|--------|-------------|
| Set Mode | `MODE:X` | X = 0 (Auto), 1 (Gamepad), 2 (OpenCV) |
| Query Mode | `MODE:?` | Request current mode |
| Set Servos | `SERVO:a,b,c,d,e,f,g` | Send 7 servo positions |
| Status | `MODE:X` | Arduino reports current mode |

**Example:**
```
Python → Arduino: "SERVO:90,85,95,90,90,90,90\n"
Arduino → Python: "MODE:1\n"
```

### Performance Metrics

- **Servo Update Rate**: 50Hz (20ms cycle)
- **Serial Baud Rate**: 115200
- **Hand Tracking FPS**: ~30fps (camera dependent)
- **Smoothing Algorithm**: Linear interpolation (LERP)
- **Latency**: <50ms (input to servo response)

---

## 🎨 Project Structure

```
robot-eyes-controller/
├── arduino/
│   └── robot_eyes_controller.ino    # Arduino firmware
├── python/
│   └── servo_controller.py          # Main Python controller
├── docs/
│   ├── wiring_diagram.png           # Connection guide
│   ├── assembly_guide.pdf           # 3D printed parts assembly
│   └── calibration.md               # Servo calibration guide
├── examples/
│   ├── custom_gestures.py           # Add your own gestures
│   └── autonomous_behaviors.py      # Create custom states
├── LICENSE
└── README.md
```

---

## 🤝 Contributing

We welcome contributions! Here's how you can help:

1. **Fork** the repository
2. **Create** a feature branch (`git checkout -b feature/AmazingFeature`)
3. **Commit** your changes (`git commit -m 'Add some AmazingFeature'`)
4. **Push** to the branch (`git push origin feature/AmazingFeature`)
5. **Open** a Pull Request

### Ideas for Contributions
- 🎭 New autonomous behavior states
- 🖐️ Additional hand gestures
- 🎨 Web interface for remote control
- 📱 Mobile app integration
- 🔊 Sound-reactive mode
- 🌈 LED integration for iris effects

---

## 📝 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

## 🙏 Acknowledgments

- **[James Bruton](https://www.youtube.com/user/jamesbruton)** - For the incredible 3D printable eye mechanism design
  - GitHub: [XRobots/ServoSmoothing](https://github.com/XRobots/ServoSmoothing)
  - His YouTube channel is a goldmine for robotics enthusiasts!
  
- **MediaPipe Team** - For the powerful hand tracking ML models
- **Arduino Community** - For the extensive servo control libraries
- **OpenCV Contributors** - For the computer vision foundation

---

## 👥 Project Team

**Created by [Alpha Insiders](https://github.com/alpha-insiders)**

<div align="center">

| Developer | Role | GitHub |
|-----------|------|--------|
| **Ghazi Alpha** | Developer | [@GhaziTrueAlpha](https://github.com/GhaziTrueAlpha) |
| **Paras Deshmukh** | Developer | [@alluringxstalwart](https://github.com/alluringxstalwart) |

</div>

*Alpha Insiders - Building innovative robotics and AI solutions*

---

## 📬 Contact & Support

- **Issues**: Open an issue on GitHub
- **Discussions**: Use GitHub Discussions for questions and ideas
- **Alpha Insiders**: [GitHub Organization](https://github.com/alpha-insiders)

---

## 🌟 Show Your Support

If you found this project helpful or interesting:
- ⭐ **Star** this repository
- 🍴 **Fork** it for your own projects
- 📢 **Share** it with the maker community
- 💬 **Comment** with your builds and improvements!

---

<div align="center">

### Made with ❤️ by Ghazi & Paras from TechDaDan

**Happy Building! 🤖✨**

</div>