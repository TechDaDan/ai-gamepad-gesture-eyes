import serial
import pygame
import cv2
import mediapipe as mp
import time
import threading
import math
import serial.tools.list_ports
import random


class ServoController:
    def __init__(self):
        self.arduino = None
        self.current_mode = 0  # 0=auto, 1=gamepad, 2=opencv
        self.running = True
        
        # Servo ranges and neutral positions
        self.servo_limits = {
            'base': (0, 180, 90),
            'left_vert': (0, 180, 90),
            'right_vert': (0, 180, 90),
            'left_horiz': (50, 130, 90),
            'right_horiz': (50, 130, 90),
            'left_eye': (20, 90, 90),
            'right_eye': (90, 170, 90)
        }
        
        self.current_positions = {k: v[2] for k, v in self.servo_limits.items()}
        self.target_positions = self.current_positions.copy()
        
        # Gamepad state variables
        self.position_locked = False
        self.eye_toggle_state = False
        self.last_button_press_time = {'lock': 0, 'toggle': 0}

        # --- Smoothing & Movement ---
        self.smoothing_factor = 0.04 
        self.eye_smoothing_factor = 0.06

        # --- Autonomous Mode State ---
        self.auto_mode_state = "IDLE"
        self.last_state_change = time.time()
        self.last_action_time = time.time()
        self.state_duration = 0
        self.action_duration = 0
        # Natural blinking state
        self.is_blinking = False
        self.blink_phase_start = 0
        self.blink_state = 0
        self.next_blink_time = time.time() + random.uniform(2, 6)
        
        self.init_components()

    def init_components(self):
        """Initialize all hardware and software components."""
        self.init_arduino()
        self.init_gamepad()
        self.init_opencv()
        self.update_thread = threading.Thread(target=self.update_loop, daemon=True)
        self.update_thread.start()

    def find_arduino_port(self):
        """Automatically find and connect to the Arduino port."""
        ports = serial.tools.list_ports.comports()
        print("Available COM ports:")
        for port in ports:
            print(f"  {port.device} - {port.description}")
        
        sorted_ports = sorted(ports, key=lambda p: 0 if any(k in p.description.lower() for k in ['ch340', 'arduino', 'usb-serial']) else 1)
        
        print("\nTrying ports in priority order...")
        for port in sorted_ports:
            try:
                print(f"Trying to connect to {port.device}...")
                test_serial = serial.Serial(port.device, 115200, timeout=2)
                time.sleep(2)
                test_serial.write(b"MODE:?\n")
                time.sleep(0.5)
                if "MODE:" in test_serial.readline().decode():
                    print(f"✓ Found Arduino on port: {port.device}")
                    return test_serial
                test_serial.close()
            except Exception as e:
                print(f"✗ Failed to connect or test {port.device}: {e}")
        
        print("\n❌ No Arduino found!")
        return None

    def init_arduino(self):
        """Initialize Arduino connection."""
        self.arduino = self.find_arduino_port()
        if self.arduino:
            print("Arduino connected successfully!")
            threading.Thread(target=self.listen_arduino, daemon=True).start()
        else:
            print("Arduino not found. Please check connection.")

    def listen_arduino(self):
        """Listen for Arduino status updates."""
        print("Starting Arduino listener thread...")
        while self.running and self.arduino:
            try:
                if self.arduino.in_waiting > 0:
                    response = self.arduino.readline().decode().strip()
                    if response.startswith("MODE:"):
                        new_mode = int(response.split(":")[1])
                        if self.current_mode != new_mode:
                            self.current_mode = new_mode
                            print(f"\n[Arduino Status] Mode changed to: {self.current_mode}")
            except Exception as e:
                print(f"Arduino communication error: {e}")
                self.arduino = None
                break
        print("Arduino listener thread stopped.")

    def init_gamepad(self):
        """Initialize gamepad."""
        try:
            pygame.init()
            pygame.joystick.init()
            if pygame.joystick.get_count() > 0:
                self.gamepad = pygame.joystick.Joystick(0)
                self.gamepad.init()
                print(f"Gamepad connected: {self.gamepad.get_name()}")
            else:
                self.gamepad = None
        except Exception as e:
            print(f"Failed to initialize gamepad: {e}")

    def init_opencv(self):
        """Initialize OpenCV and MediaPipe."""
        try:
            self.cap = cv2.VideoCapture(0)
            self.mp_hands = mp.solutions.hands
            self.hands = self.mp_hands.Hands(static_image_mode=False, max_num_hands=1, min_detection_confidence=0.7)
            self.mp_drawing = mp.solutions.drawing_utils
            print("OpenCV and MediaPipe initialized.")
        except Exception as e:
            print(f"Failed to initialize OpenCV: {e}")
            self.cap = None

    def send_to_arduino(self, command):
        """Send a command to the Arduino."""
        if self.arduino:
            try:
                self.arduino.write((command + '\n').encode())
                self.arduino.flush()
            except Exception:
                self.arduino = None

    def set_mode(self, mode):
        """Set the control mode."""
        if 0 <= mode <= 2:
            self.send_to_arduino(f"MODE:{mode}")

    def apply_smoothing_curve(self, current, target, factor):
        """Apply a non-linear smoothing for more natural movement."""
        distance = target - current
        ease_factor = factor + (abs(distance) / 180.0) * (factor * 2)
        return current + distance * min(ease_factor, 1.0)

    def update_all_servos(self):
        """Update all servo positions based on their targets using the smoothing curve."""
        for key, target_val in self.target_positions.items():
            min_val, max_val, _ = self.servo_limits[key]
            clamped_target = max(min_val, min(max_val, target_val))
            factor = self.eye_smoothing_factor if 'eye' in key else self.smoothing_factor
            self.current_positions[key] = self.apply_smoothing_curve(
                self.current_positions[key], clamped_target, factor
            )

    def update_loop(self):
        """Main update loop that runs in a thread."""
        while self.running:
            try:
                if self.current_mode == 0: self.handle_auto_mode()
                elif self.current_mode == 1: self.handle_gamepad()
                elif self.current_mode == 2: self.handle_opencv()
                
                self.update_all_servos()
                self.send_servo_positions()
            except Exception as e:
                print(f"Error in update loop: {e}")
            time.sleep(0.02)

    def handle_gamepad(self):
        """Handle gamepad input with corrected tilt and eye controls."""
        if not self.gamepad: return
        pygame.event.pump()
        current_time = time.time()
        
        lock_button = self.gamepad.get_button(4) or self.gamepad.get_button(5)
        if lock_button and (current_time - self.last_button_press_time['lock'] > 0.5):
            self.position_locked = not self.position_locked
            self.last_button_press_time['lock'] = current_time
            print(f"Position Lock: {'ON' if self.position_locked else 'OFF'}")

        if self.position_locked: return

        # --- TILT & VERTICAL FIX ---
        l2 = self.gamepad.get_button(6)
        r2 = self.gamepad.get_button(7)
        tilt_adjustment = 0
        if r2: tilt_adjustment = -30 # Right tilt: L goes up (less), R goes down (less)
        if l2: tilt_adjustment = 30  # Left tilt: L goes down (more), R goes up (more)

        dpad_y = self.gamepad.get_hat(0)[1]
        
        # Corrected Vertical Logic: Left stick UP (-1) should result in a lower servo value.
        left_v_target = 90 + (self.gamepad.get_axis(1) * 90) + (dpad_y * 90) + tilt_adjustment
        # Corrected Vertical Logic: Right stick UP (-1) should result in a higher servo value (opposite).
        right_v_target = 90 - (self.gamepad.get_axis(3) * 90) - (dpad_y * 90) + tilt_adjustment
        
        self.target_positions['left_vert'] = left_v_target
        self.target_positions['right_vert'] = right_v_target
        
        # Eye button controls
        a_pressed = self.gamepad.get_button(0)
        b_pressed = self.gamepad.get_button(1)
        if b_pressed and (current_time - self.last_button_press_time['toggle'] > 0.5):
            self.eye_toggle_state = not self.eye_toggle_state
            self.last_button_press_time['toggle'] = current_time

        if a_pressed or self.eye_toggle_state:
            self.target_positions['left_eye'], self.target_positions['right_eye'] = 10, 170
        else:
            self.target_positions['left_eye'], self.target_positions['right_eye'] = 90, 90
            
        # Horizontal control
        self.target_positions['base'] = 90 - (self.gamepad.get_axis(2) * 90)
        horiz_offset = self.gamepad.get_axis(0) * 40
        self.target_positions['left_horiz'] = 90 - horiz_offset
        self.target_positions['right_horiz'] = 90 - horiz_offset

    def handle_opencv(self):
        """Handle OpenCV hand tracking with corrected controls and all gestures restored."""
        if not self.cap or not self.cap.isOpened(): return
        ret, frame = self.cap.read()
        if not ret: return
        
        frame = cv2.flip(frame, 1)
        results = self.hands.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        
        if results.multi_hand_landmarks:
            lm = results.multi_hand_landmarks[0].landmark
            self.mp_drawing.draw_landmarks(frame, results.multi_hand_landmarks[0], self.mp_hands.HAND_CONNECTIONS)

            is_pinched = math.hypot(lm[4].x - lm[8].x, lm[4].y - lm[8].y) < 0.05
            fingers_up = [lm[i].y < lm[i-2].y for i in [8, 12, 16, 20]]
            is_victory = fingers_up.count(True) == 2 and fingers_up[0] and fingers_up[1]
            is_fist = fingers_up.count(True) == 0

            # --- FIX: Inverted vertical control ---
            # Top of screen (lm[...].y ~ 0) should be UP (low servo value for left, high for right)
            vert_target = lm[0].y * 180 
            
            if is_pinched:
                self.target_positions['left_horiz'] = 90 - ((lm[8].x - 0.5) * 80)
                self.target_positions['right_horiz'] = 90 - ((lm[8].x - 0.5) * 80)
                # Use hand's overall vertical position for eye vertical
                eye_vert_target = lm[0].y * 180
                self.target_positions['left_vert'] = eye_vert_target
                self.target_positions['right_vert'] = 180 - eye_vert_target
                self.target_positions['base'] = 90

            elif is_victory:
                self.target_positions['base'] = (1 - lm[0].x) * 180
                # Use corrected vert_target
                tilt = (lm[5].y - lm[17].y) * 150
                self.target_positions['left_vert'] = vert_target + tilt
                self.target_positions['right_vert'] = 180 - vert_target + tilt
                self.target_positions['left_horiz'] = 90
                self.target_positions['right_horiz'] = 90

            elif is_fist:
                self.target_positions['left_eye'], self.target_positions['right_eye'] = 10, 170
                self.target_positions['left_horiz'] = 90 - ((lm[5].x - 0.5) * 80)
                self.target_positions['right_horiz'] = 90 - ((lm[5].x - 0.5) * 80)
                self.target_positions['left_vert'] = vert_target
                self.target_positions['right_vert'] = 180 - vert_target
            
            else: # Default Palm Control
                self.target_positions['left_eye'], self.target_positions['right_eye'] = 90, 90
                self.target_positions['base'] = (1 - lm[0].x) * 180
                self.target_positions['left_vert'] = vert_target
                self.target_positions['right_vert'] = 180 - vert_target
                horiz_target = 130 - (lm[0].x * 80)
                self.target_positions['left_horiz'] = horiz_target
                self.target_positions['right_horiz'] = horiz_target
        
        cv2.imshow('Hand Tracking Control', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'): self.running = False
    
    def handle_auto_mode(self):
        """Run the autonomous behavior state machine."""
        current_time = time.time()
        
        if not self.is_blinking and current_time > self.next_blink_time:
            self.is_blinking, self.blink_state, self.blink_phase_start = True, 1, current_time

        if self.is_blinking:
            if self.blink_state == 1:
                self.target_positions['left_eye'], self.target_positions['right_eye'] = 10, 170
                if current_time - self.blink_phase_start > 0.15:
                    self.blink_state, self.blink_phase_start = 2, current_time
            elif self.blink_state == 2:
                self.target_positions['left_eye'], self.target_positions['right_eye'] = 90, 90
                if current_time - self.blink_phase_start > 0.2:
                    self.is_blinking = False
                    self.next_blink_time = current_time + random.uniform(2, 8)
        
        if current_time - self.last_state_change > self.state_duration:
            self.last_state_change, self.last_action_time = current_time, 0
            possible_states = ["IDLE", "CURIOUS", "ALERT", "SLEEPY"]
            if self.auto_mode_state in possible_states: possible_states.remove(self.auto_mode_state)
            self.auto_mode_state = random.choice(possible_states)
            self.state_duration = random.uniform(5, 15)
            print(f"[Auto Mode] New state: {self.auto_mode_state}")

        if current_time - self.last_action_time > self.action_duration:
            self.last_action_time = current_time
            if self.auto_mode_state == "IDLE":
                self.action_duration = random.uniform(2, 4)
                self.target_positions['base'] = random.uniform(80, 100)
                vert = random.uniform(85, 95)
                self.target_positions.update({'left_vert': vert, 'right_vert': 180 - vert})
                horiz = random.uniform(85, 95)
                self.target_positions.update({'left_horiz': horiz, 'right_horiz': horiz})
            elif self.auto_mode_state == "CURIOUS":
                self.action_duration = random.uniform(1.5, 3)
                self.target_positions['base'] = random.choice([60, 120])
                vert = random.choice([70, 110])
                self.target_positions.update({'left_vert': vert, 'right_vert': 180 - vert})
                horiz = random.uniform(60, 120)
                self.target_positions.update({'left_horiz': horiz, 'right_horiz': horiz})
            elif self.auto_mode_state == "ALERT":
                self.action_duration = random.uniform(3, 5)
                self.target_positions['base'] = random.choice([45, 135])
                self.target_positions.update({'left_vert': 90, 'right_vert': 90, 'left_horiz': 90, 'right_horiz': 90})
            elif self.auto_mode_state == "SLEEPY":
                self.action_duration = random.uniform(4, 8)
                left_eye_target = random.uniform(20, 80) 
                self.target_positions['left_eye'] = left_eye_target
                self.target_positions['right_eye'] = 90 + (90 - left_eye_target)
                self.target_positions.update({'left_vert': 110, 'right_vert': 70})

    def send_servo_positions(self):
        """Send current servo positions to Arduino."""
        pos = [int(self.current_positions[k]) for k in ['base', 'left_vert', 'right_vert', 'left_horiz', 'right_horiz', 'left_eye', 'right_eye']]
        self.send_to_arduino(f"SERVO:{','.join(map(str, pos))}")

    def cleanup(self):
        """Cleanup resources on exit."""
        print("Cleaning up resources...")
        self.running = False
        time.sleep(0.1)
        if self.arduino: self.arduino.close()
        if self.cap: self.cap.release()
        cv2.destroyAllWindows()
        pygame.quit()

def main():
    """Main entry point for the script."""
    controller = ServoController()
    if not controller.arduino:
        controller.cleanup()
        return

    print("\n--- Servo Controller Interface ---\n (1) Auto\n (2) Gamepad\n (3) OpenCV\n (b) Blink\n (s) Status\n (q) Quit\n---------------------------------")
    
    try:
        while controller.running:
            cmd = input("Enter command: ").strip().lower()
            if cmd == 'q': break
            elif cmd == '1': controller.set_mode(0)
            elif cmd == '2': controller.set_mode(1)
            elif cmd == '3': controller.set_mode(2)
            elif cmd == 'b': controller.send_to_arduino("BLINK")
            elif cmd == 's': print(f"\n--- STATUS ---\n Mode: {controller.current_mode}\n Arduino: {controller.arduino is not None}\n Gamepad: {controller.gamepad is not None}\n Lock: {'ON' if controller.position_locked else 'OFF'}\n----------------\n")
            elif cmd: print("Invalid command.")
    except (EOFError, KeyboardInterrupt): pass
    finally:
        controller.cleanup()
        print("Controller stopped.")

if __name__ == "__main__":
    main()

