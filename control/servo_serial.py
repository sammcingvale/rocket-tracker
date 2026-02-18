"""
Phase 2: Serial interface for commanding the pan/tilt servo controller.

This script talks to the Arduino over USB serial. Use it to:
  1. Test that the serial link works
  2. Send manual angle commands
  3. (Phase 3) Bridge the tracking code to the gimbal

SETUP:
  pip install pyserial

USAGE:
  python servo_serial.py                     # interactive mode
  python servo_serial.py --port /dev/tty...  # specify serial port

FINDING YOUR PORT:
  Mac:    ls /dev/tty.usb*       → something like /dev/tty.usbmodem1101
  Linux:  ls /dev/ttyACM*        → something like /dev/ttyACM0
  Windows: check Device Manager  → something like COM3

LEARNING NOTES:
  Serial communication is how the laptop and Arduino talk to each other.
  They share a "baud rate" (115200 bits/sec in our case) — both sides
  must agree on this or you get garbage data. The protocol is simple:
  we send text commands ending with newline, the Arduino parses them
  and moves the servos.

  In Phase 3, the tracking loop will call send_angles() ~30 times/sec
  to continuously update the gimbal position. Latency through serial
  is typically 1-5ms, which is fast enough for our needs.
"""

import argparse
import sys
import time
import glob
import serial


def find_serial_port():
    """
    Auto-detect the Arduino serial port.

    Scans common patterns for Mac, Linux, and Windows.
    Returns the first match or None.
    """
    patterns = [
        "/dev/tty.usbmodem*",   # Mac (Arduino UNO R4)
        "/dev/tty.usbserial*",  # Mac (older boards)
        "/dev/ttyACM*",         # Linux
        "/dev/ttyUSB*",         # Linux (FTDI)
        "COM[0-9]*",            # Windows
    ]

    for pattern in patterns:
        ports = glob.glob(pattern)
        if ports:
            return ports[0]

    return None


class ServoController:
    """
    Interface to the Arduino servo controller over serial.

    Usage:
        ctrl = ServoController("/dev/tty.usbmodem1101")
        ctrl.home()
        ctrl.set_pan(45)
        ctrl.set_tilt(120)
        ctrl.set_both(90, 90)
        ctrl.close()
    """

    def __init__(self, port, baud=115200, timeout=1):
        """
        Open serial connection to the Arduino.

        Note the 2-second sleep after opening — the Arduino resets
        when a serial connection is established (the DTR line toggles).
        We need to wait for the bootloader to finish before sending
        commands, otherwise they get eaten during the reset.
        """
        print(f"Connecting to {port} at {baud} baud...")
        self.ser = serial.Serial(port, baud, timeout=timeout)
        time.sleep(2)  # Wait for Arduino reset

        # Read and display the startup banner
        while self.ser.in_waiting:
            line = self.ser.readline().decode("utf-8", errors="replace").strip()
            if line:
                print(f"  Arduino: {line}")

        print("Connected!\n")

    def send(self, command):
        """Send a command string and return any response lines."""
        self.ser.write(f"{command}\n".encode())
        time.sleep(0.05)  # Brief pause for Arduino to process

        responses = []
        while self.ser.in_waiting:
            line = self.ser.readline().decode("utf-8", errors="replace").strip()
            if line:
                responses.append(line)
        return responses

    def set_pan(self, angle):
        """Set pan servo angle (0-180)."""
        return self.send(f"P{int(angle)}")

    def set_tilt(self, angle):
        """Set tilt servo angle (0-180)."""
        return self.send(f"T{int(angle)}")

    def set_both(self, pan, tilt):
        """Set both servo angles simultaneously."""
        return self.send(f"B{int(pan)},{int(tilt)}")

    def home(self):
        """Center both servos to home position."""
        return self.send("H")

    def get_angles(self):
        """Query current servo angles. Returns (pan, tilt) or None."""
        responses = self.send("C")
        for r in responses:
            if r.startswith("C"):
                parts = r[1:].split(",")
                if len(parts) == 2:
                    return int(parts[0]), int(parts[1])
        return None

    def sweep(self):
        """Toggle sweep test mode."""
        return self.send("S")

    def close(self):
        """Close the serial connection."""
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("Serial connection closed.")


def interactive_mode(ctrl):
    """
    Interactive command loop — type commands directly.

    This is handy for testing. Type servo commands or use shortcuts:
      h          → home
      s          → sweep test
      p <angle>  → pan
      t <angle>  → tilt
      b <p> <t>  → both
      c          → current angles
      q          → quit

    Or type raw commands that go straight to the Arduino.
    """
    print("=== Interactive Servo Control ===")
    print("Commands: h=home, s=sweep, p <angle>, t <angle>, b <pan> <tilt>, c=current, q=quit")
    print("Or type raw Arduino commands (P90, T45, B90,45, etc.)")
    print()

    while True:
        try:
            user = input("servo> ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\nExiting.")
            break

        if not user:
            continue

        parts = user.split()
        cmd = parts[0].lower()

        if cmd == "q" or cmd == "quit":
            break
        elif cmd == "h" or cmd == "home":
            responses = ctrl.home()
        elif cmd == "s" or cmd == "sweep":
            responses = ctrl.sweep()
        elif cmd == "c" or cmd == "current":
            angles = ctrl.get_angles()
            if angles:
                print(f"  Pan: {angles[0]}°  Tilt: {angles[1]}°")
            continue
        elif cmd == "p" and len(parts) == 2:
            responses = ctrl.set_pan(int(parts[1]))
        elif cmd == "t" and len(parts) == 2:
            responses = ctrl.set_tilt(int(parts[1]))
        elif cmd == "b" and len(parts) == 3:
            responses = ctrl.set_both(int(parts[1]), int(parts[2]))
        else:
            # Send raw command
            responses = ctrl.send(user)

        for r in responses:
            print(f"  Arduino: {r}")


def demo_mode(ctrl):
    """
    Run a quick automated demo — useful for testing without interaction.
    Moves through a sequence of positions to verify everything works.
    """
    print("=== Running Demo Sequence ===\n")

    sequence = [
        ("Home",         90,  90),
        ("Look left",    30,  90),
        ("Look right",   150, 90),
        ("Look up",      90,  30),
        ("Look down",    90,  150),
        ("Upper-left",   30,  30),
        ("Lower-right",  150, 150),
        ("Back to home", 90,  90),
    ]

    for name, pan, tilt in sequence:
        print(f"  {name}: pan={pan}° tilt={tilt}°")
        responses = ctrl.set_both(pan, tilt)
        for r in responses:
            print(f"    Arduino: {r}")
        time.sleep(1.5)  # Wait for servo to reach position

    print("\nDemo complete!")


def main():
    parser = argparse.ArgumentParser(description="Phase 2: Servo serial controller")
    parser.add_argument("--port", type=str, help="Serial port (auto-detected if omitted)")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate")
    parser.add_argument("--demo", action="store_true", help="Run automated demo sequence")
    args = parser.parse_args()

    # Find port
    port = args.port
    if not port:
        port = find_serial_port()
        if not port:
            print("ERROR: Could not auto-detect Arduino serial port.")
            print("Available ports:")
            for pattern in ["/dev/tty.usb*", "/dev/ttyACM*", "/dev/ttyUSB*"]:
                for p in glob.glob(pattern):
                    print(f"  {p}")
            print("\nSpecify manually with: --port /dev/tty.usbmodemXXXX")
            sys.exit(1)
        print(f"Auto-detected port: {port}")

    # Connect
    try:
        ctrl = ServoController(port, args.baud)
    except serial.SerialException as e:
        print(f"ERROR: Could not open serial port: {e}")
        sys.exit(1)

    # Run
    try:
        if args.demo:
            demo_mode(ctrl)
        else:
            interactive_mode(ctrl)
    finally:
        ctrl.close()


if __name__ == "__main__":
    main()
