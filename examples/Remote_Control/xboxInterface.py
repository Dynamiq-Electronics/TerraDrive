import pygame
import serial
import time

SERIAL_PORT = 'COM4'   # change to your bridge ESP32's port
BAUD_RATE   = 115200
DEADZONE    = 0.08     # ignore stick drift below this

def apply_deadzone(val, dz):
    if abs(val) < dz:
        return 0.0
    # rescale so output starts from 0 at the deadzone edge
    sign = 1 if val > 0 else -1
    return sign * (abs(val) - dz) / (1.0 - dz)

def main():
    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        print("No controller found")
        return

    joy = pygame.joystick.Joystick(0)
    joy.init()
    print(f"Controller: {joy.get_name()}")

    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)  # wait for ESP32 to boot
    print(f"Serial open on {SERIAL_PORT}")

    try:
        while True:
            pygame.event.pump()

            # Left stick: axis 0 = X (left/right), axis 1 = Y (up/down)
            y = apply_deadzone(-joy.get_axis(1), DEADZONE)  # forward/back
            x = apply_deadzone( joy.get_axis(0), DEADZONE)  # turn

            # Arcade mix — blend forward and turn into left/right motor speeds
            left  = (y + x) * 100.0
            right = (y - x) * 100.0

            # Clamp to -100, 100
            left  = max(-100.0, min(100.0, left))
            right = max(-100.0, min(100.0, right))

            line = f"{left:.1f},{right:.1f}\n"
            ser.write(line.encode())

            time.sleep(0.02)

    except KeyboardInterrupt:
        print("Stopped")
        ser.write(b"0.0,0.0\n")  # zero motors on exit
        ser.close()
        pygame.quit()

if __name__ == "__main__":
    main()