from gpiozero.pins.lgpio import LGPIOFactory
from gpiozero import AngularServo
import serial
import time
import pandas as pd
import matplotlib.pyplot as plt

# Use lgpio pin factory
factory = LGPIOFactory(chip=0)

# Servo configuration
SERVO_PIN = 21  # GPIO BCM pin
MIN_ANGLE = 0
MAX_ANGLE = 180
MIN_PULSE_WIDTH = 0.5 / 1000  # 0.5 ms
MAX_PULSE_WIDTH = 2.5 / 1000  # 2.5 ms
DELAY = 0

# LiDAR configuration
SERIAL_PORT = '/dev/ttyAMA0'  # UART serial port
BAUD_RATE = 115200

def setup_servo():
    servo = AngularServo(SERVO_PIN,
                         min_angle=MIN_ANGLE,
                         max_angle=MAX_ANGLE,
                         min_pulse_width=MIN_PULSE_WIDTH,
                         max_pulse_width=MAX_PULSE_WIDTH,
                         pin_factory=factory)
    return servo

def setup_lidar():
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    ser.reset_input_buffer()
    return ser

def get_distance(ser):
    """Read distance from TFmini-S LiDAR"""
    total = 0
    for i in range(5):
        while True:
            ser.reset_input_buffer()
            if ser.read(1) != b'\x59':
                continue
            if ser.read(1) == b'\x59':
                frame = ser.read(7)
                if len(frame) != 7:
                    continue

                checksum = (0x59 + 0x59 + sum(frame[:6])) % 256
                if checksum == frame[6]:
                    distance = frame[0] + (frame[1] << 8)
                    total += distance
                    break
    return total / 5
                    
def move_servo(servo, angle):
    servo.angle = angle
    time.sleep(DELAY)  # Give time to move

def main():
    servo = setup_servo()
    ser = setup_lidar()
    scan_data = []

    try:
        print("Starting LiDAR scan...")
        for angle in range(0, 181, 1):  # Sweep from 0 to 180 degrees
            move_servo(servo, angle)
            distance = get_distance(ser)
            scan_data.append((angle, distance))
            print(f"Angle: {angle:3}, Distance: {distance}cm")

    except KeyboardInterrupt:
        print("Scan interrupted!")

    finally:
        with open('lidar_scan.csv', 'w') as f:
            f.write("Angle,Distance(cm)\n")
            for angle, distance in scan_data:
                f.write(f"{angle},{distance}\n")
        print("Scan data saved to lidar_scan.csv")

        df = pd.read_csv("lidar_scan.csv")
        print(df.head(10))

        df.plot()
        plt.show()

        ser.close()

if __name__ == "__main__":
    main()
