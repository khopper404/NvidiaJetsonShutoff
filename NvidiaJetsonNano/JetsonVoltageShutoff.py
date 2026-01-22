import serial
import time
import subprocess


ser = serial.Serial(
    port='/dev/ttyUSB0',
    baudrate=115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1 
)

try:
    print(f"Opening serial port: {ser.name}")
    if not ser.is_open:
        ser.open()

    print("Serial port opened. Reading data...")

    while True:
        line = ser.readline(2)

        if line:

            hex = line.hex()
            print(hex)
            if(hex == "0201"):
                print("Shutting down")
                subprocess.run("echo '{JETSON_PASSWORD}' | sudo -S shutdown -h now", shell=True)
        else:
            print("No data received (timeout).")

        time.sleep(0.1) 

except serial.SerialException as e:
    print(f"Serial port error: {e}")
except KeyboardInterrupt:
    print("Program terminated by user.")
finally:
    if ser.is_open:
        ser.close()
        print("Serial port closed.")
