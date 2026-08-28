import serial
import time

PORT = "COM11"
BAUD = 115200

# Open connection to Arduino
ser = serial.Serial(PORT, BAUD, timeout=1)

# Arduino may reset when serial connection opens
time.sleep(2)

print("Connected to Arduino")

# Send movement command

command = "G28"
ser.write((command + "\n").encode())

command = "G01 X50 Y25 F600"
ser.write((command + "\n").encode())

print("Sent:", command)
print("Waiting for START_DATA...")

# Open CSV file
with open("motor_data.csv", "w") as file:

    recording = False

    while True:

        # Read one line from Arduino
        line = ser.readline().decode(errors="ignore").strip()

        # Ignore empty lines
        if line == "":
            continue

        # Show everything Arduino sends
        print(line)

        # Start recording
        if line == "START_DATA":
            recording = True
            print("Recording started")
            continue

        # Stop recording
        if line == "END_DATA":
            print("Recording finished")
            break

        # Save data while recording
        if recording:
            file.write(line + "\n")

# Close serial connection
ser.close()

print("Serial closed")
print("Data saved to motor_data.csv")