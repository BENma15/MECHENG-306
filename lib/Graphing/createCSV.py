import serial

ser = serial.Serial("COM1", 115200)

recording = False

with open("motor_data.csv", "w") as file:

    while True:
        line = ser.readline().decode().strip()

        if line == "START_DATA":
            recording = True
            print("Receiving data...")
            continue

        if line == "END_DATA":
            print("Finished. Saved to motor_data.csv")
            break

        if recording:
            file.write(line + "\n")