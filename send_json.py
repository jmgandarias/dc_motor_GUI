
import json
import serial
import time

# --- CONFIGURA AQUÍ ---
JSON_FILE = "config/config.json"     # archivo JSON a enviar
SERIAL_PORT = "COM12"          # en Windows: COMx | en Linux: /dev/ttyUSB0
BAUDRATE = 500000
# -----------------------

def main():
    # Lee el archivo JSON
    with open(JSON_FILE, "r") as f:
        data = json.load(f)

    # Convierte a una sola línea JSON
    json_text = json.dumps(data) + "\n"

    print("⏳ Abriendo el puerto serie...")
    ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
    time.sleep(2)  # Espera a que el ESP32 inicialice el USB-serial

    print("📤 Enviando JSON:")
    print(json_text)

    ser.write(json_text.encode("utf-8"))
    ser.close()

    print("✅ JSON enviado correctamente.")

if __name__ == "__main__":
    main()
