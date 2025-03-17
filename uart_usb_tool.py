import serial
import serial.tools.list_ports
import threading

def list_ports():
    ports = serial.tools.list_ports.comports()
    return [port.device for port in ports]

def open_serial(port, baudrate=115200, timeout=1):
    try:
        ser = serial.Serial(port, baudrate, timeout=timeout)
        print(f"[INFO] Connected to {port} at baudrate {baudrate}")
        return ser
    except serial.SerialException as e:
        print(f"[ERROR] Cannot open port {port}: {e}")
        return None

def read_serial(ser):
    while True:
        try:
            if ser and ser.is_open:
                data = ser.readline().decode('utf-8').strip()
                if data:
                    print(f"\n[MCU] {data}")
        except Exception as e:
            print(f"[ERROR] Read error: {e}")
            break

def write_serial(ser):
    while True:
        try:
            if ser and ser.is_open:
                user_input = input("[PC] Send: ")
                ser.write(user_input.encode('utf-8') + b'\n')
        except Exception as e:
            print(f"[ERROR] Write error: {e}")
            break

def main():
    available_ports = list_ports()
    if not available_ports:
        print("[ERROR] No USB ports found!")
        return
    
    print("Available ports:")
    for idx, port in enumerate(available_ports):
        print(f"{idx}: {port}")
    
    port_idx = int(input("Select port (enter number): "))
    if port_idx < 0 or port_idx >= len(available_ports):
        print("[ERROR] Invalid port selection!")
        return
    
    port = available_ports[port_idx]
    ser = open_serial(port)
    if not ser:
        return
    
    read_thread = threading.Thread(target=read_serial, args=(ser,), daemon=True)
    read_thread.start()
    write_serial(ser)

if __name__ == "__main__":
    main()
