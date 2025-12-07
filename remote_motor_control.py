import asyncio
import threading
import tkinter as tk
from tkinter import ttk
from bleak import BleakClient, BleakScanner

# -----------------------------
# BLE UART UUIDs (from ESP32)
# -----------------------------
NUS_SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
NUS_RX_UUID      = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
NUS_TX_UUID      = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"

ble_client = None
esp32_address = None
current_direction = None  # 'F','R','L','G' or None
loop = asyncio.new_event_loop()  # Dedicated event loop for Bleak

# -----------------------------
# BLE Functions
# -----------------------------
async def connect_ble():
    global ble_client, esp32_address
    print("Scanning for BLE devices...")
    devices = await BleakScanner.discover()
    esp32_address = None
    for d in devices:
        if "ESP32 Vehicle" in (d.name or ""):
            esp32_address = d.address
            break

    if not esp32_address:
        print("ESP32 Vehicle not found!")
        return

    print(f"Connecting to {esp32_address}...")
    ble_client = BleakClient(esp32_address, loop=loop)
    await ble_client.connect()
    print("Connected to ESP32!")

    # Subscribe to notifications (optional)
    def notification_handler(sender, data):
        print(f"ESP32 says: {data.decode()}")

    try:
        await ble_client.start_notify(NUS_TX_UUID, notification_handler)
        print("Notification handler started.")
    except Exception as e:
        print(f"Notification setup failed: {e}")

    # Set disconnected callback to trigger reconnect attempts
    try:
        def _on_disconnect(client):
            print("BLE disconnected. Scheduling reconnect...")
            try:
                asyncio.run_coroutine_threadsafe(reconnect_loop(), loop)
            except Exception as exc:
                print(f"Failed to schedule reconnect: {exc}")

        ble_client.set_disconnected_callback(_on_disconnect)
    except Exception as e:
        print(f"Could not set disconnected callback: {e}")


async def reconnect_loop():
    """Attempt to reconnect to the known ESP32 address with backoff."""
    global ble_client, esp32_address
    backoff = 1
    while True:
        if not esp32_address:
            print("No known ESP32 address to reconnect to. Rescanning...")
            devices = await BleakScanner.discover()
            for d in devices:
                if "ESP32 Motor" in (d.name or ""):
                    esp32_address = d.address
                    break
            if not esp32_address:
                await asyncio.sleep(backoff)
                backoff = min(backoff * 2, 30)
                continue

        try:
            print(f"Reconnecting to {esp32_address}...")
            ble_client = BleakClient(esp32_address, loop=loop)
            await ble_client.connect()
            print("Reconnected to ESP32!")

            # restart notifications and set disconnect callback
            def notification_handler(sender, data):
                print(f"ESP32 says: {data.decode()}")

            try:
                await ble_client.start_notify(NUS_TX_UUID, notification_handler)
            except Exception as e:
                print(f"Notification restart failed: {e}")

            try:
                ble_client.set_disconnected_callback(lambda c: asyncio.run_coroutine_threadsafe(reconnect_loop(), loop))
            except Exception:
                pass

            break
        except Exception as e:
            print(f"Reconnect attempt failed: {e}")
            await asyncio.sleep(backoff)
            backoff = min(backoff * 2, 30)

async def send_command_async(cmd: str):
    if ble_client and ble_client.is_connected:
        try:
            # WRITE REQUEST (response=True) to match NimBLE characteristic
            await ble_client.write_gatt_char(NUS_RX_UUID, cmd.encode(), response=True)
            print(f"Sent command: {cmd}")
        except Exception as e:
            print(f"Failed to send {cmd}: {e}")
    else:
        print("BLE client not connected! Scheduling reconnect and queuing command if needed.")
        # Try to trigger a reconnect in background
        try:
            asyncio.run_coroutine_threadsafe(reconnect_loop(), loop)
        except Exception:
            pass

def send_command(cmd: str):
    asyncio.run_coroutine_threadsafe(send_command_async(cmd), loop)

# -----------------------------
# GUI Commands
# -----------------------------
def forward():
    set_direction('F')

def reverse():
    set_direction('R')

def stop():
    global current_direction
    current_direction = None
    send_command("S")

def left():
    set_direction('L')

def right():
    set_direction('G')

def set_direction(dir_char: str):
    """Set the current movement direction and send the initial command."""
    global current_direction
    current_direction = dir_char
    # send integer percent based on current (float) slider value
    try:
        spd = int(round(float(speed_var.get())))
    except Exception:
        spd = int(speed_var.get())
    send_command(f"{current_direction}{spd}")

# -----------------------------
# Start asyncio loop in separate thread
# -----------------------------
def start_loop():
    asyncio.set_event_loop(loop)
    loop.run_until_complete(connect_ble())
    loop.run_forever()

threading.Thread(target=start_loop, daemon=True).start()

# -----------------------------
# Tkinter GUI
# -----------------------------
root = tk.Tk()
root.title("ESP32 Motor Controller")

speed_var = tk.DoubleVar(value=50.0)

frame = ttk.Frame(root, padding=10)
frame.grid()

ttk.Label(frame, text="Speed (%)").grid(row=0, column=0, columnspan=3)
def on_speed_change(value):
    # `value` comes as a string/float from the Scale; keep float for higher resolution
    try:
        v = float(value)
    except Exception:
        v = float(speed_var.get())
    speed_var.set(v)
    if current_direction:
        send_command(f"{current_direction}{int(round(v))}")

speed_slider = ttk.Scale(frame, from_=0, to=100, variable=speed_var, orient=tk.HORIZONTAL, length=200, command=on_speed_change)
speed_slider.grid(row=1, column=0, columnspan=3, pady=5)

# Control buttons
ttk.Button(frame, text="Forward", command=forward, width=10).grid(row=2, column=1, pady=5)
ttk.Button(frame, text="Left",    command=left,    width=10).grid(row=3, column=0, pady=5)
ttk.Button(frame, text="Stop",    command=stop,    width=10).grid(row=3, column=1, pady=5)
ttk.Button(frame, text="Right",   command=right,   width=10).grid(row=3, column=2, pady=5)
ttk.Button(frame, text="Reverse", command=reverse, width=10).grid(row=4, column=1, pady=5)

# Keyboard controls: map keys to specific full-speed commands
def on_keypress(event):
    key = (event.keysym or '').lower()
    if key == 'w':
        try:
            speed_var.set(100)
        except Exception:
            pass
        send_command("F100")
    elif key == 's':
        try:
            speed_var.set(100)
        except Exception:
            pass
        send_command("R100")
    elif key == 'a':
        try:
            speed_var.set(100)
        except Exception:
            pass
        send_command("L100")
    elif key == 'd':
        try:
            speed_var.set(100)
        except Exception:
            pass
        send_command("G100")
    elif key == 'p':
        try:
            speed_var.set(0)
        except Exception:
            pass
        send_command("S")
    elif key == 'b':
        try:
            speed_var.set(0)
        except Exception:
            pass
        send_command("B")
    elif key == 'h':
        send_command("H")
        

# Ensure the root has keyboard focus so key events are received
root.bind('<KeyPress>', on_keypress)
try:
    root.focus_set()
except Exception:
    pass

root.mainloop()
