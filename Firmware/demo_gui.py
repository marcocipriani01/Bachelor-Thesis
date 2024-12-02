import threading
from time import sleep
import serial
import serial.tools.list_ports
import tkinter as tk
from tkinter import ttk


class DemoGUIApp:
    def __init__(self, tk_root):
        self.root = tk_root
        self.root.title("DMM GUI")
        self.root.geometry('400x600+600+200')
        self.root.resizable(width=False, height=False)
        self.style = ttk.Style()
        self.style.theme_use('vista')

        # Serial port
        self.ser = None
        self.connected = False

        # Serial Port combo box
        self.port_label = tk.Label(self.root, text="Select serial port:")
        self.port_label.pack(pady=5)
        self.port_combo = ttk.Combobox(self.root, values=self.get_serial_ports())
        self.port_combo.pack(pady=5)
        self.connect_button = tk.Button(self.root, text="Connect", command=self.toggle_connection)
        self.connect_button.pack(pady=5)

        # Sampling Time combo box
        self.sampling_label = tk.Label(self.root, text="Select sampling time (ms):")
        self.sampling_label.pack(pady=5)
        self.sampling_combo = ttk.Combobox(self.root, values=['5', '10', '50', '100', '200', '500', '1000'])
        self.sampling_combo.pack(pady=5)
        self.sampling_button = tk.Button(self.root, text="Apply", command=self.set_sampling_time)
        self.sampling_button.pack(pady=5)

        self.waveform_label = tk.Label(self.root, text="Waveform: Unknown", font=('Helvetica', 14))
        self.waveform_label.pack(pady=10)

        self.min_voltage_text = self.create_output_display("Min voltage (V):")
        self.max_voltage_text = self.create_output_display("Max voltage (V):")
        self.avg_voltage_text = self.create_output_display("Average voltage (V):")
        self.rms_voltage_text = self.create_output_display("RMS voltage (V):")
        self.frequency_text = self.create_output_display("Frequency (Hz):")
        self.period_text = self.create_output_display("Period (ms):")
        self.amplitude_text = self.create_output_display("Fundamental frequency amplitude (V):")

        # Thread for reading serial data
        self.read_thread = None
        self.running = False

    def create_output_display(self, description: str):
        label = tk.Label(self.root, text=description)
        label.pack(pady=2)
        text = tk.Entry(self.root)
        text.pack(pady=2)
        return text

    @staticmethod
    def get_serial_ports():
        """Return a list of available serial ports."""
        ports = serial.tools.list_ports.comports()
        return [port.device for port in ports]

    def toggle_connection(self):
        if not self.connected:
            port = self.port_combo.get()
            if port == "":
                return
            try:
                self.ser = serial.Serial(port, 115200, timeout=1)
                self.ser.flush()
                self.connected = True
                self.connect_button.config(text="Disconnect")
                self.running = True
                self.read_thread = threading.Thread(target=self.read_data, daemon=True)
                self.read_thread.start()
            except serial.SerialException as e:
                print(f"Error connecting to port {port}: {e}")
        else:
            if self.ser is not None:
                self.ser.close()
            self.connected = False
            self.connect_button.config(text="Connect")
            self.running = False
            if self.read_thread:
                self.read_thread.join()

    def read_data(self):
        while self.running:
            if self.ser is not None and self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8').strip()
                self.process_command(line)
            else:
                sleep(0.1)

    def process_command(self, command):
        """Process incoming serial data commands and update UI."""
        if command.startswith('>m'):
            self.min_voltage_text.delete(0, tk.END)
            self.min_voltage_text.insert(0, command[2:])
        elif command.startswith('>M'):
            self.max_voltage_text.delete(0, tk.END)
            self.max_voltage_text.insert(0, command[2:])
        elif command.startswith('>A'):
            self.avg_voltage_text.delete(0, tk.END)
            self.avg_voltage_text.insert(0, command[2:])
        elif command.startswith('>R'):
            self.rms_voltage_text.delete(0, tk.END)
            self.rms_voltage_text.insert(0, command[2:])
        elif command.startswith('>f'):
            self.frequency_text.delete(0, tk.END)
            self.frequency_text.insert(0, command[2:])
        elif command.startswith('>T'):
            self.period_text.delete(0, tk.END)
            self.period_text.insert(0, command[2:])
        elif command.startswith('>a'):
            self.amplitude_text.delete(0, tk.END)
            self.amplitude_text.insert(0, command[2:])
        elif command.startswith('>W'):
            waveform_types = {
                0: "Unknown",
                1: "DC",
                2: "Sine",
                3: "Rectified Sine",
                4: "Square",
                5: "Triangle",
                6: "Sawtooth",
            }
            waveform_text = waveform_types.get(int(command[2:]), "Unknown")
            self.waveform_label.config(text=f"Waveform: {waveform_text}")

    def set_sampling_time(self):
        """Send the new sampling time command to the serial port."""
        if self.ser and self.connected:
            sampling = self.sampling_combo.get()
            if sampling == '':
                return
            command = f'>S{int(sampling)}'
            self.ser.write(command.encode())


if __name__ == '__main__':
    root = tk.Tk()
    app = DemoGUIApp(root)
    root.mainloop()
