import gdb
import csv


# Configuration parameters
ELF_FILE_PATH = "F:/Tesi/Firmware/build/debug/Firmware.elf"
TARGET_ADDRESS = "tcp:localhost:61234"

def read_array(symbol_name: str, size: int, out_path: str):
    symbol = gdb.lookup_symbol(symbol_name)[0]
    if symbol is None:
        raise ValueError(f"Symbol \"{symbol_name}\" not found in ELF file.")

    value = symbol.value()

    if not value:
        raise ValueError(f"Could not retrieve the address of \"{symbol_name}\".")

    with open(out_path, mode='w', newline="") as file:
        writer = csv.writer(file)
        writer.writerow(["i", symbol_name])
        for i in range(size):
            writer.writerow([i, value[i]])

    print(f"Data from '{symbol_name}' has been successfully written to '{out_path}'.")


gdb.execute(f'file {ELF_FILE_PATH}')
gdb.execute(f'target remote {TARGET_ADDRESS}')

read_array("adcData", 4096, "adcData.csv")
read_array("fftMagnitudes", 4096 // 2, "fftMagnitudes.csv")

gdb.execute('disconnect')
gdb.execute('quit')
