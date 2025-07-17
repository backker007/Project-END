from smbus2 import SMBus

with SMBus(1) as bus:
    print("Scanning I2C bus...")
    devices = []
    for address in range(0x03, 0x78):
        try:
            bus.read_byte(address)
            devices.append(hex(address))
        except OSError:
            pass

    if devices:
        print("Found devices at addresses:")
        for device in devices:
            print("  ->", device)
    else:
        print("No I2C devices found.")
