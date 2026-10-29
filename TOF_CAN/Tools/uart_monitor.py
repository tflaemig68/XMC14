"""Read 55 AA framed int16 telemetry and optionally set a height setpoint.
Usage: python Tools/uart_monitor.py COM5 --setpoint 150
Requires pyserial. Baud must match UARTLIB_BAUDRATE in the firmware.
"""
import argparse
import binascii
import struct

def command(value):
    if not 0 <= value <= 300:
        raise ValueError('Sollwert muss zwischen 0 und 300 mm liegen')
    head = b'\xa5\x5a' + struct.pack('<h', value)
    return head + struct.pack('<H', binascii.crc_hqx(head, 0xffff))

class TelemetryDecoder:
    """Find 55 AA and consume exactly six payload bytes, even across USB reads.

    A header inside an already aligned payload is ordinary data. Without a
    checksum, corrupted frames and false synchronization cannot be ruled out.
    """
    def __init__(self):
        self.pending = bytearray()

    def feed(self, data):
        self.pending.extend(data)
        values = []
        while True:
            start = self.pending.find(b'\x55\xaa')
            if start < 0:
                # Retain a partial header at the end of this USB read.
                self.pending[:] = b'\x55' if self.pending.endswith(b'\x55') else b''
                break
            del self.pending[:start]
            if len(self.pending) < 8:
                break
            values.append(struct.unpack('<hhh', self.pending[2:8]))
            del self.pending[:8]
        return values

def main():
    import serial
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('port')
    parser.add_argument('--baud', type=int, default=128000)
    parser.add_argument('--setpoint', type=int)
    args = parser.parse_args()
    # Vor dem Oeffnen validieren, damit ein Tippfehler nichts uebertraegt.
    request = command(args.setpoint) if args.setpoint is not None else None
    with serial.Serial(args.port, args.baud, timeout=0.02, write_timeout=1,
                       bytesize=8, parity='N', stopbits=1,
                       xonxoff=False, rtscts=False, dsrdtr=False) as port:
        if request:
            port.write(request)
        print('Hoehe_mm,Sollwert_mm,PWM_Prozent')
        decoder = TelemetryDecoder()
        while True:
            for values in decoder.feed(port.read(256)):
                print(','.join(map(str, values)), flush=True)
if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
