"""Read 55 AA framed int16 telemetry and optionally set group, height and PID gains.
Usage: python Tools/uart_monitor.py COM5 --group 1 --setpoint 150 --kp 0.33 --ki 0.10 --kd 0.02
Requires pyserial. Baud must match UARTLIB_BAUDRATE in the firmware.
"""
import argparse
import binascii
import struct

def command(group, value, kp, ki, kd):
    """Encode integer wire values; gain arguments are hundredths (33 = 0.33)."""
    if not 1 <= group <= 32:
        raise ValueError('Gruppe muss zwischen 1 und 32 liegen')
    if not 0 <= value <= 300:
        raise ValueError('Sollwert muss zwischen 0 und 300 mm liegen')
    if any(not -32768 <= gain <= 32767 for gain in (kp, ki, kd)):
        raise ValueError('KP/KI/KD muessen als Rohwerte in int16 passen')
    head = b'\xa5\x5a' + struct.pack('<Bhhhh', group, value, kp, ki, kd)
    return head + struct.pack('<H', binascii.crc_hqx(head, 0xffff))

def gain_argument(text):
    """Parse exact real gain with resolution 0.01; reject silent rounding."""
    from decimal import Decimal, InvalidOperation
    try:
        raw = Decimal(text) * 100
        if not raw.is_finite() or raw != raw.to_integral_value() or not -32768 <= raw <= 32767:
            raise ValueError()
        return int(raw)
    except (InvalidOperation, ValueError):
        raise argparse.ArgumentTypeError('Verstaerkung: -327.68..327.67, maximal 2 Nachkommastellen')

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
    parser.add_argument('--group', type=int)
    parser.add_argument('--kp', type=gain_argument)
    parser.add_argument('--ki', type=gain_argument)
    parser.add_argument('--kd', type=gain_argument)
    args = parser.parse_args()
    # Vor dem Oeffnen validieren, damit ein Tippfehler nichts uebertraegt.
    fields = (args.group, args.setpoint, args.kp, args.ki, args.kd)
    if any(v is not None for v in fields) and not all(v is not None for v in fields):
        parser.error('Zum Senden --group, --setpoint, --kp, --ki und --kd gemeinsam angeben')
    try:
        request = command(*fields) if args.group is not None else None
    except ValueError as error:
        parser.error(str(error))
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
