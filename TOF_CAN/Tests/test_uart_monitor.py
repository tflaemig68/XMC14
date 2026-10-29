"""Host stream parser tests, no serial device needed."""
import importlib.util
from pathlib import Path
import struct
import unittest
spec = importlib.util.spec_from_file_location('monitor', Path(__file__).resolve().parents[1]/'Tools/uart_monitor.py')
monitor = importlib.util.module_from_spec(spec)
spec.loader.exec_module(monitor)
def frame(*values):
    return bytes.fromhex('55 AA') + struct.pack('<hhh', *values)
class DecoderTests(unittest.TestCase):
    def test_every_split(self):
        packet = frame(-1, 300, 50)
        for split in range(9):
            d = monitor.TelemetryDecoder()
            self.assertEqual(d.feed(packet[:split]) + d.feed(packet[split:]), [(-1,300,50)])
    def test_noise_and_multiple_packets(self):
        d = monitor.TelemetryDecoder()
        self.assertEqual(d.feed(b'noise\x55' + frame(150,300,50) + frame(1,2,3)), [(150,300,50),(1,2,3)])
        self.assertEqual(d.pending, b'')
    def test_header_in_payload(self):
        d = monitor.TelemetryDecoder()
        self.assertEqual(d.feed(frame(-21931,0,0)+frame(1,2,3)), [(-21931,0,0),(1,2,3)])
    def test_recovery_after_missing_byte(self):
        d = monitor.TelemetryDecoder()
        result = d.feed(frame(1,2,3)[:-1]+frame(4,5,6)+frame(7,8,9))
        self.assertEqual(result[-1], (7,8,9))
    def test_bytewise_and_idle(self):
        d = monitor.TelemetryDecoder(); result = []
        for byte in frame(-32768,32767,-1):
            result += d.feed(bytes([byte])); result += d.feed(b'')
        self.assertEqual(result,[(-32768,32767,-1)])
if __name__ == '__main__': unittest.main()
