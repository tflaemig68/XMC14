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
class CommandTests(unittest.TestCase):
    def test_reference(self):
        self.assertEqual(monitor.command(1,150,33,10,2), bytes.fromhex('A5 5A 01 96 00 21 00 0A 00 02 00 20 D9'))
        self.assertEqual(monitor.command(32,300,32767,-32768,0), bytes.fromhex('A5 5A 20 2C 01 FF 7F 00 80 00 00 44 6E'))
    def test_ranges(self):
        for args in [(0,150,0,0,0),(33,150,0,0,0),(1,-1,0,0,0),(1,301,0,0,0),
                     (1,150,32768,0,0),(1,150,0,-32769,0),(1,150,0,0,32768)]:
            with self.assertRaises(ValueError): monitor.command(*args)
    def test_gain_resolution(self):
        self.assertEqual(monitor.gain_argument('0.33'),33)
        self.assertEqual(monitor.gain_argument('-327.68'),-32768)
        self.assertEqual(monitor.gain_argument('327.67'),32767)
        for value in ['0.001','327.68','-327.69','NaN','Infinity','abc']:
            with self.assertRaises(Exception): monitor.gain_argument(value)

if __name__ == '__main__': unittest.main()
