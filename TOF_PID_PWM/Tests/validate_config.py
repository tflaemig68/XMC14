"""Compile-check CAN configuration boundaries with the DAVE ARM toolchain.

Run from any directory: python Tests/validate_config.py
This checks both public headers and real application/library code, not hardware.
"""
from pathlib import Path
import subprocess
import sys

root = Path(__file__).resolve().parents[1]
compiler = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(
    'C:/Infineon/Tools/DAVE IDE/450/eclipse/ARM-GCC-49/bin/arm-none-eabi-gcc.exe')
base = [str(compiler), '-fsyntax-only', '-std=gnu99', '-Wall', '-Wextra', '-Werror',
        '-mcpu=cortex-m0', '-mthumb', '-DXMC1404_Q040x0200']
for folder in ['.', 'Dave/Generated', 'Libraries', 'Libraries/OLED',
               'Libraries/XMCLib/inc', 'Libraries/CMSIS/Include',
               'Libraries/CMSIS/Infineon/XMC1400_series/Include']:
    base += ['-I', str(root / folder)]
cases = [
    ('standard-default', [], None),
    ('standard-max', ['CANLIB_RX_ID=0x7FFUL', 'CANLIB_TX_ID=0x7FFUL'], None),
    ('extended', ['CANLIB_ID_BITS=29U', 'CANLIB_RX_ID=0x18FF0123UL', 'CANLIB_TX_ID=0x18FF0123UL'], None),
    ('extended-max', ['CANLIB_ID_BITS=29U', 'CANLIB_RX_ID=0x1FFFFFFFUL', 'CANLIB_TX_ID=0x1FFFFFFFUL'], None),
    ('bad-width', ['CANLIB_ID_BITS=12U'], 'CANLIB_ID_BITS muss 11 oder 29 sein'),
    ('standard-overflow', ['CANLIB_RX_ID=0x800UL'], 'CANLIB_RX_ID passt nicht'),
    ('extended-overflow', ['CANLIB_ID_BITS=29U', 'CANLIB_RX_ID=0x20000000UL'], 'CANLIB_RX_ID passt nicht'),
    ('negative-id', ['CANLIB_RX_ID=-1'], 'CANLIB_RX_ID passt nicht'),
    ('tx-standard-overflow', ['CANLIB_TX_ID=0x800UL'], 'CANLIB_TX_ID passt nicht'),
    ('tx-extended-overflow', ['CANLIB_ID_BITS=29U', 'CANLIB_TX_ID=0x20000000UL'], 'CANLIB_TX_ID passt nicht'),
    ('tx-negative-id', ['CANLIB_TX_ID=-1'], 'CANLIB_TX_ID passt nicht'),
    ('zero-bitrate', ['CANLIB_BITRATE=0U'], 'CANLIB_BITRATE muss'),
]
for name, defines, expected_error in cases:
    for source in ['main.c', 'Libraries/CAN/CANlib.c']:
        result = subprocess.run(base + ['-D' + d for d in defines] + [str(root/source)],
                                capture_output=True, text=True)
        if expected_error is None:
            assert result.returncode == 0, (name, source, result.stderr)
        else:
            assert result.returncode != 0 and expected_error in result.stderr, (name, source, result.stderr)
    print('PASS ' + name)
print('24 compiler checks passed; no CAN hardware test performed.')
