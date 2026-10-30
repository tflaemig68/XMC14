"""Strict compile check of the 32-group monitor and receiver."""
from pathlib import Path
import subprocess
root=Path(__file__).resolve().parents[1]
base=['C:/Infineon/Tools/DAVE IDE/450/eclipse/ARM-GCC-49/bin/arm-none-eabi-gcc.exe',
      '-fsyntax-only','-std=gnu99','-Wall','-Wextra','-Werror','-mcpu=cortex-m0','-mthumb','-DXMC1404_Q040x0200']
for folder in ['.','Dave/Generated','Libraries','Libraries/OLED','Libraries/XMCLib/inc','Libraries/CMSIS/Include','Libraries/CMSIS/Infineon/XMC1400_series/Include']:
    base+=['-I',str(root/folder)]
for source in ['main.c','Libraries/CAN/CANlib.c','Libraries/CAN/GroupMonitor.c','Libraries/UART/UARTlib.c','Libraries/UART/UARTgroups.c']:
    subprocess.run(base+[str(root/source)],check=True)
print('PASS: strict compile of monitor, 17-filter CAN receiver plus TX and ISE model')
