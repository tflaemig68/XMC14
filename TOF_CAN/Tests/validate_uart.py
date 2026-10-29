"""Compile UART and run actual ARM binary protocol code in Unicorn.
Dependencies: unicorn, pyelftools; optional first argument is dependency path.
Hardware routing/FIFO interrupts and physical baudrate require a board test.
"""
from pathlib import Path
import subprocess, sys, tempfile
if len(sys.argv)>1: sys.path.insert(0,sys.argv[1])
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS
from unicorn.arm_const import UC_ARM_REG_SP, UC_ARM_REG_LR, UC_ARM_REG_R0, UC_ARM_REG_PC
from elftools.elf.elffile import ELFFile
root=Path(__file__).resolve().parents[1]
gcc='C:/Infineon/Tools/DAVE IDE/450/eclipse/ARM-GCC-49/bin/arm-none-eabi-gcc.exe'
base=[gcc,'-mcpu=cortex-m0','-mthumb','-std=gnu99','-Wall','-Wextra','-Werror','-DXMC1404_Q040x0200']
for folder in ['.','Dave/Generated','Libraries','Libraries/OLED','Libraries/UART','Libraries/XMCLib/inc','Libraries/CMSIS/Include','Libraries/CMSIS/Infineon/XMC1400_series/Include']:
    base+=['-I',str(root/folder)]
for file in ['main.c','Libraries/UART/UARTlib.c','Libraries/UART/UARTprotocol.c']:
    subprocess.run(base+['-fsyntax-only',str(root/file)],check=True)
with tempfile.TemporaryDirectory(prefix='uart-test-') as tmp:
    output=Path(tmp)/'uart.elf'
    subprocess.run(base+['-O1','-ffunction-sections','-fdata-sections','-nostdlib',str(root/'Tests/uart_protocol_test.c'),str(root/'Libraries/UART/UARTprotocol.c'),'-lgcc','-Wl,--gc-sections,-e,uart_test,-Ttext=0x10000,-Tdata=0x20000000','-o',str(output)],check=True)
    cpu=Uc(UC_ARCH_ARM,UC_MODE_THUMB|UC_MODE_MCLASS)
    cpu.mem_map(0x10000,0x20000); cpu.mem_map(0x20000000,0x10000)
    with output.open('rb') as f:
        elf=ELFFile(f); entry=elf.header['e_entry']
        for seg in elf.iter_segments():
            if seg['p_type']=='PT_LOAD' and seg['p_filesz']: cpu.mem_write(seg['p_vaddr'],seg.data())
    cpu.reg_write(UC_ARM_REG_SP,0x2000FFF0); cpu.reg_write(UC_ARM_REG_LR,0x2F001)
    cpu.emu_start(entry|1,0x2F000,count=2000000)
    assert cpu.reg_read(UC_ARM_REG_PC)==0x2F000, 'Emulation timeout'
    assert cpu.reg_read(UC_ARM_REG_R0)==0, f'C assertion line {cpu.reg_read(UC_ARM_REG_R0)}'
print('PASS UART: signed int16, byte order, CRC reference, limits, corrupt/incomplete commands, resynchronization')
