"""Run actual ARM TOF decoder and scope-selection code in Unicorn.
Dependencies: pip install unicorn pyelftools (or pass a dependency directory).
No I2C timing/hardware emulation. Return value identifies failing C source line.
"""
from pathlib import Path
import subprocess, sys, tempfile
if len(sys.argv) > 1:
    sys.path.insert(0, sys.argv[1])
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS
from unicorn.arm_const import UC_ARM_REG_SP, UC_ARM_REG_LR, UC_ARM_REG_R0, UC_ARM_REG_PC
from elftools.elf.elffile import ELFFile
root = Path(__file__).resolve().parents[1]
gcc = 'C:/Infineon/Tools/DAVE IDE/450/eclipse/ARM-GCC-49/bin/arm-none-eabi-gcc.exe'
with tempfile.TemporaryDirectory(prefix='scope-test-') as tmp:
    output = Path(tmp)/'scope.elf'
    command = [gcc, '-mcpu=cortex-m0', '-mthumb', '-std=gnu99', '-O1',
               '-ffunction-sections', '-fdata-sections', '-nostdlib',
               '-I', str(root), str(root/'Tests/receive_test.c'), str(root/'Libraries/CAN/GroupMonitor.c'),
               '-lgcc',
               '-Wl,--gc-sections,-e,receive_test,-Ttext=0x10000,-Tdata=0x20000000',
               '-o', str(output)]
    command += ['-DXMC1404_Q040x0200']
    for folder in ['Dave/Generated', 'Libraries', 'Libraries/OLED',
                   'Libraries/XMCLib/inc', 'Libraries/CMSIS/Include',
                   'Libraries/CMSIS/Infineon/XMC1400_series/Include']:
        command += ['-I', str(root/folder)]
    subprocess.run(command, check=True, capture_output=True)
    cpu = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
    cpu.mem_map(0x10000, 0x20000)
    cpu.mem_map(0x20000000, 0x10000)
    with output.open('rb') as stream:
        elf = ELFFile(stream)
        for segment in elf.iter_segments():
            if segment['p_type'] == 'PT_LOAD' and segment['p_filesz']:
                cpu.mem_write(segment['p_vaddr'], segment.data())
        entry = elf.header['e_entry']
    cpu.reg_write(UC_ARM_REG_SP, 0x2000FFF0)
    cpu.reg_write(UC_ARM_REG_LR, 0x2F001)
    cpu.emu_start(entry | 1, 0x2F000, count=2000000)
    assert cpu.reg_read(UC_ARM_REG_PC) == 0x2F000, 'Emulation did not finish'
    result = cpu.reg_read(UC_ARM_REG_R0)
    assert result == 0, f'Scope assertion failed at C line {result}'
    print('PASS: 32-group decoding, sorted presence/expiry, 60-second ISE/expiry, invalid/legacy data, clock wrap, 5-second pagination, six-digit ISE limit, common random target/5-second timing/BUSY/source change')
