"""Compare executed Thumb instructions, NOT clock cycles or real elapsed time.
Builds with the production -Os option. Includes initialization and 100 steps.
Usage: python Tests/benchmark_pid.py <path containing unicorn and elftools>
"""
from pathlib import Path
import json, subprocess, sys, tempfile
if len(sys.argv)>1: sys.path.insert(0,sys.argv[1])
from unicorn import Uc,UC_ARCH_ARM,UC_MODE_THUMB,UC_MODE_MCLASS,UC_HOOK_CODE
from unicorn.arm_const import UC_ARM_REG_SP,UC_ARM_REG_LR,UC_ARM_REG_R0,UC_ARM_REG_R1,UC_ARM_REG_PC
from elftools.elf.elffile import ELFFile
root=Path(__file__).resolve().parents[1]
gcc='C:/Infineon/Tools/DAVE IDE/450/eclipse/ARM-GCC-49/bin/arm-none-eabi-gcc.exe'
rows=[]
with tempfile.TemporaryDirectory(prefix='pid-benchmark-') as tmp:
    out=Path(tmp)/'pid.elf'
    subprocess.run([gcc,'-mcpu=cortex-m0','-mthumb','-std=gnu99','-Os','-Wall','-Wextra','-Werror',
                    '-ffunction-sections','-fdata-sections','-nostdlib','-I',str(root),
                    str(root/'Tests/pid_benchmark.c'),str(root/'Libraries/PID.c'),str(root/'Tests/pid_reference.c'),
                    '-lgcc','-Wl,--gc-sections,-e,pid_benchmark,-Ttext=0x10000,-Tdata=0x20000000','-o',str(out)],check=True)
    with out.open('rb') as f:
        elf=ELFFile(f)
        entry=elf.header['e_entry']
        segments=[(s['p_vaddr'],s.data()) for s in elf.iter_segments() if s['p_type']=='PT_LOAD' and s['p_filesz']]
        symbols={s.name:s['st_value'] & ~1 for s in elf.get_section_by_name('.symtab').iter_symbols()}
    for case,name in enumerate(['PI fixed 50ms','PID fixed 50ms','PI jitter 50/51/52ms','PID jitter 50/51/52ms']):
        row={'case':name}
        for optimized,label in [(0,'reference'),(1,'optimized')]:
            cpu=Uc(UC_ARCH_ARM,UC_MODE_THUMB|UC_MODE_MCLASS)
            cpu.mem_map(0x10000,0x20000); cpu.mem_map(0x20000000,0x10000)
            for address,data in segments: cpu.mem_write(address,data)
            cpu.reg_write(UC_ARM_REG_SP,0x2000FFF0); cpu.reg_write(UC_ARM_REG_LR,0x2F001)
            cpu.reg_write(UC_ARM_REG_R0,case); cpu.reg_write(UC_ARM_REG_R1,optimized)
            counts={'instructions':0,'soft_float_divisions':0}
            def trace(uc,address,size,user):
                counts['instructions']+=1
                if address==symbols.get('__aeabi_fdiv'): counts['soft_float_divisions']+=1
            cpu.hook_add(UC_HOOK_CODE,trace)
            cpu.emu_start(entry|1,0x2F000,count=10000000)
            assert cpu.reg_read(UC_ARM_REG_PC)==0x2F000,'Timeout'
            row[label]=counts
        row['instruction_reduction_percent']=round(100*(1-row['optimized']['instructions']/row['reference']['instructions']),1)
        rows.append(row)
report={'meaning':'Executed Thumb instructions including init + 100 steps; NOT cycles or board runtime','compiler':'DAVE GCC 4.9, -Os, cortex-m0', 'results':rows}
(root/'Tests/pid_benchmark_results.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
for row in rows:
    print(row['case'],row['reference'],'->',row['optimized'],'reduction',row['instruction_reduction_percent'],'%')
