import sys

x = sys.stdin.read(1024)
if x == "":
    x = "0x40087D51:0x3FFB11F0 0x40084699:0x3FFB1210 0x4000BFED:0x3FFBBA50 0x400923C1:0x3FFBBA60 0x40101D1A:0x3FFBBA80 0x40101D27:0x3FFBBAA0 0x400D3CE2:0x3FFBBAC0 0x400900E3:0x3FFBBAE0"


x = x.split(" ")
x = list(map(lambda x: x.split(":"), x))

#print(x)


from elftools.elf.elffile import ELFFile


elf = ELFFile(open("build/TROOPERS24.elf", 'rb'))
symtab = elf.get_section_by_name('.symtab')

for a in x:
    #for addr in a:
        addr = a[0]
        addr = int(addr, 16)
        #print(addr)
        name, base = "", 0
        for s  in symtab.iter_symbols():
            if addr >= s.entry["st_value"] and  addr - s.entry["st_value"] < addr-base:
                name = s.name
                base = s.entry["st_value"]
    
        print("%s+%d" % (name, addr-base))
