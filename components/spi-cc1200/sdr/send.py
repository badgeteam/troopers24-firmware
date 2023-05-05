import os
import socket
import struct
import sys
from binascii import *

import crcmod

s = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)

pdu = unhexlify(sys.argv[1])
pdu = os.urandom(6 * 3)
pdu = struct.pack("B", len(pdu)) + pdu
pdu = unhexlify("aaaaaa930b51de") + pdu + struct.pack(">H", crcmod.mkCrcFun(0x18005, rev=False, initCrc=0xFFFF,
                                                                            xorOut=0x0000)(pdu))

s.sendto(pdu, ("127.0.0.1", 52001))

pdu = b"".join(map(lambda x: b"\xff" if x == "0" else b"\x01", bin(int(hexlify(pdu), 16))[2:]))
pdu = b"\x00" * 128 + pdu + b"\x00" * 128
s.sendto(pdu, ("127.0.0.1", 1234))
