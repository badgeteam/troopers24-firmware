import hashlib
import os
import socket
import struct
import sys
from binascii import hexlify, unhexlify

import crcmod

SECRET = b"iubeex9aJeinahphae8QuaV1eithooya"
HASH_CHAIN_LEN = 16384

state = SECRET
chain = []
for i in range(HASH_CHAIN_LEN):
    m = hashlib.sha256()
    m.update(state)
    state = m.digest()
    chain += [state]
chain.reverse()

print("#define CC1200_TROOPERS_CHAIN_TAIL \"%s\"" % "".join(map(lambda b: "\\x%02x" % b, chain[0])))

if len(sys.argv) != 3:
    print("usage: %s cmd pyld" % sys.argv[0])
    sys.exit(1)

if os.path.exists("state_msgid.int"):
    with open("state_msgid.int", "r") as f:
        msgid = int(f.read())
    msgid += 1
else:
    msgid = 0

print(b"\n".join(map(hexlify, chain[:msgid])).decode("utf-8"))
troopers_msg = struct.pack("<IBHH", 0xc0ffee00, int(sys.argv[1]), int(sys.argv[2]), msgid) + chain[msgid]
print("sending troopers msg: %s" % hexlify(troopers_msg).decode("utf-8"))

pdu = struct.pack("B", len(troopers_msg)) + troopers_msg
pdu = unhexlify("aaaaaa930b51de") + pdu + struct.pack(">H", crcmod.mkCrcFun(0x18005, rev=False, initCrc=0xFFFF,
                                                                            xorOut=0x0000)(pdu))

with open("state_msgid.int", "w") as f:
    f.write("%d" % msgid)

s = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
s.sendto(pdu, ("127.0.0.1", 52001))

pdu = b"".join(map(lambda x: b"\xff" if x == "0" else b"\x01", bin(int(hexlify(pdu), 16))[2:]))
pdu = b"\x00" * 128 + pdu + b"\x00" * 128
s.sendto(pdu, ("127.0.0.1", 1234))
