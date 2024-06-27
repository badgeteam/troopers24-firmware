#!/bin/bash

# Always fail hard
set -e

i="$1"

read -p "Ready to burn $i. Press any key to continue..."

verify=`espefuse.py dump | grep BLOCK2 | cut -c 56-60 | python -c 'import sys,struct; print(struct.unpack("<H", bytes.fromhex(sys.stdin.read()))[0])'`
if [ "$verify" != "0" ]; then
    read -p "Badge already contains an ID: $verify"
    continue
fi

# Write binary
python -c 'import sys, struct; f=open("id.bin", "wb+"); f.write(struct.pack(">H", int(sys.argv[1])) + bytes([0]*30)); f.close()' $i

# Burn fuses
espefuse.py burn_block_data BLOCK2 id.bin --do-not-confirm > /dev/null

# Verify fuses
verify=`espefuse.py dump | grep BLOCK2 | cut -c 56-60 | python -c 'import sys,struct; print(struct.unpack("<H", bytes.fromhex(sys.stdin.read()))[0])'`
if [ "$i" != $verify ]; then
    echo "Wrote $i, read $verify"
    exit
fi
