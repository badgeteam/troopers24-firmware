#!/bin/bash

# Always fail hard
set -e

i=1

if [ -f "id.bck" ]; then
    i=`python -c 'import struct; f=open("id.bck", "rb"); print(struct.unpack(">H", f.read(2))[0]); f.close()'`
    read -p "Restored last written id $i. Press any key to continue..."
fi

while :; do
    verify=`espefuse.py dump | grep BLOCK2 | cut -c 56-60 | python -c 'import sys,struct; print(struct.unpack("<H", bytes.fromhex(sys.stdin.read()))[0])'`
    if [ "$verify" != "0" ]; then
        read -p "Badge already contains an ID: $verify"
        continue
    fi

    read -p "Ready to burn $i. Press any key to continue..."
    
    # Write binary
    python -c 'import sys, struct; f=open("id.bin", "wb+"); f.write(struct.pack(">H", int(sys.argv[1])) + bytes([0]*30)); f.close()' $i

    # Burn fuses
    espefuse.py burn_block_data BLOCK2 id.bin > /dev/null

    # Verify fuses
    verify=`espefuse.py dump | grep BLOCK2 | cut -c 56-60 | python -c 'import sys,struct; print(struct.unpack("<H", bytes.fromhex(sys.stdin.read()))[0])'`
    if [ "$i" != $verify ]; then
        echo "Wrote $i, read $verify"
        exit
    fi

    # Move the current id to the last one
    rm -f id.bck
    mv id.bin id.bck

    # Increment i
    ((i=i+1))
done
