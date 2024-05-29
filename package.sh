#!/bin/bash

rm -rf ../dist/
mkdir ../dist/
cp build/bootloader/bootloader.bin ../dist/
cp build/partition_table/partition-table.bin ../dist/
cp build/ota_data_initial.bin ../dist/
cp build/phy_init_data.bin ../dist/
cp build/TROOPERS23.bin ../dist/

(cd components/appfs/tools/; ./generate.sh)

cp components/appfs/tools/appfs.bin ../dist/

echo "#!/bin/bash" > ../dist/flash.sh
echo "" >> ../dist/flash.sh
echo 'DEVICE=$1' >> ../dist/flash.sh
echo "esptool.py erase_flash" >> ../dist/flash.sh
echo 'esptool.py -p $DEVICE -b 460800 --before default_reset --after hard_reset --chip esp32  write_flash --flash_mode dio --flash_size detect --flash_freq 80m 0x1000 bootloader.bin 0x8000 partition-table.bin 0xd000 ota_data_initial.bin 0xf000 phy_init_data.bin 0x10000 TROOPERS23.bin' >> ../dist/flash.sh
echo 'esptool.py --port $DEVICE --baud 960000 write_flash 0x330000 appfs.bin' >> ../dist/flash.sh
chmod +x ../dist/flash.sh

rm -f ../dist.zip
zip -r ../dist.zip ../dist/
