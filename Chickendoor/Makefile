flash: killscreen
	# python3 /home/merlin/.arduino15/packages/esp32/tools/esptool_py/4.6/esptool.py --chip esp32 --port /dev/ttyUSB0 --baud 460800 --before default_reset --after hard_reset write_flash -z --flash_mode keep --flash_freq keep --flash_size keep 0x1000 /tmp/arduino_build_310141/Chickendoor.ino.bootloader.bin 0x8000 /tmp/arduino_build_310141/Chickendoor.ino.partitions.bin 0xe000 /home/merlin/.arduino15/packages/esp32/hardware/esp32/3.0.7/tools/partitions/boot_app0.bin 0x10000 /tmp/arduino_build_310141/Chickendoor.ino.bin 
	#esptool.py --chip esp32 --port /dev/ttyUSB0 --baud 460800 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 80m --flash_size detect 0x1000 bootloader_qio_80m.bin 0x10000 Chickendoor.ino.bin 
	./esptool.py --chip esp32 --port /dev/ttyUSB0 --baud 460800 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 80m --flash_size detect 0x10000 Chickendoor.ino.bin 
	sleep 1 # time for ttyUSB0 to come back
	ls -l /dev/ttyUSB0
	screen -S screen /dev/ttyUSB0 115200

copy:
	cp /tmp/arduino_build_*/*.bin .

push: copy
	rsync -av -SH -P --delete --delete-after --force . root@rpi3a1:/root/Chickendoor/


restart: killscreen
	./esptool.py --chip esp32 --port /dev/ttyUSB0 --baud 460800 --before default_reset --after hard_reset chip_id 
	ls -l /dev/ttyUSB0
	screen -S screen /dev/ttyUSB0 115200


killscreen:
	pkill -f -9 '[Ss][Cc][Rr][Ee][Ee][Nn] .*/dev/ttyUSB0' || :
