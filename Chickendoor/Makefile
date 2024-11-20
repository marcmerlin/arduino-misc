flash: killscreen
	#esptool.py --chip esp32 --port /dev/ttyUSB0 --baud 460800 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 80m --flash_size detect 0x1000 bootloader_qio_80m.bin 0x10000 Chickendoor.ino.bin 
	./esptool.py --chip esp32 --port /dev/ttyUSB0 --baud 460800 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 80m --flash_size detect 0x10000 Chickendoor.ino.bin 
	sleep 1 # time for ttyUSB0 to come back
	ls -l /dev/ttyUSB0
	screen -S screen /dev/ttyUSB0 115200


restart: killscreen
	./esptool.py --chip esp32 --port /dev/ttyUSB0 --baud 460800 --before default_reset --after hard_reset chip_id 
	ls -l /dev/ttyUSB0
	screen -S screen /dev/ttyUSB0 115200


killscreen:
	pkill -f -9 'scree[n] .*/dev/ttyUSB0' || :
