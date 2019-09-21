UPLOADING BOOTLOADER TO STM32 BLUEPILL FOR ROSSERIAL
      
	1.Install stm32flash in terminal
      
	2.Download bootloader from https://github.com/rogerclarkmelbourne/STM32duino-bootloader/blob/master/bootloader_only_binaries/generic_boot20_pc13.bin
      
	3.Connect stm32 to PC using USB-serial converter. 
      
	4.Navigate to the directory which contains the bootloader.
      
	5.Change boot0 to 1 in stm32.
      
	6.In terminal run the command "stm32flash -v -w [bootloader file] /dev/ttyUSB0.
      

UPLOAD METHOD
      
	>>Tools
           
	     >>Board:"Generic STM32F103C series"
           
	     >>upload method:stm32duino bootloader
           
	     >>port: to be selected
