# MPLAB IDE generated this makefile for use with GNU make.
# Project: microPCR.mcp
# Date: Wed Jul 22 22:27:53 2015

AS = MPASMWIN.exe
CC = mcc18.exe
LD = mplink.exe
AR = mplib.exe
RM = rm

./OUTPUT/microPCR.cof : OBJECTS/usb_device.o OBJECTS/usb_function_hid.o OBJECTS/Init.o OBJECTS/Pragma.o OBJECTS/GlobalTypeVars.o OBJECTS/Timer.o OBJECTS/main.o OBJECTS/Delay.o OBJECTS/usb_descriptors.o OBJECTS/PCRTask.o OBJECTS/Usb_Task.o OBJECTS/Temp_Table.o
	$(LD) /l"C:\MCC18\lib" "rm18f4553 - HID Bootload.lkr" "OBJECTS\usb_device.o" "OBJECTS\usb_function_hid.o" "OBJECTS\Init.o" "OBJECTS\Pragma.o" "OBJECTS\GlobalTypeVars.o" "OBJECTS\Timer.o" "OBJECTS\main.o" "OBJECTS\Delay.o" "OBJECTS\usb_descriptors.o" "OBJECTS\PCRTask.o" "OBJECTS\Usb_Task.o" "OBJECTS\Temp_Table.o" /u_CRUNTIME /u_DEBUG /z__MPLAB_BUILD=1 /z__MPLAB_DEBUG=1 /o"./OUTPUT\microPCR.cof" /M"./OUTPUT\microPCR.map" /W

OBJECTS/usb_device.o : USB/usb_device.c C:/MCC18/h/stdio.h C:/MCC18/h/stdlib.h C:/MCC18/h/string.h usb/usb_common.h usb/usb_device.h USB/usb_device.c DEFINE/GenericTypeDefs.h C:/MCC18/h/stddef.h CONFIG/Compiler.h C:/MCC18/h/p18cxxx.h C:/MCC18/h/p18f4553.h C:/MCC18/h/stdarg.h USB/USB.h USB/usb_config.h C:/MCC18/h/limits.h usb/usb_ch9.h usb/usb_hal.h USB/usb_hal_pic18.h HardwareProfile.h HardwareProfile\ -\ PICDEM\ FSUSB.h
	$(CC) -p=18F4553 /i"C:\MCC18\h" "USB\usb_device.c" -fo="./OBJECTS\usb_device.o" -D__DEBUG -Ou- -Ot- -Ob- -Op- -Or- -Od- -Opa-

OBJECTS/usb_function_hid.o : USB/usb_function_hid.c C:/MCC18/h/stdio.h C:/MCC18/h/stdlib.h C:/MCC18/h/string.h usb/usb_common.h usb/usb_device.h USB/usb_function_hid.c DEFINE/GenericTypeDefs.h C:/MCC18/h/stddef.h CONFIG/Compiler.h C:/MCC18/h/p18cxxx.h C:/MCC18/h/p18f4553.h C:/MCC18/h/stdarg.h USB/usb.h USB/usb_config.h C:/MCC18/h/limits.h usb/usb_ch9.h usb/usb_hal.h USB/usb_hal_pic18.h USB/usb_function_hid.h
	$(CC) -p=18F4553 /i"C:\MCC18\h" "USB\usb_function_hid.c" -fo="./OBJECTS\usb_function_hid.o" -D__DEBUG -Ou- -Ot- -Ob- -Op- -Or- -Od- -Opa-

OBJECTS/Init.o : CONFIG/Init.c C:/MCC18/h/stdio.h C:/MCC18/h/stdlib.h C:/MCC18/h/string.h usb/usb_common.h usb/usb_device.h PCR/PCRTask.h CONFIG/Init.c HardwareProfile.h HardwareProfile\ -\ PICDEM\ FSUSB.h CONFIG/Compiler.h C:/MCC18/h/p18cxxx.h C:/MCC18/h/p18f4553.h C:/MCC18/h/stdarg.h C:/MCC18/h/stddef.h CONFIG/Init.h USB/usb.h DEFINE/GenericTypeDefs.h USB/usb_config.h C:/MCC18/h/limits.h usb/usb_ch9.h usb/usb_hal.h USB/usb_hal_pic18.h DEFINE/GlobalTypeVars.h DEFINE/UserDefs.h PCR/Timer.h
	$(CC) -p=18F4553 /i"C:\MCC18\h" "CONFIG\Init.c" -fo="./OBJECTS\Init.o" -D__DEBUG -Ou- -Ot- -Ob- -Op- -Or- -Od- -Opa-

OBJECTS/Pragma.o : CONFIG/Pragma.c HardwareProfile.h HardwareProfile\ -\ PICDEM\ FSUSB.h
	$(CC) -p=18F4553 /i"C:\MCC18\h" "CONFIG\Pragma.c" -fo="./OBJECTS\Pragma.o" -D__DEBUG -Ou- -Ot- -Ob- -Op- -Or- -Od- -Opa-

OBJECTS/GlobalTypeVars.o : DEFINE/GlobalTypeVars.c DEFINE/GlobalTypeVars.h DEFINE/GenericTypeDefs.h C:/MCC18/h/stddef.h DEFINE/UserDefs.h
	$(CC) -p=18F4553 /i"C:\MCC18\h" "DEFINE\GlobalTypeVars.c" -fo="./OBJECTS\GlobalTypeVars.o" -D__DEBUG -Ou- -Ot- -Ob- -Op- -Or- -Od- -Opa-

OBJECTS/Timer.o : PCR/Timer.c C:/MCC18/h/stdio.h C:/MCC18/h/stdlib.h C:/MCC18/h/string.h PCR/Timer.c HardwareProfile.h HardwareProfile\ -\ PICDEM\ FSUSB.h CONFIG/Compiler.h C:/MCC18/h/p18cxxx.h C:/MCC18/h/p18f4553.h C:/MCC18/h/stdarg.h C:/MCC18/h/stddef.h DEFINE/GenericTypeDefs.h DEFINE/GlobalTypeVars.h DEFINE/UserDefs.h PCR/Timer.h
	$(CC) -p=18F4553 /i"C:\MCC18\h" "PCR\Timer.c" -fo="./OBJECTS\Timer.o" -D__DEBUG -Ou- -Ot- -Ob- -Op- -Or- -Od- -Opa-

OBJECTS/main.o : main.c C:/MCC18/h/stdio.h C:/MCC18/h/stdlib.h C:/MCC18/h/string.h usb/usb_common.h usb/usb_device.h CONFIG/BootLoader.h main.c HardwareProfile.h HardwareProfile\ -\ PICDEM\ FSUSB.h PCR/Timer.h USB/usb.h CONFIG/Compiler.h C:/MCC18/h/p18cxxx.h C:/MCC18/h/p18f4553.h C:/MCC18/h/stdarg.h C:/MCC18/h/stddef.h DEFINE/GenericTypeDefs.h USB/usb_config.h C:/MCC18/h/limits.h usb/usb_ch9.h usb/usb_hal.h USB/usb_hal_pic18.h CONFIG/Init.h
	$(CC) -p=18F4553 /i"C:\MCC18\h" "main.c" -fo="./OBJECTS\main.o" -D__DEBUG -Ou- -Ot- -Ob- -Op- -Or- -Od- -Opa-

OBJECTS/Delay.o : TOOLS/Delay.c TOOLS/Delay.h TOOLS/Delay.c DEFINE/GenericTypeDefs.h C:/MCC18/h/stddef.h
	$(CC) -p=18F4553 /i"C:\MCC18\h" "TOOLS\Delay.c" -fo="./OBJECTS\Delay.o" -D__DEBUG -Ou- -Ot- -Ob- -Op- -Or- -Od- -Opa-

OBJECTS/usb_descriptors.o : USB/usb_descriptors.c C:/MCC18/h/stdio.h C:/MCC18/h/stdlib.h C:/MCC18/h/string.h usb/usb_common.h usb/usb_device.h USB/usb_descriptors.c USB/usb.h CONFIG/Compiler.h C:/MCC18/h/p18cxxx.h C:/MCC18/h/p18f4553.h C:/MCC18/h/stdarg.h C:/MCC18/h/stddef.h DEFINE/GenericTypeDefs.h USB/usb_config.h C:/MCC18/h/limits.h usb/usb_ch9.h usb/usb_hal.h USB/usb_hal_pic18.h USB/usb_function_hid.h
	$(CC) -p=18F4553 /i"C:\MCC18\h" "USB\usb_descriptors.c" -fo="./OBJECTS\usb_descriptors.o" -D__DEBUG -Ou- -Ot- -Ob- -Op- -Or- -Od- -Opa-

OBJECTS/PCRTask.o : PCR/PCRTask.c C:/MCC18/h/stdio.h C:/MCC18/h/stdlib.h C:/MCC18/h/string.h PCR/PCRTask.h C:/MCC18/h/math.h PCR/PCRTask.c HardwareProfile.h HardwareProfile\ -\ PICDEM\ FSUSB.h CONFIG/Compiler.h C:/MCC18/h/p18cxxx.h C:/MCC18/h/p18f4553.h C:/MCC18/h/stdarg.h C:/MCC18/h/stddef.h DEFINE/GenericTypeDefs.h DEFINE/UserDefs.h DEFINE/GlobalTypeVars.h PCR/Temp_Table.h
	$(CC) -p=18F4553 /i"C:\MCC18\h" "PCR\PCRTask.c" -fo="./OBJECTS\PCRTask.o" -D__DEBUG -Ou- -Ot- -Ob- -Op- -Or- -Od- -Opa-

OBJECTS/Usb_Task.o : ../150722_microPCR/USB/Usb_Task.c C:/MCC18/h/stdio.h C:/MCC18/h/stdlib.h C:/MCC18/h/string.h usb/usb_common.h usb/usb_device.h TOOLS/Delay.h ../150722_microPCR/USB/Usb_Task.c HardwareProfile.h HardwareProfile\ -\ PICDEM\ FSUSB.h DEFINE/UserDefs.h DEFINE/GenericTypeDefs.h C:/MCC18/h/stddef.h USB/usb.h CONFIG/Compiler.h C:/MCC18/h/p18cxxx.h C:/MCC18/h/p18f4553.h C:/MCC18/h/stdarg.h USB/usb_config.h C:/MCC18/h/limits.h usb/usb_ch9.h usb/usb_hal.h USB/usb_hal_pic18.h USB/usb_function_hid.h
	$(CC) -p=18F4553 /i"C:\MCC18\h" "D:\YJ\Embedded\2015_Project\uPCR\microPCR\150722_microPCR\USB\Usb_Task.c" -fo="./OBJECTS\Usb_Task.o" -D__DEBUG -Ou- -Ot- -Ob- -Op- -Or- -Od- -Opa-

OBJECTS/Temp_Table.o : PCR/Temp_Table.c C:/MCC18/h/stdio.h C:/MCC18/h/stdlib.h C:/MCC18/h/string.h usb/usb_common.h usb/usb_device.h PCR/Temp_Table.c PCR/Temp_Table.h USB/usb.h CONFIG/Compiler.h C:/MCC18/h/p18cxxx.h C:/MCC18/h/p18f4553.h C:/MCC18/h/stdarg.h C:/MCC18/h/stddef.h DEFINE/GenericTypeDefs.h USB/usb_config.h C:/MCC18/h/limits.h usb/usb_ch9.h usb/usb_hal.h USB/usb_hal_pic18.h
	$(CC) -p=18F4553 /i"C:\MCC18\h" "PCR\Temp_Table.c" -fo="./OBJECTS\Temp_Table.o" -D__DEBUG -Ou- -Ot- -Ob- -Op- -Or- -Od- -Opa-

clean : 
	$(RM) "OBJECTS\usb_device.o" "OBJECTS\usb_function_hid.o" "OBJECTS\Init.o" "OBJECTS\Pragma.o" "OBJECTS\GlobalTypeVars.o" "OBJECTS\Timer.o" "OBJECTS\main.o" "OBJECTS\Delay.o" "OBJECTS\usb_descriptors.o" "OBJECTS\PCRTask.o" "OBJECTS\Usb_Task.o" "OBJECTS\Temp_Table.o" ".\OUTPUT\microPCR.cof" ".\OUTPUT\microPCR.hex"

