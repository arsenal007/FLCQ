GPUTILS_DIR=C:\Program Files (x86)\gputils
SDCC_DIR=C:\Program Files\SDCC

PROJECT=FLCQ

OBJECTS=main.o eeprom.o timer0.o
OUTPUT=${FLCQ}.hex
PROCESSOR=16f628a
SCRIPT=16f628a_g.lkr
PROCESSOR_LIB="${SDCC_DIR}\non-free\lib\pic14\pic16f628a.lib"
SDCC_LIB="${SDCC_DIR}\lib\pic14\libsdcc.lib"
LINKER="${GPUTILS_DIR}\bin\gplink.exe"
AS="${GPUTILS_DIR}\bin\gpasm.exe"
CC="${SDCC_DIR}\bin\sdcc.exe"


all:$(OUTPUT)

$(OUTPUT):	$(OBJECTS) $(SCRIPT)
	 ${LINKER} --optimize-pagesel 3 --optimize s -S2 --map -c -s $(SCRIPT) -o ${PROJECT}.hex $(OBJECTS) ${SDCC_LIB} ${PROCESSOR_LIB}

%.o: %.asm
	${AS} -c $<
	
%.o: %.c
	${CC} -mpic14 -p${PROCESSOR} -V -c --use-non-free --opt-code-size --nostdlibcall $<

clean:
	rm -f eeprom.o timer0.asm main.o main.asm main.lst eeprom.lst ${PROJECT}.lst ${PROJECT}.map ${PROJECT}.hex ${PROJECT}.cod ${PROJECT}.cof .cod .cof .hex .lst .map

