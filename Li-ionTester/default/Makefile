################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL := cmd.exe
RM := rm -rf

USER_OBJS :=

LIBS := 
PROJ := 

O_SRCS := 
C_SRCS := 
S_SRCS := 
S_UPPER_SRCS := 
OBJ_SRCS := 
ASM_SRCS := 
PREPROCESSING_SRCS := 
OBJS := 
OBJS_AS_ARGS := 
C_DEPS := 
C_DEPS_AS_ARGS := 
EXECUTABLES := 
OUTPUT_FILE_PATH :=
OUTPUT_FILE_PATH_AS_ARGS :=
AVR_APP_PATH :=$$$AVR_APP_PATH$$$
QUOTE := "
ADDITIONAL_DEPENDENCIES:=
OUTPUT_FILE_DEP:=
LIB_DEP:=
LINKER_SCRIPT_DEP:=

# Every subdirectory with source files must be described here
SUBDIRS := 


# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS +=  \
../parser.c \
../Li-ionTester.c \
../lcd_lib_2.c \
../buttons.c \
../usart.c


PREPROCESSING_SRCS += 


ASM_SRCS += 


OBJS +=  \
parser.o \
Li-ionTester.o \
lcd_lib_2.o \
buttons.o \
usart.o

OBJS_AS_ARGS +=  \
parser.o \
Li-ionTester.o \
lcd_lib_2.o \
buttons.o \
usart.o

C_DEPS +=  \
parser.d \
Li-ionTester.d \
lcd_lib_2.d \
buttons.d \
usart.d

C_DEPS_AS_ARGS +=  \
parser.d \
Li-ionTester.d \
lcd_lib_2.d \
buttons.d \
usart.d

OUTPUT_FILE_PATH +=Li-ionTester.elf

OUTPUT_FILE_PATH_AS_ARGS +=Li-ionTester.elf

ADDITIONAL_DEPENDENCIES:=

OUTPUT_FILE_DEP:= ./makedep.mk

LIB_DEP+= 

LINKER_SCRIPT_DEP+= 


# AVR32/GNU C Compiler











./%.o: .././%.c
	@echo Building file: $<
	@echo Invoking: AVR/GNU C Compiler : 4.5.1
	$(QUOTE)C:\Program Files (x86)\Atmel\AVR Tools\AVR Toolchain\bin\avr-gcc.exe$(QUOTE)  -x c -funsigned-char -funsigned-bitfields -DF_CPU=8000000  -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -mmcu=atmega8 -c -gdwarf-2 -std=gnu99 -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"   -o "$@" "$<" 
	@echo Finished building: $<
	



# AVR32/GNU Preprocessing Assembler



# AVR32/GNU Assembler




ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: $(OUTPUT_FILE_PATH) $(ADDITIONAL_DEPENDENCIES)

$(OUTPUT_FILE_PATH): $(OBJS) $(USER_OBJS) $(OUTPUT_FILE_DEP) $(LIB_DEP) $(LINKER_SCRIPT_DEP)
	@echo Building target: $@
	@echo Invoking: AVR/GNU Linker : 4.5.1
	$(QUOTE)C:\Program Files (x86)\Atmel\AVR Tools\AVR Toolchain\bin\avr-gcc.exe$(QUOTE) -o$(OUTPUT_FILE_PATH_AS_ARGS) $(OBJS_AS_ARGS) $(USER_OBJS) $(LIBS) -Wl,-Map="Li-ionTester.map" -Wl,--start-group  -Wl,--end-group -Wl,--gc-sections -mmcu=atmega8  
	@echo Finished building target: $@
	"C:\Program Files (x86)\Atmel\AVR Tools\AVR Toolchain\bin\avr-objcopy.exe" -O ihex -R .eeprom -R .fuse -R .lock -R .signature -R .user_signatures  "Li-ionTester.elf" "Li-ionTester.hex"
	"C:\Program Files (x86)\Atmel\AVR Tools\AVR Toolchain\bin\avr-objcopy.exe" -j .eeprom  --set-section-flags=.eeprom=alloc,load --change-section-lma .eeprom=0  --no-change-warnings -O ihex "Li-ionTester.elf" "Li-ionTester.eep" || exit 0
	"C:\Program Files (x86)\Atmel\AVR Tools\AVR Toolchain\bin\avr-objdump.exe" -h -S "Li-ionTester.elf" > "Li-ionTester.lss"
	"C:\Program Files (x86)\Atmel\AVR Tools\AVR Toolchain\bin\avr-objcopy.exe" -O srec -R .eeprom -R .fuse -R .lock -R .signature -R .user_signatures "Li-ionTester.elf" "Li-ionTester.srec"
	"C:\Program Files (x86)\Atmel\AVR Tools\AVR Toolchain\bin\avr-size.exe" "Li-ionTester.elf"
	
	





# Other Targets
clean:
	-$(RM) $(OBJS_AS_ARGS) $(EXECUTABLES)  
	-$(RM) $(C_DEPS_AS_ARGS)   
	rm -rf "Li-ionTester.elf" "Li-ionTester.a" "Li-ionTester.hex" "Li-ionTester.lss" "Li-ionTester.eep" "Li-ionTester.map" "Li-ionTester.srec" "Li-ionTester.usersignatures"
	