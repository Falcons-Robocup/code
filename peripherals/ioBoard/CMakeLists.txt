cmake_minimum_required(VERSION 2.8.11)
project(IoBoard C)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)

# The following folder will be included
include_directories("${PROJECT_SOURCE_DIR}/include")

set(project_elf_file ${PROJECT_NAME}.elf)
set(project_map_file ${PROJECT_NAME}.map)
set(project_hex_file ${PROJECT_NAME}.hex)
set(bootloader_hex_file ${CMAKE_SOURCE_DIR}/bootloader/xboot.hex)
set(hex_file ${PROJECT_NAME}_with_bootloader.hex)

set(AVR_MCU "atxmega64a3")

set(CMAKE_SYSTEM_PROCESSOR avr)

set(CMAKE_C_COMPILER avr-gcc)
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_FLAGS "-ffreestanding -fshort-enums -mmcu=${AVR_MCU}")

set(CMAKE_EXE_LINKER_FLAGS "-mmcu=${AVR_MCU} -Wl,--gc-sections -mrelax -Wl,-Map,${project_map_file}")

set(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS)

set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES "${project_map_file}")

add_executable(${project_elf_file} src/main.c
                           src/ADC.c
                           src/Charge_Pulse.c
                           src/Digital_Input.c
                           src/Package.c
                           src/Shoot_Level.c
                           src/Shoot_Pulse.c
                           src/UART.c)

add_custom_target(${project_hex_file}
                   COMMAND avr-objcopy -j .text -j .data -O ihex ${project_elf_file} ${project_hex_file}
                   COMMAND avr-size -C;--mcu=${AVR_MCU} ${project_elf_file}
                   WORKING_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
                   DEPENDS ${project_elf_file})

add_custom_target(bootloader
                  COMMAND make
                  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/bootloader)

add_custom_target(${hex_file}
                  srec_cat ${bootloader_hex_file} -intel ${project_hex_file} -intel -o ${hex_file} -intel
                  WORKING_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
                  DEPENDS bootloader ${project_hex_file})

add_custom_target(${PROJECT_NAME}
                  ALL
                  DEPENDS ${hex_file})

add_custom_target(upload
                  COMMAND avrdude -p ${AVR_MCU} -P usb -e -c avrispmkII -U flash:w:${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${hex_file}
                  DEPENDS ${hex_file}
                  COMMENT "Uploading ${hex_file} to ${AVR_MCU}.")

add_custom_target(upload_without_bootloader
                  COMMAND avrdude -p ${AVR_MCU} -P usb -e -c avrispmkII -U flash:w:${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${project_hex_file}
                  DEPENDS ${hex_file}
                  COMMENT "Uploading ${hex_file} to ${AVR_MCU}.")
