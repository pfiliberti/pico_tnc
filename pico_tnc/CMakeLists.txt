add_executable(pico_tnc
        main.c
        tnc.c
        send.c
        receive.c
        decode.c
        filter.c
        bell202.c
        hf_demod.c
        ax25.c
        usb_input.c
        usb_output.c
        serial.c
        tty.c
        flash.c
        kiss.c
)

pico_generate_pio_header(pico_tnc ${CMAKE_CURRENT_LIST_DIR}/pio_dac.pio)

# === Embed the ROM binary using objcopy ===
set(ROM_SRC ${CMAKE_SOURCE_DIR}/roms/hk21rom.bin)
set(ROM_OBJ ${CMAKE_CURRENT_BINARY_DIR}/tncrom.o)

# Step 1: Generate ROM object from binary
add_custom_command(
  OUTPUT ${ROM_OBJ}
  # Copy the ROM binary to the build dir with a simple name
  COMMAND ${CMAKE_COMMAND} -E copy ${ROM_SRC} hk21rom.bin
  # Run objcopy to convert it to an object file
  COMMAND ${CMAKE_OBJCOPY} -I binary -O elf32-littlearm -B arm hk21rom.bin ${ROM_OBJ}
  DEPENDS ${ROM_SRC}
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
  COMMENT "Embedding ROM binary as object file"
)

# Step 2: Declare custom target for ROM object
add_custom_target(rom_obj ALL DEPENDS ${ROM_OBJ})

# Step 3: Make sure pico_tnc depends on the rom_obj target
add_dependencies(pico_tnc rom_obj)

# Step 4: Include the binary object file in the link
target_sources(pico_tnc PRIVATE ${ROM_OBJ})

# === Includes ===
target_include_directories(pico_tnc PRIVATE
    include
    ${CMAKE_CURRENT_SOURCE_DIR}/z80emu
)

# === Link libraries ===
target_link_libraries(pico_tnc
    pico_stdlib
    hardware_adc
    hardware_dma
    hardware_pwm
    hardware_uart
    hardware_pio
    hardware_rtc
    z80emu
)

pico_enable_stdio_usb(pico_tnc 1)
pico_enable_stdio_uart(pico_tnc 1)

# create map/bin/hex file etc.
pico_add_extra_outputs(pico_tnc)

# add url via pico_set_program_url
example_auto_set_url(pico_tnc)
