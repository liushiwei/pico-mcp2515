# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(pico-mcp2515 C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# 设置目标文件名
set(EMBED_INPUT data.txt)
set(EMBED_OUTPUT text_data.o)

# 先复制源文件到构建目录
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/assets/data.txt
  ${CMAKE_CURRENT_BINARY_DIR}/data.txt
  COPYONLY
)

# Step 2: objcopy 嵌入生成原始对象文件
add_custom_command(
  OUTPUT data_raw.o
  COMMAND ${CMAKE_OBJCOPY}
    -I binary -O elf32-littlearm -B arm
    ${CMAKE_CURRENT_BINARY_DIR}/data.txt
    ${CMAKE_CURRENT_BINARY_DIR}/data_raw.o
  DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/data.txt
)

# Step 3: 重命名符号（防止路径污染）arm-none-eabi-nm text_data.o 查看符号名
add_custom_command(
  OUTPUT text_data.o
  COMMAND ${CMAKE_OBJCOPY}
    --redefine-sym _binary_D__work_MCU_RP2040_pico_mcp2515_build_data_txt_start=_binary_data_txt_start
    --redefine-sym _binary_D__work_MCU_RP2040_pico_mcp2515_build_data_txt_end=_binary_data_txt_end
    --redefine-sym _binary_D__work_MCU_RP2040_pico_mcp2515_build_data_txt_size=_binary_data_txt_size
    ${CMAKE_CURRENT_BINARY_DIR}/data_raw.o
    ${CMAKE_CURRENT_BINARY_DIR}/text_data.o
  DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/data_raw.o
  COMMENT "Fixing polluted symbols for data.txt"
)

# Add executable. Default name is the project name, version 0.1
add_executable(pico-mcp2515
    # Library executables
    include/mcp2515/mcp2515.cpp

    # Main executalbes
    src/pico-mcp2515.cpp
    ${CMAKE_CURRENT_BINARY_DIR}/text_data.o
)

# Add MCP2515 Lib
target_include_directories(pico-mcp2515 PUBLIC include/)

pico_set_program_name(pico-mcp2515 "pico-mcp2515")
pico_set_program_version(pico-mcp2515 "0.1")

pico_enable_stdio_uart(pico-mcp2515 1)
pico_enable_stdio_usb(pico-mcp2515 0)

# Add any user requested libraries
target_link_libraries(pico-mcp2515
    pico_stdlib
    hardware_spi
)

pico_add_extra_outputs(pico-mcp2515)

add_custom_command(TARGET pico-mcp2515 POST_BUILD
  COMMAND ${CMAKE_NM} -C $<TARGET_FILE:pico-mcp2515> | grep _binary_
  COMMENT "Listing embedded symbols"
)