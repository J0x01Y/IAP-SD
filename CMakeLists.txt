cmake_minimum_required(VERSION 3.22)
project(FirmwareSuperBuild NONE)

include(ExternalProject)

set(TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/cmake/gcc-arm-none-eabi.cmake)
include(${TOOLCHAIN_FILE})
set(MBEDTLS_INSTALL_DIR ${CMAKE_BINARY_DIR}/mbedtls-install)

ExternalProject_Add(mbedtls
    SOURCE_DIR ${CMAKE_SOURCE_DIR}/Middlewares/mbedTLS
    BINARY_DIR ${CMAKE_BINARY_DIR}/mbedtls-build
    INSTALL_DIR ${CMAKE_BINARY_DIR}/mbedtls-install
    CMAKE_ARGS -DCMAKE_TOOLCHAIN_FILE=${TOOLCHAIN_FILE}
               -DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
               -DCMAKE_INSTALL_PREFIX=${MBEDTLS_INSTALL_DIR}
    BUILD_ALWAYS TRUE
)

ExternalProject_Add(bootloader
    SOURCE_DIR ${CMAKE_SOURCE_DIR}/bootloader
    BINARY_DIR ${CMAKE_BINARY_DIR}/bootloader-build
    DEPENDS mbedtls
    BUILD_BYPRODUCTS ${CMAKE_BINARY_DIR}/bootloader-build/bootloader.elf
    CMAKE_ARGS -DCMAKE_TOOLCHAIN_FILE=${TOOLCHAIN_FILE}
               -DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
               -DMBEDTLS_INSTALL_DIR=${MBEDTLS_INSTALL_DIR}
    BUILD_ALWAYS TRUE
    INSTALL_COMMAND ""
)

ExternalProject_Add(application
    SOURCE_DIR ${CMAKE_SOURCE_DIR}/application
    BINARY_DIR ${CMAKE_BINARY_DIR}/application-build
    DEPENDS mbedtls
    BUILD_BYPRODUCTS ${CMAKE_BINARY_DIR}/application-build/application.elf
    CMAKE_ARGS -DCMAKE_TOOLCHAIN_FILE=${TOOLCHAIN_FILE}
               -DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
               -DMBEDTLS_INSTALL_DIR=${MBEDTLS_INSTALL_DIR}
    BUILD_ALWAYS TRUE
    INSTALL_COMMAND ""
)

set(CRYPTO_CHECK_OK ${CMAKE_BINARY_DIR}/compare_crypto.ok)

add_custom_command(
    OUTPUT ${CRYPTO_CHECK_OK}
    DEPENDS bootloader application
    COMMAND ${CMAKE_OBJCOPY} -O binary --only-section=.text.crypto bootloader-build/bootloader.elf boot.crypto
    COMMAND ${CMAKE_OBJCOPY} -O binary --only-section=.text.crypto application-build/application.elf app.crypto
    COMMAND ${TOOLCHAIN_PREFIX}objdump -d --section=.text.crypto bootloader-build/bootloader.elf > boot.crypto.asm
    COMMAND ${TOOLCHAIN_PREFIX}objdump -d --section=.text.crypto application-build/application.elf > app.crypto.asm
    COMMAND bash ${CMAKE_SOURCE_DIR}/Tools/compare_crypto.sh
        boot.crypto
        app.crypto
        ${CRYPTO_CHECK_OK}
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)
add_custom_target(verify_crypto ALL DEPENDS ${CRYPTO_CHECK_OK})

# set(APP_BIN "${CMAKE_BINARY_DIR}/application_stripped.bin")
# set(SIGNED_FW "${CMAKE_BINARY_DIR}/firmware.fw")

# add_custom_command(
#   OUTPUT ${APP_BIN}
#   COMMAND ${CMAKE_OBJCOPY}
#     -O binary
#     --remove-section=.text.crypto
#     --remove-section=.rodata.crypto
#     application.elf ${APP_BIN}
#   DEPENDS application
#   COMMENT "🔧 Strip crypto and export ${APP_BIN}"
# )

# add_custom_command(
#   OUTPUT ${SIGNED_FW}
#   COMMAND python3 ${CMAKE_SOURCE_DIR}/tools/sign_firmware.py ${APP_BIN} ${SIGNED_FW}
#   DEPENDS ${APP_BIN}
#   COMMENT "🔏 Signing firmware -> ${SIGNED_FW}"
# )

# add_custom_target(firmware_package ALL DEPENDS ${SIGNED_FW})