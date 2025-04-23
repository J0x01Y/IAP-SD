cmake_minimum_required(VERSION 3.22)
project(FirmwareSuperBuild NONE)

include(ExternalProject)

set(TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/cmake/gcc-arm-none-eabi.cmake)
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

ExternalProject_Add(bootloader_proj
    SOURCE_DIR ${CMAKE_SOURCE_DIR}/bootloader
    DEPENDS mbedtls
    CMAKE_ARGS -DCMAKE_TOOLCHAIN_FILE=${TOOLCHAIN_FILE}
               -DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
               -DMBEDTLS_INSTALL_DIR=${MBEDTLS_INSTALL_DIR}
    BUILD_ALWAYS TRUE
    INSTALL_COMMAND ""
)

ExternalProject_Add(app_proj
    SOURCE_DIR ${CMAKE_SOURCE_DIR}/application
    DEPENDS mbedtls
    CMAKE_ARGS -DCMAKE_TOOLCHAIN_FILE=${TOOLCHAIN_FILE}
               -DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
               -DMBEDTLS_INSTALL_DIR=${MBEDTLS_INSTALL_DIR}
    BUILD_ALWAYS TRUE
    INSTALL_COMMAND ""
)