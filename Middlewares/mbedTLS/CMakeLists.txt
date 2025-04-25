cmake_minimum_required(VERSION 3.22)
project(mbedTLS)

set(mbedTLS_Src
    ${CMAKE_SOURCE_DIR}/library/asn1parse.c
    ${CMAKE_SOURCE_DIR}/library/asn1write.c
    ${CMAKE_SOURCE_DIR}/library/base64.c
    ${CMAKE_SOURCE_DIR}/library/bignum.c
    ${CMAKE_SOURCE_DIR}/library/camellia.c
    ${CMAKE_SOURCE_DIR}/library/certs.c
    ${CMAKE_SOURCE_DIR}/library/cipher.c
    ${CMAKE_SOURCE_DIR}/library/cipher_wrap.c
    ${CMAKE_SOURCE_DIR}/library/dhm.c
    ${CMAKE_SOURCE_DIR}/library/ecdsa.c
    ${CMAKE_SOURCE_DIR}/library/ecp.c
    ${CMAKE_SOURCE_DIR}/library/ecp_curves.c
    ${CMAKE_SOURCE_DIR}/library/entropy_poll.c
    ${CMAKE_SOURCE_DIR}/library/md.c
    ${CMAKE_SOURCE_DIR}/library/md_wrap.c
    ${CMAKE_SOURCE_DIR}/library/memory_buffer_alloc.c
    ${CMAKE_SOURCE_DIR}/library/oid.c
    ${CMAKE_SOURCE_DIR}/library/pem.c
    ${CMAKE_SOURCE_DIR}/library/pk.c
    ${CMAKE_SOURCE_DIR}/library/pkparse.c
    ${CMAKE_SOURCE_DIR}/library/pk_wrap.c
    ${CMAKE_SOURCE_DIR}/library/platform.c
    ${CMAKE_SOURCE_DIR}/library/platform_util.c
    ${CMAKE_SOURCE_DIR}/library/rsa_internal.c
    ${CMAKE_SOURCE_DIR}/library/sha256.c
    ${CMAKE_SOURCE_DIR}/library/sha512.c
    ${CMAKE_SOURCE_DIR}/library/ssl_ciphersuites.c
    ${CMAKE_SOURCE_DIR}/library/version_features.c
    ${CMAKE_SOURCE_DIR}/library/x509.c
    ${CMAKE_SOURCE_DIR}/library/x509_crt.c
    ${CMAKE_SOURCE_DIR}/custom/libc.c
)

add_library(mbedTLS STATIC)

target_sources(mbedTLS PRIVATE ${mbedTLS_Src})
target_compile_definitions(mbedTLS PUBLIC MBEDTLS_CONFIG_FILE=<mbedtls_config.h> MBEDTLS_LIB_BUILD )
target_compile_options(mbedTLS PRIVATE
    -ffunction-sections
    -fdata-sections
)

target_include_directories(mbedTLS PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/mbedtls>
    $<INSTALL_INTERFACE:include>
    $<INSTALL_INTERFACE:include/mbedtls>
)
install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
  DESTINATION include
)

install(
  TARGETS mbedTLS
  EXPORT mbedTLSConfig
  ARCHIVE DESTINATION lib
  INCLUDES DESTINATION include
)

install(
  EXPORT mbedTLSConfig
  NAMESPACE mbedtls::
  DESTINATION lib/cmake/mbedTLS
)
