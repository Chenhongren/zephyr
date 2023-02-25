# SPDX-License-Identifier: Apache-2.0

# Purpose of the generic.cmake is to define a generic C compiler which can be
# used for devicetree pre-processing and other pre-processing tasks which must
# be performed before the target can be determined.

# Todo: deprecate CLANG_ROOT_DIR
set_ifndef(LLVM_TOOLCHAIN_PATH "$ENV{CLANG_ROOT_DIR}")
zephyr_get(LLVM_TOOLCHAIN_PATH)

if(LLVM_TOOLCHAIN_PATH)
  set(TOOLCHAIN_HOME ${LLVM_TOOLCHAIN_PATH}/bin/)
endif()

set(LLVM_TOOLCHAIN_PATH ${CLANG_ROOT_DIR} CACHE PATH "clang install directory")

set(COMPILER clang)
set(BINTOOLS llvm)

if("${ARCH}" STREQUAL "arm")
  set(triple arm-none-eabi)
  set(CMAKE_EXE_LINKER_FLAGS_INIT "--specs=nosys.specs")
elseif("${ARCH}" STREQUAL "x86")
  if(CONFIG_64BIT)
    set(triple x86_64-pc-none-elf)
  else()
    set(triple i686-pc-none-elf)
  endif()
endif()

set(CMAKE_C_COMPILER_TARGET   ${triple})
set(CMAKE_ASM_COMPILER_TARGET ${triple})
set(CMAKE_CXX_COMPILER_TARGET ${triple})

set(TOOLCHAIN_HAS_NEWLIB OFF CACHE BOOL "True if toolchain supports newlib")

list(APPEND TOOLCHAIN_C_FLAGS --config ${ZEPHYR_BASE}/cmake/toolchain/llvm/clang.cfg)
list(APPEND TOOLCHAIN_LD_FLAGS --config ${ZEPHYR_BASE}/cmake/toolchain/llvm/clang.cfg)

message(STATUS "Found toolchain: host (clang/ld)")
