# cmake/arm-gcc.cmake - toolchain file for the G2FD firmware (bare-metal ARM)
#
# Usage (from the repository root):
#   cmake -B gcc/build -G "MinGW Makefiles" -DCMAKE_MAKE_PROGRAM=<make>
#   cmake --build gcc/build
#
# arm-none-eabi-gcc must be on PATH, or set ARM_GCC_TOOLCHAIN_DIR to the
# toolchain 'bin' directory before configuring:
#   cmake -B gcc/build -DARM_GCC_TOOLCHAIN_DIR=C:/path/to/gnu-tools/bin ...

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(ARM_GCC_TOOLCHAIN_DIR "" CACHE PATH "Directory containing arm-none-eabi-* executables (optional)")

if(ARM_GCC_TOOLCHAIN_DIR)
    set(ARM_GCC_PREFIX "${ARM_GCC_TOOLCHAIN_DIR}/arm-none-eabi-")
else()
    set(ARM_GCC_PREFIX "arm-none-eabi-")
endif()

find_program(CMAKE_C_COMPILER "${ARM_GCC_PREFIX}gcc" REQUIRED)
find_program(CMAKE_ASM_COMPILER "${ARM_GCC_PREFIX}gcc" REQUIRED)
find_program(CMAKE_OBJCOPY "${ARM_GCC_PREFIX}objcopy")
find_program(CMAKE_SIZE "${ARM_GCC_PREFIX}size")

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# Skip CMake's compiler detection / ABI try-compiles: bare-metal cross
# compilers always fail the hosted-compiler sanity checks. All required
# flags are provided explicitly by CMakeLists.txt instead.
set(CMAKE_C_COMPILER_FORCED TRUE)
set(CMAKE_CXX_COMPILER_FORCED TRUE)
set(CMAKE_ASM_COMPILER_FORCED TRUE)

# Bare-metal: no OS syscalls. nosys.specs provides empty stubs so that CMake's
# compiler ABI check (which links a test executable) succeeds.
# Note: give it to the linker only -- adding it to compile flags too would
# load the spec file twice ("attempt to rename spec" error).
set(CMAKE_EXE_LINKER_FLAGS_INIT "--specs=nosys.specs")

# Compile .s (GNU as syntax) through gcc with the same CPU flags as C code
set(CMAKE_ASM_FLAGS_INIT "-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard -x assembler-with-cpp")
