# A compiled shader (SPIR-V) as a C array of 32-bit words, so the Vulkan
# engine carries its shaders inside its library (roadmap item 28, way B) and
# needs no file at run time. Usage:
#   cmake -DINPUT=x.spv -DOUTPUT=x.inc -DNAME=kXSpv -P embed_spirv.cmake
file(READ "${INPUT}" hex HEX)
string(LENGTH "${hex}" hex_length)
math(EXPR remainder "${hex_length} % 8")
if(NOT remainder EQUAL 0)
  message(FATAL_ERROR "${INPUT} is not SPIR-V: ${hex_length} hex digits")
endif()
# SPIR-V words are little-endian in the file: "aabbccdd" is 0xddccbbaa.
string(REGEX REPLACE "(..)(..)(..)(..)" "0x\\4\\3\\2\\1," words "${hex}")
math(EXPR word_count "${hex_length} / 8")
file(WRITE "${OUTPUT}"
     "// Generated from ${INPUT} by embed_spirv.cmake; do not edit.\n"
     "static const unsigned int ${NAME}[${word_count}] = {${words}};\n")
