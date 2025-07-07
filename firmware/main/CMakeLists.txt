file(GLOB_RECURSE SOURCE *.cpp *.c)

idf_component_register(
    SRCS "main.c" "${SOURCE}"
    INCLUDE_DIRS "."
)
