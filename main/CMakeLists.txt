file(GLOB_RECURSE SOURCE *.cpp *.c)

idf_component_register(
    SRCS "${SOURCE}"
    INCLUDE_DIRS "."
)
