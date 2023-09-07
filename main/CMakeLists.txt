file(
    GLOB_RECURSE files
        *.h
        *.cpp
)

set(dependencies freertos esp_system arduino-esp32 ArduinoJson expected fmt esp-nimble-cpp espchrono)

idf_component_register(
    SRCS
        ${files}
    INCLUDE_DIRS
        .
    REQUIRES
        ${dependencies}
)

target_compile_options(${COMPONENT_TARGET}
    PRIVATE
        -fstack-reuse=all
        -fstack-protector-all
        -fdiagnostics-color=always
        -Wno-unused-function
        -Wno-deprecated-declarations
        -Wno-missing-field-initializers
        -Wno-parentheses
)
