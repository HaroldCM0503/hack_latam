idf_component_register(
    SRCS        "main.c"
    INCLUDE_DIRS "."
    REQUIRES    nvs_flash esp_wifi esp_event esp_netif lwip
)

# Allow setting the board's RX_ID from the idf.py command line, e.g.
#   idf.py build -DRX_ID=2 flash
# Without this hook, CMake reports "Manually-specified variables were not
# used by the project: RX_ID" and the firmware silently falls back to the
# default `#define RX_ID 1` in main.c.
if(DEFINED RX_ID)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE "RX_ID=${RX_ID}")
endif()
