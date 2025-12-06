idf_component_register(
    SRCS "ble_comm.c"
    INCLUDE_DIRS "."
    REQUIRES motor_control bt
)