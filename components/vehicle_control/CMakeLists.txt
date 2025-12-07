idf_component_register(
    SRCS "vehicle_control.c"
    INCLUDE_DIRS "."
    REQUIRES driver motor_control
)