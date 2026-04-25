idf_component_register(
    SRCS "control_task.c" "main.c"
    INCLUDE_DIRS "."
    REQUIRES freertos log
)
