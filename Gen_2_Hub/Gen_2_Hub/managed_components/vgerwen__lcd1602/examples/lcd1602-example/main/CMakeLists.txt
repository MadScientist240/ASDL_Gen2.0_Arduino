
set(req lcd1602)

idf_component_register(SRCS "lcd1602-example.c"
                    INCLUDE_DIRS "."
                    REQUIRES ${req})
