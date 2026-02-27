idf_component_register(
    SRCS 
        "radio_link.cpp"
    INCLUDE_DIRS 
        "include"
    REQUIRES 
        rc_protocol   # It needs access to your shared structs
        driver
        esp_timer
)