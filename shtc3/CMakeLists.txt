
# componet standalone mode
set(COMPONENT_SRCS "hts221.c")

set(COMPONENT_ADD_INCLUDEDIRS ". include")

# requirements can't depend on config
set(COMPONENT_REQUIRES i2c_bus)

register_component()
