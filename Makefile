#
# This is a project Makefile. It is assumed the directory this Makefile resides in is a
# project subdirectory.
#

PROJECT_NAME := gatt_server_demos

COMPONENT_ADD_INCLUDEDIRS := components/include

EXTRA_COMPONENT_DIRS = $(IDF_PATH)/examples/common_components/led_strip

include $(IDF_PATH)/make/project.mk
