#
# This is a project Makefile. It is assumed the directory this Makefile resides in is a
# project subdirectory.
#

PROJECT_NAME := Sensornode

EXCLUDE_COMPONENTS := vfs ulp wear_levelling wifi_provisioning wpa_supplicant tcpip_adapter tcp_transport sdmmc openssl micro-ecc mdns lwip json jsmn expat fatfs ethernet esp-tls

include $(IDF_PATH)/make/project.mk

