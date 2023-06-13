# This file can be used to set build configuration
# variables.  These variables are defined in a file called 
# "Makefile" that is located next to this one.

# For instructions on how to use this system, see
# https://github.com/Analog-Devices-MSDK/VSCode-Maxim/tree/develop#build-configuration

# **********************************************************

# Add your config here!

BOARD = CAM01_RevA

### Remove the camera definition to override where debayering.c comes from
#CAMERA = HM0360_COLOR
#ifneq ($(BOARD),CAM01_RevA)
#$(error ERR_NOTSUPPORTED: This project is only supported on the MAX78000CAM01 board.  (see https://analog-devices-msdk.github.io/msdk/USERGUIDE/#board-support-packages))
#endif

#Manually add the camera driver, omitting the debayering.c from the misc drivers
SRCS += hm0360_color.c
PROJ_CFLAGS+=-DCAMERA_BAYER
PROJ_CFLAGS+=-DCAMERA_HM0360_COLOR