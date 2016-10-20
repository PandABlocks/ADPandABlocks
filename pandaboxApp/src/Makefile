TOP=../..

include $(TOP)/configure/CONFIG

#USR_CXXFLAGS += -Werror
USR_CXXFLAGS += -Wall -Wextra
USR_CXXFLAGS += -Wno-unused-parameter


# -------------------------------
# Build an Diamond Support Module
# -------------------------------

LIBRARY_IOC += pandabox

# xxxRecord.h will be created from xxxRecord.dbd
#DBDINC += xxx.h

# xxxRecord.dbd will be installed into <top>/dbd
#DBD += xxx.dbd
DBD += pandabox.dbd
pandabox_DBD += base.dbd
pandabox_DBD += asyn.dbd
pandabox_DBD += pandaboxsupport.dbd

# The following are compiled and added to the support library
#xxx_SRCS += xxxCodeA.c
pandabox_SRCS += pandabox.cpp

# We need to link against the EPICS Base libraries
#xxx_LIBS += $(EPICS_BASE_IOC_LIBS)

#INC += zebra2Regs.h
INC += pandabox.h


USR_INCLUDES += $(XML2_INCLUDE)

# ------------------------
# Build an IOC Application
# ------------------------

#PROD_IOC = pandabox
#
## pandabox.dbd will be installed into <top>/dbd
#DBD += pandabox.dbd
#
## pandabox.dbd will be created from these files
#pandabox_DBD += base.dbd
##pandabox_DBD += xxx.dbd
#
## pandabox_registerRecordDeviceDriver.cpp will be created
## pandabox.dbd
#pandabox_SRCS += pandabox_registerRecordDeviceDriver.cpp
#
## These two lines are needed for non-vxWorks builds, such as Linux
#pandabox_SRCS_DEFAULT += pandaboxMain.cpp
#pandabox_SRCS_vxWorks += -nil-
#
## Add locally compiled object code
##pandabox_SRCS +=
#
## The following adds object code from base/src/vxWorks
#pandabox_OBJS_vxWorks += $(EPICS_BASE_BIN)/vxComLibrary
#
## This line says that this IOC Application depends on the
## xxx Support Module
##pandabox_LIBS += xxx
#
## We need to link this IOC Application against the EPICS Base libraries
#pandabox_LIBS += $(EPICS_BASE_IOC_LIBS)

# ---------------------------------------------------

# NOTE: To build SNL programs, SNCSEQ must be defined
# in the <top>/configure/RELEASE file
#
# The difference between a file with an "st" extension
# and one with an "stt" extension is:
# The "st" extension files get passed through the C preprocessor
# whereas the "stt" files do not.

#ifneq ($(SNCSEQ),)
    # This builds sncExample as a component of pandabox
#    pandabox_SNCFLAGS += +r
#    pandabox_DBD      += sncExample.dbd
#    pandabox_SRCS     += sncExample.stt
#    pandabox_LIBS     += seq pv

    # The following builds sncExample as a standalone application
#    PROD_HOST           += sncExample
#    sncExample_SNCFLAGS += +m
#    sncExample_SRCS     += sncExample.stt
#    sncExample_LIBS     += seq pv
    # Here, we need to specify the dependency on EPICS Base libraries
    # again because this is a standalone application and
    # not part of pandabox
#    sncExample_LIBS     += $(EPICS_BASE_HOST_LIBS)
#endif

include $(TOP)/configure/RULES
