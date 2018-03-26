TOP=../..

include $(TOP)/configure/CONFIG

#USR_CXXFLAGS += -Werror
USR_CXXFLAGS += -Wall -Wextra
USR_CXXFLAGS += -Wno-unused-parameter


# -------------------------------
# Build an Diamond Support Module
# -------------------------------

LIBRARY_IOC += ADPandABlocks

# xxxRecord.h will be created from xxxRecord.dbd
#DBDINC += xxx.h

# xxxRecord.dbd will be installed into <top>/dbd
#DBD += xxx.dbd
DBD += ADPandABlocks.dbd
#DBD += ADPandABlocksMotorSync.dbd
ADPandABlocks_DBD += base.dbd
ADPandABlocks_DBD += asyn.dbd
ADPandABlocks_DBD += ADPandABlockssupport.dbd

# The following are compiled and added to the support library
#xxx_SRCS += xxxCodeA.c
ADPandABlocks_SRCS += ADPandABlocks.cpp

# We need to link against the EPICS Base libraries
#xxx_LIBS += $(EPICS_BASE_IOC_LIBS)

#INC += zebra2Regs.h
INC += ADPandABlocks.h


USR_INCLUDES += $(XML2_INCLUDE)

# ------------------------
# Build an IOC Application
# ------------------------

#PROD_IOC = ADPandABlocks
#
## ADPandABlocks.dbd will be installed into <top>/dbd
#DBD += ADPandABlocks.dbd
#
## ADPandABlocks.dbd will be created from these files
#ADPandABlocks_DBD += base.dbd
##ADPandABlocks_DBD += xxx.dbd
#
## ADPandABlocks_registerRecordDeviceDriver.cpp will be created
## ADPandABlocks.dbd
#ADPandABlocks_SRCS += ADPandABlocks_registerRecordDeviceDriver.cpp
#
## These two lines are needed for non-vxWorks builds, such as Linux
#ADPandABlocks_SRCS_DEFAULT += ADPandABlocksMain.cpp
#ADPandABlocks_SRCS_vxWorks += -nil-
#
## Add locally compiled object code
##ADPandABlocks_SRCS +=
#
## The following adds object code from base/src/vxWorks
#ADPandABlocks_OBJS_vxWorks += $(EPICS_BASE_BIN)/vxComLibrary
#
## This line says that this IOC Application depends on the
## xxx Support Module
##ADPandABlocks_LIBS += xxx
#
## We need to link this IOC Application against the EPICS Base libraries
#ADPandABlocks_LIBS += $(EPICS_BASE_IOC_LIBS)

# ---------------------------------------------------

# NOTE: To build SNL programs, SNCSEQ must be defined
# in the <top>/configure/RELEASE file
#
# The difference between a file with an "st" extension
# and one with an "stt" extension is:
# The "st" extension files get passed through the C preprocessor
# whereas the "stt" files do not.

#ifneq ($(SNCSEQ),)
    # This builds sncExample as a component of ADPandABlocks
#    ADPandABlocks_SNCFLAGS += +r
#    ADPandABlocks_DBD      += sncExample.dbd
#    ADPandABlocks_SRCS     += sncExample.stt
#    ADPandABlocks_LIBS     += seq pv

    # The following builds sncExample as a standalone application
#    PROD_HOST           += sncExample
#    sncExample_SNCFLAGS += +m
#    sncExample_SRCS     += sncExample.stt
#    sncExample_LIBS     += seq pv
    # Here, we need to specify the dependency on EPICS Base libraries
    # again because this is a standalone application and
    # not part of ADPandABlocks
#    sncExample_LIBS     += $(EPICS_BASE_HOST_LIBS)
#endif

include $(TOP)/configure/RULES
