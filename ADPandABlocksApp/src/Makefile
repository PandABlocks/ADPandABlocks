TOP=../..

include $(TOP)/configure/CONFIG

#USR_CXXFLAGS += -Werror
USR_CXXFLAGS += -Wall -Wextra
USR_CXXFLAGS += -Wno-unused-parameter
USR_CXXFLAGS += -std=c++11


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


include $(TOP)/configure/RULES
