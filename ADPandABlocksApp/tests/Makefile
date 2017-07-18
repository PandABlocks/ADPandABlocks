TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#USR_CXXFLAGS += -Werror
USR_CXXFLAGS += -Wall -Wextra
USR_CXXFLAGS += -Wno-unused-parameter

PROD_IOC += tests
tests_SRCS += tests.cpp
tests_SRCS += test_ADPandABlocks.cpp
tests_SRCS += testingutilities.cpp

tests_LIBS += boost_unit_test_framework
boost_unit_test_framework_DIR=$(BOOST_LIB)

PROD_LIBS += NDPlugin asyn ADPandABlocks

CXXFLAGS += -std=c++0x -pthread

USR_INCLUDES += $(XML2_INCLUDE)

include $(TOP)/configure/RULES
