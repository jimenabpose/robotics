#
# Makefile for a Webots C++ controller
#
# if your controller uses several C++ sources files:
# CPP_SOURCES = my_controller.cpp my_clever_algo.cpp my_gui.cpp
# or
# CC_SOURCES = my_controller.cc my_clever_algo.cc my_gui.cc
#
# if your controller needs additional libraries:
# LIBRARIES=-L/path/to/my/library -lmy_library -lmy_other_library
#
# if some special CFLAGS need to be passed, for example to find include
# files, or set optimization level:
# CFLAGS=-O3 -I/my_library_path/include
#

ifndef WEBOTS_HOME_PATH
ifeq ($(OS),Windows_NT)
nullstring :=
space := $(nullstring) # a string containing a single space
WEBOTS_HOME_PATH=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))
else
WEBOTS_HOME_PATH=$(WEBOTS_HOME)
endif
endif

include $(WEBOTS_HOME_PATH)/projects/default/controllers/Makefile.include
