#------------------------------------------------------------------------------
# TARGET is the name of the output
# BUILD is the directory where object files & intermediate files will be placed
# SOURCES is a list of directories containing source code
# INCLUDES is a list of directories containing header files
#------------------------------------------------------------------------------
TARGET   := shape-opt
BUILD    := build
SOURCES  := src
INCLUDES := src

#------------------------------------------------------------------------------
# options for code generation
#------------------------------------------------------------------------------
CC  := g++
CXX := g++

# everything is relative to the build directory

CXXFLAGS := -Wall -Werror -pedantic -g \
            $(shell libpng12-config --cflags) \
			-I/usr/include/eigen2/ -Wno-long-long

LDFLAGS := $(shell libpng12-config --libs) \
           -lglut -lGLU -lGL -lX11

ifneq ($(BUILD),$(notdir $(CURDIR)))

export OUTPUT := $(TARGET)
export VPATH  := $(foreach dir,$(SOURCES),$(CURDIR)/$(dir))

export DEPSDIR := $(CURDIR)/$(BUILD)

#------------------------------------------------------------------------------
# automatically build a list of object files for our project
#------------------------------------------------------------------------------
CPPFILES := $(foreach dir,$(SOURCES),$(notdir $(wildcard $(dir)/*.cpp)))

export OFILES := $(CPPFILES:.cpp=.o)

#------------------------------------------------------------------------------
# build a list of include paths
#------------------------------------------------------------------------------
export INCLUDE := $(foreach dir,$(INCLUDES),-I$(CURDIR)/$(dir)) \
	 -I$(CURDIR)/$(BUILD)

.PHONY: $(BUILD) clean

$(BUILD):
	@[ -d $@ ] || mkdir -p $@
	@make --no-print-directory -C $(BUILD) -f $(CURDIR)/Makefile

all : $(BUILD)

clean:
	@echo clean ...
	@rm -fr $(BUILD) $(TARGET)

else

#------------------------------------------------------------------------------
# main targets
#------------------------------------------------------------------------------
$(OUTPUT) : $(OFILES)

endif
