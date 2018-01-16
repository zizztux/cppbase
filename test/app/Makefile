#
# Copyright (c) 2017-2018, SeungRyeol Lee
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# * Neither the name of the copyright holder nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#

#
# Directories and files
#
TOP_DIR := ../..
HDR_DIR := $(TOP_DIR)/include
OUTPUT_DIR := $(TOP_DIR)/test

OUTPUT := $(OUTPUT_DIR)/$(PROJECT).so.$(VERSION)
HDRS += $(addprefix $(HDR_DIR)/,cppbase.h)
SRCS += $(addprefix $(SRC_DIR)/template/,asm_template.S c_template.c cpp_template.cpp)
OBJS += $(patsubst $(SRC_DIR)/%.c,$(OUTPUT_DIR)/%.o,$(filter %.c,$(SRCS)))
OBJS += $(patsubst $(SRC_DIR)/%.cc,$(OUTPUT_DIR)/%.o,$(filter %.cc,$(SRCS)))
OBJS += $(patsubst $(SRC_DIR)/%.cpp,$(OUTPUT_DIR)/%.o,$(filter %.cpp,$(SRCS)))
OBJS += $(patsubst $(SRC_DIR)/%.S,$(OUTPUT_DIR)/%.o,$(filter %.S,$(SRCS)))

#
# Build options
#
CPPFLAGS := -I$(shell pwd)/$(HDR_DIR) -DVERSION=\"$(VERSION)\"
CFLAGS := -fPIC -Wall -O2 
CXXFLAGS := -std=c++1z -fPIC -Wall -O2 
LDFLAGS := -lstdc++fs 
ifeq ($(DEBUG),1)
CPPFLAGS +=
CFLAGS += -g
CXXFLAGS += -g
LDFLAGS += -g
endif

#
# Build targets
#
all: $(OUTPUT)

$(OUTPUT): $(OBJS)
	$(CXX) $(CXXFLAGS) $(CPPFLAGS) -o $@ $^ $(LDFLAGS)

include $(TOP_DIR)/build/linux/rules.mk

clean:

.PHONY: all clean
