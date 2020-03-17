#
# Copyright (c) 2017-2020, SeungRyeol Lee
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
# External dependencies
#
LIBCPPBASE_DIR := /home/zizztux/workspace/library/libcppbase/build/out

#
# Directories and files
#
HDRS := $(wildcard *.h *.hpp)
SRCS := $(wildcard *.c *.cc *.cpp)

#
# Build options
#
CPPFLAGS := -I$(LIBCPPBASE_DIR)/include
CXXFLAGS := -std=c++1z -Wall -O2
LDFLAGS := -lcppbase -L$(LIBCPPBASE_DIR)/lib
ifeq ($(DEBUG),0)
CPPFLAGS += -DNDEBUG
CXXFLAGS +=
else
CPPFLAGS +=
CXXFLAGS += -g
endif

#
# Build targets
#
all: app

app: $(SRCS) $(HDRS)
	$(CXX) $(CXXFLAGS) $(CPPFLAGS) -o $@ $(SRCS) $(LDFLAGS)

clean:
	\rm -f *.o app

.PHONY: all clean
