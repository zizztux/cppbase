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

.SUFFIXES: .c .cc .cpp .d .o

$(OUTPUT_DIR)/%.d: $(SRC_DIR)/%.c
	@$(MKDIR_P) $(dir $@)
	@$(CC) $(CFLAGS) $(CPPFLAGS) -MM -MP -MT "$(@:.d=.o) $@" -MF $@ $<

$(OUTPUT_DIR)/%.d: $(SRC_DIR)/%.cc
	@$(MKDIR_P) $(dir $@)
	@$(CXX) $(CXXFLAGS) $(CPPFLAGS) -MM -MP -MT "$(@:.d=.o) $@" -MF $@ $<

$(OUTPUT_DIR)/%.d: $(SRC_DIR)/%.cpp
	@$(MKDIR_P) $(dir $@)
	@$(CXX) $(CXXFLAGS) $(CPPFLAGS) -MM -MP -MT "$(@:.d=.o) $@" -MF $@ $<

$(OUTPUT_DIR)/%.o: $(SRC_DIR)/%.c
	$(CC) $(CFLAGS) $(CPPFLAGS) -c -o $@ $<

$(OUTPUT_DIR)/%.o: $(SRC_DIR)/%.cc
	$(CXX) $(CXXFLAGS) $(CPPFLAGS) -c -o $@ $<

$(OUTPUT_DIR)/%.o: $(SRC_DIR)/%.cpp
	$(CXX) $(CXXFLAGS) $(CPPFLAGS) -c -o $@ $<

-include $(patsubst $(SRC_DIR)/%.c,$(OUTPUT_DIR)/%.d,$(filter %.c,$(SRCS)))
-include $(patsubst $(SRC_DIR)/%.cc,$(OUTPUT_DIR)/%.d,$(filter %.cc,$(SRCS)))
-include $(patsubst $(SRC_DIR)/%.cpp,$(OUTPUT_DIR)/%.d,$(filter %.cpp,$(SRCS)))