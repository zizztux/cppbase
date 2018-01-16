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
# Package information
#
AC_INIT([C++ Base Library],
	[0.0.1],
	[zizztux.lee@gmail.com],
	[libcppbase],
	[https://github.com/zizztux/libcppbase])

#
# Checks for system
#
AC_CONFIG_SRCDIR([../../include/cppbase.h])
AC_CONFIG_AUX_DIR([scripts])
AC_CANONICAL_BUILD
AC_CANONICAL_HOST
AC_CANONICAL_TARGET

: ${CFLAGS=""}
: ${CXXFLAGS=""}

#
# Checks for programs
#
AC_PROG_CC
AC_PROG_CPP
AC_PROG_CXX
AC_PROG_INSTALL
AC_PROG_LN_S
AC_PROG_MKDIR_P
AC_PROG_RANLIB
AC_CHECK_TOOL([AR], [ar])
AC_CHECK_TOOL([AS], [as])
AC_CHECK_TOOL([LD], [ld])
AC_CHECK_TOOL([NM], [nm])
AC_CHECK_TOOL([OBJCOPY], [objcopy])
AC_CHECK_TOOL([OBJDUMP], [objdump])
AC_CHECK_TOOL([STRIP], [strip])
AC_PATH_PROG([LDCONFIG], [ldconfig], , [$PATH:/sbin:/usr/sbin])
AC_HEADER_STDC

is_cross_compiler=
if test x"${host}" = x"${target}" ; then
  is_cross_compiler=no
else
  is_cross_compiler=yes
fi

#
# Checks for build environment
#
AC_LANG([C++])

#
# Substitution for toolchain flags
#
AC_SUBST([ARFLAGS])
AC_SUBST([ASFLAGS])
AC_SUBST([CFLAGS])
AC_SUBST([CPPFLAGS])
AC_SUBST([CXXFLAGS])
AC_SUBST([LDFLAGS])

#
# Output
#
AC_CONFIG_HEADERS([config.h])
AC_CONFIG_FILES([Makefile])
AC_OUTPUT
