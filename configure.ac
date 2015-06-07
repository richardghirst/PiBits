#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.

AC_PREREQ([2.69])
AC_INIT([pi-blaster], [0.1.1])
AC_CONFIG_SRCDIR([pi-blaster.c])
AC_CONFIG_HEADERS([config.h])
AM_INIT_AUTOMAKE

# Default folders
AC_PREFIX_DEFAULT([/usr])

test "$sysconfdir" = '${prefix}/etc' && sysconfdir=/etc
test "$sharedstatedir" = '${prefix}/com' && sharedstatedir=/var
test "$localstatedir" = '${prefix}/var' && localstatedir=/var

# Checks for systemd.
PKG_PROG_PKG_CONFIG
AC_ARG_WITH([systemdsystemunitdir],
 AS_HELP_STRING([--with-systemdsystemunitdir=DIR], [Directory for systemd service files]),
 [], [with_systemdsystemunitdir=$($PKG_CONFIG --variable=systemdsystemunitdir systemd)])
if test "x$with_systemdsystemunitdir" != xno; then
 AC_SUBST([systemdsystemunitdir], [$with_systemdsystemunitdir])
fi
AM_CONDITIONAL(HAVE_SYSTEMD, [test -n "$with_systemdsystemunitdir" -a "x$with_systemdsystemunitdir" != xno ])
AM_CONDITIONAL(CROSS_COMPILING, [test x"$cross_compiling" = x"yes" ])

# Checks for programs.
AC_PROG_CC
AC_PROG_INSTALL
AC_PROG_MAKE_SET

# Check for libraries.

# Check for header files.
AC_CHECK_HEADERS([fcntl.h stdint.h stdlib.h string.h sys/ioctl.h sys/time.h unistd.h])

# Checks for typedefs, structures, and compiler characteristics.
AC_CHECK_HEADER_STDBOOL
AC_TYPE_SIZE_T
AC_TYPE_UINT32_T
AC_TYPE_UINT8_T

# Checks for library functions.
AC_HEADER_MAJOR
AC_FUNC_MMAP
AC_CHECK_FUNCS([memset mkfifo munmap])

AC_CONFIG_FILES([Makefile])
AC_OUTPUT
