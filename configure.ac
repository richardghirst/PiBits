#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.

AC_PREREQ([2.69])
AC_INIT(pi-blaster, 0.1.0)
AC_CONFIG_SRCDIR([pi-blaster.c])
AC_CONFIG_HEADERS([config.h])
AM_INIT_AUTOMAKE

# Default folders
AC_PREFIX_DEFAULT(/usr)

test "$sysconfdir" = '${prefix}/etc' && sysconfdir=/etc
test "$sharedstatedir" = '${prefix}/com' && sharedstatedir=/var
test "$localstatedir" = '${prefix}/var' && localstatedir=/var

# Checks for programs.
AC_PROG_CC

AC_CONFIG_FILES([Makefile])
AC_OUTPUT
