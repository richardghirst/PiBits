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

# Checks for systemd
PKG_PROG_PKG_CONFIG
AC_ARG_WITH([systemdsystemunitdir],
 AS_HELP_STRING([--with-systemdsystemunitdir=DIR], [Directory for systemd service files]),
 [], [with_systemdsystemunitdir=$($PKG_CONFIG --variable=systemdsystemunitdir systemd)])
if test "x$with_systemdsystemunitdir" != xno; then
 AC_SUBST([systemdsystemunitdir], [$with_systemdsystemunitdir])
fi
AM_CONDITIONAL(HAVE_SYSTEMD, [test -n "$with_systemdsystemunitdir" -a "x$with_systemdsystemunitdir" != xno ])

# Checks for programs.
AC_PROG_CC

AC_CONFIG_FILES([Makefile])
AC_OUTPUT
