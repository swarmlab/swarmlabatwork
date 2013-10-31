AC_PREREQ(2.61)
AC_INIT(libgptk, 0.2)
AC_CONFIG_MACRO_DIR([m4])

AM_INIT_AUTOMAKE(libgptk,0.2)
AM_PROG_LIBTOOL

# Custom itpp-config location
ITPP_CONFIG=""
itpp_config_path=""
AC_ARG_WITH([itpp-config],
            AS_HELP_STRING([--with-itpp-config=DIR],
                           [specify the path to the itpp-config script]),
            [itpp_config_path=$withval])            

# Checks for programs.
AC_PROG_LIBTOOL
AC_PROG_CC
AC_PROG_CXX
AC_PROG_INSTALL

# If no custom path specify, assume itpp-config is in the path
if test -z "$itpp_config_path"; then
  ITPP_CONFIG="itpp-config"
else
  ITPP_CONFIG="$itpp_config_path/itpp-config"
fi

# Check that itpp-config is found in the specified path (if no path, use default)
if test -z $itpp_config_path; then
    AC_CHECK_PROG(ITPP_CONFIG_FOUND, "itpp-config", "yes", "no")
else
    AC_CHECK_PROG(ITPP_CONFIG_FOUND, "itpp-config", "yes", "no", $itpp_config_path)
fi 

# If itpp-config not found, abort with error message
if test "$ITPP_CONFIG_FOUND" = "no"; then
    AC_MSG_ERROR(["itpp-config script not found. Make sure you have installed the IT++ library. If the itpp-config script is not in your path, please use "configure --with-itpp-config=DIR" where DIR is the folder containing itpp-config. For further information, see installation instructions in the doc/ folder."])
    exit 1
fi

# Checks for libraries.


# Set itpp flags and libraries
CPPFLAGS=`${ITPP_CONFIG} --cflags` $CPPFLAGS
LIBS=`${ITPP_CONFIG} --libs` 

# Checks for header files.

# Checks for typedefs, structures, and compiler characteristics.

# Checks for library functions.

AC_SUBST(CPPFLAGS)
AC_SUBST(LIBS)

AC_CONFIG_SUBDIRS([src/gaussian_processes 
                   src/covariance_functions
                   src/design
                   src/parameter_transforms 
                   src/optimisation
                   src/likelihood_models 
                   src/io 
                   src/itppext
                   src/plotting
                   tests
                   examples])
                   
AC_CONFIG_FILES([Makefile 
                 src/Makefile 
                 src/gaussian_processes/Makefile 
                 src/covariance_functions/Makefile
                 src/design/Makefile
                 src/parameter_transforms/Makefile 
                 src/optimisation/Makefile
                 src/likelihood_models/Makefile
                 src/io/Makefile
                 src/itppext/Makefile
                 src/plotting/Makefile
                 tests/Makefile
                 examples/Makefile])
AC_OUTPUT
