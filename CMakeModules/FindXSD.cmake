# a script taken from http://www.codesynthesis.com/pipermail/xsd-users/2006-March/000269.html and heavily modified:
# Locate Xsd from code synthesis include paths and binary
# Xsd can be found at http://codesynthesis.com/products/xsd/
# Written by Frederic Heem, frederic.heem _at_ telsey.it
#
# This module defines
# XSD_INCLUDE_DIRS, where to find elements.hxx, etc.
# XSD_EXECUTABLE, where is the xsd compiler

# list common paths here:
FIND_PATH(XSD_INCLUDE_DIRS xsd/cxx/parser/elements.hxx
  /usr/local/include
  /usr/include
)

FIND_PROGRAM(XSD_EXECUTABLE 
  NAMES xsdcxx xsd
)

if (NOT XSD_INCLUDE_DIRS)
  message (FATAL_ERROR "Unable to find xsd include files (xsd/cxx/parser/elements.hxx)")
endif (NOT XSD_INCLUDE_DIRS)
if (NOT XSD_EXECUTABLE)
  message (FATAL_ERROR "Unable to find xsd or xsdcxx executable")
endif (NOT XSD_EXECUTABLE)


MARK_AS_ADVANCED(
    XSD_INCLUDE_DIRS
      XSD_EXECUTABLE
)
