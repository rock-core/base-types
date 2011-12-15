# Provisions for adding python bindings
# 1. Looking for python interpreter, python lib/includes, numpy includes, 
#    python install path and boost-python
# 2. Adding the required include pathes.
# 3. Include ListMacros.
# 4. Provide a macro for building and installing a python binding library and a
#    relate package
include(FindPythonInterp)
include(FindPythonLibs)
include(FindNumpy)
include(PythonInstallPath)
find_package(Boost COMPONENTS python)

#todo make query for findings and throw errors if not.

INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${NUMPY_INCLUDE_DIR})

include(ListMacros)

# Adds a library with python bindings
# Assums using boost-python to generat the binding and to use additional python scripts
# to facilitate the library. It builds a shared library using boost-python and install
# it into the same path as the python package. The python package is installed using
# python's distutils assuming that the package provides a setup.py script.
# add_python_library( TARGET_NAME
#   SOURCES ...
#   PYSCRIPTS ...
#   DEPS ...
#   PREFIX .
#   PYDIR .
#   NOINSTALL
#   NOPRE )
# SOURCES gives the c/c++ files to use for the shared library.
# PYSCRIPTS give the python file the package depends on.
# DEPS are libraried besides boost-python the bindings are linked to.
# PYDIR ist the directory where package python files are located.
# NOINSTALL skips the installation.
# Normally the shared library is named _${TARGET_NAME} but with NOPRE it is just named
# ${TARGET_NAME}
macro(add_python_library TARGET_NAME)
    list_keys_split(${TARGET_NAME} "SOURCES;PYSCRIPTS;DEPS;PREFIX;PYDIR;NOINSTALL;NOPRE" ${ARGN})
   
    # Command to build the python package script wise
    ADD_CUSTOM_COMMAND(OUTPUT ${TARGET_NAME}_PyBuild
        COMMAND python ${${TARGET_NAME}_PYDIR}/setup.py 
        ARGS build 
        WORKING_DIRECTORY ${${TARGET_NAME}_PYDIR}
        DEPENDS ${${TARGET_NAME}_PYSCRIPTS}
        COMMENT "Running python setup.py build")
    
    if(${${TARGET_NAME}_NOPRE_FOUND})
        set(LIB_TARGET_NAME ${TARGET_NAME})
    else()
        set(LIB_TARGET_NAME _${TARGET_NAME})
    endif()

    # Add the shared library imported into python
    ADD_LIBRARY(${LIB_TARGET_NAME} SHARED ${${TARGET_NAME}_SOURCES} ${TARGET_NAME}_PyBuild)
    TARGET_LINK_LIBRARIES(${LIB_TARGET_NAME} ${${TARGET_NAME}_DEPS} ${Boost_LIBRARIES})
    SET_TARGET_PROPERTIES(${LIB_TARGET_NAME} PROPERTIES PREFIX "")

    # Installation of shared library
    if(NOT ${${TARGET_NAME}_NOINSTALL_FOUND})
        if(${${TARGET_NAME}_PREFIX_FOUND})
            INSTALL( TARGETS ${LIB_TARGET_NAME} LIBRARY DESTINATION ${PREFIX}/${TARGET_NAME})
        else()
            INSTALL( TARGETS ${LIB_TARGET_NAME} LIBRARY DESTINATION ${PYTHON_SITE_PATH}/${TARGET_NAME})
        endif()
    endif()

    # The package install command
    SET(${TARGET_NAME}_PYINSTALLCMD 
        "python ${${TARGET_NAME}_PYDIR}/setup.py install --prefix ${CMAKE_INSTALL_PREFIX}")

    # Installation of package
    if(NOT ${${TARGET_NAME}_NOINSTALL_FOUND})
        INSTALL(CODE "execute_process(COMMAND ${${TARGET_NAME}_PYINSTALLCMD} 
               WORKING_DIRECTORY ${${TARGET_NAME}_PYDIR})")
    endif()

endmacro(add_python_library)





