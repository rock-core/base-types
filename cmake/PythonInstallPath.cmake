# from the opencv cmake script
# Gives the path where to install additional python packages.
# This is normally prefix/lib/pythonx.y/site-packages or prefix/lib/pythonx.y/dist-packages.
#
# Note: As known for now python's distutils uses 'dist-packages' if no prefix is given 
# and 'site-packages' if a prefix is given.
#
# This module defines:
# PYTHON_INSTALL_PATH   : The path to install python packages to. site/dist naming depends on os.
# PYTHON_SITE_PATH      : Same as PYTHON_INSTALL_PATH but always with 'site-packages'.

execute_process(COMMAND ${PYTHON_EXECUTABLE} --version
      ERROR_VARIABLE PYTHON_VERSION_FULL
      OUTPUT_STRIP_TRAILING_WHITESPACE)

string(REGEX MATCH "[0-9].[0-9]" PYTHON_VERSION_MAJOR_MINOR "${PYTHON_VERSION_FULL}")
if(UNIX)
    if(APPLE)
        set(PYTHON_INSTALL_PATH lib/python${PYTHON_VERSION_MAJOR_MINOR}/site-packages CACHE PATH "Where to install the python packages.")
    else() #debian based assumed, install to the dist-packages.
       set(PYTHON_INSTALL_PATH lib/python${PYTHON_VERSION_MAJOR_MINOR}/dist-packages CACHE PATH "Where to install the python packages.")
    endif()
       set(PYTHON_SITE_PATH lib/python${PYTHON_VERSION_MAJOR_MINOR}/site-packages CACHE PATH "Where to install the python packages.")
endif()
if(WIN32)
    get_filename_component(PYTHON_PATH "[HKEY_LOCAL_MACHINE\\SOFTWARE\\Python\\PythonCore\\${PYTHON_VERSION_MAJOR_MINOR}\\InstallPath]" ABSOLUTE CACHE)
    set(PYTHON_INSTALL_PATH "${PYTHON_PATH}/Lib/site-packages")
    set(PYTHON_INSTALL_PATH "${PYTHON_PATH}/Lib/site-packages")
endif()

