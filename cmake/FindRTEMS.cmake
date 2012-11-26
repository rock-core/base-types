INCLUDE(CMakeForceCompiler)

SET(CMAKE_SYSTEM_NAME RTEMS)

if(DEFINED ENV{RTEMS_INSTALL_DIR})
    MESSAGE("OVERSIDING Setting of rtems install dir withenvoirmnment")
    set(RTEMS_INSTALL_DIR $ENV{RTEMS_INSTALL_DIR} CACHE PATH "path to rtems installation dir" FORCE)
    #set(RTEMS_INSTALL_DIR $ENV{RTEMS_INSTALL_DIR})
endif()
    

if(NOT RTEMS_INSTALL_DIR)
    MESSAGE(FATAL_ERROR "Please set RTEMS_INSTALL_DIR before running rock cross dev for rtems -${RTEMS_INSTALL_DIR}-")
endif()
if(NOT DEFINED ENV{TARGET})
    MESSAGE(FATAL_ERROR "Please set the Enviorment variable TARGET before running rock cross dev for rtems")
endif()
if(NOT DEFINED ENV{BSP})
    MESSAGE(FATAL_ERROR "Please set the Enviorment variable BSP before running rock cross dev for rtems")
endif()

#set(RTEMS_INSTALL_DIR $ENV{RTEMS_INSTALL_DIR} CACHE PATH "path to rtems installation dir" FORCE)
  set(RTEMS_CC_PREFIX $ENV{TARGET})
  set(BSP $ENV{BSP})
  SET(CMAKE_CROSSCOMPILING ON)

  if(EXISTS ${RTEMS_INSTALL_DIR}/${RTEMS_CC_PREFIX}/${BSP}/lib/include/rtems/system.h)
    message("-- Looking for RTEMS - found in ${RTEMS_INSTALL_DIR}")
    set(RTEMS_FOUND TRUE)
    set(RTEMS_INCLUDE_DIRS ${RTEMS_INSTALL_DIR}/${RTEMS_CC_PREFIX}/${BSP}/lib/include )
    set(RTEMS_LIBRARIES -L${RTEMS_INSTALL_DIR}/${RTEMS_CC_PREFIX}/${BSP}/lib -lrtemsbsp -lrtemscpu -lc )

    set(RTEMS_BUILD ${RTEMS_INSTALL_DIR}/${RTEMS_CC_PREFIX})
    set(PROJECT_RELEASE ${RTEMS_BUILD}/${BSP})
    set(PROJECT_TOOLS ${RTEMS_BUILD}/${BSP}/build-tools)
    set(CMAKE_C_COMPILER ${RTEMS_CC_PREFIX}-gcc)
    set(AS ${RTEMS_CC_PREFIX}-as)
    set(AR ${RTEMS_CC_PREFIX}-ar)
    set(NM ${RTEMS_CC_PREFIX}-nm)
    set(LD ${RTEMS_CC_PREFIX}-ld)
    set(SIZE ${RTEMS_CC_PREFIX}-size)
    set(OBJCOPY ${RTEMS_CC_PREFIX}-objcopy)
    set(RANLIB ${RTEMS_CC_PREFIX}-ranlib)
    set(CMAKE_CXX_COMPILER ${RTEMS_CC_PREFIX}-g++)
    set(CXXLINK ${RTEMS_CC_PREFIX}-g++)
    set(ARFLAGS "-ruv")
    set(CDEFS "-D__RTEMS__")
    set(BUILD_STATIC ON)
    set(NOT_BUILD_SHARED ON)

    MESSAGE("FOO: ${RTEMS_BUILD}") 
    #set(CFLAGS "--pipe -DBOOST_SYSTEM_NO_DEPRECATED  -D__BSD_VISIBLE  -D__RTEMS__  -DBoost_USE_STATIC_LIBS  -DBOOST_DISABLE_THREADS -DBOOST_THREAD_POSIX  -DNOT_BUILD_SHARED -DRUBY_EXTENSIONS_AVAILABLE=NO -B${RTEMS_BUILD}/${BSP}/lib/ -specs bsp_specs -qrtems ${IMPORT_CPPFLAGS}  ${DIR_CPPFLAGS} -I${RTEMS_BUILD}/${BSP}/lib/include  -I${RTEMS_BUILD}/include  -I${RTEMS_BUILD}/${BSP}/lib/include/networking  -I${RTEMS_BUILD}/${BSP}/lib/include/sys  -I${INSTALL_PREFIX}/include  -I${INSTALL_PREFIX}/include -fno-omit-frame-pointer ")
    #set(CFLAGS "--pipe -DBOOST_SYSTEM_NO_DEPRECATED  -D__BSD_VISIBLE  -D__RTEMS__  -DBoost_USE_STATIC_LIBS  DBOOST_DISABLE_THREADS -DNOT_BUILD_SHARED -DRUBY_EXTENSIONS_AVAILABLE=NO -B${RTEMS_BUILD}/${BSP}/lib/ -specs bsp_specs -qrtems ${IMPORT_CPPFLAGS}  ${DIR_CPPFLAGS} -I${RTEMS_BUILD}/${BSP}/lib/include  -I${RTEMS_BUILD}/include  -I${RTEMS_BUILD}/${BSP}/lib/include/networking  -I${RTEMS_BUILD}/${BSP}/lib/include/sys  -I${INSTALL_PREFIX}/include  -I${INSTALL_PREFIX}/include -fno-omit-frame-pointer ")
    #set(CFLAGS "--pipe -DBOOST_SYSTEM_NO_DEPRECATED  -D__BSD_VISIBLE  -D__RTEMS__  -DBoost_USE_STATIC_LIBS  -DBOOST_DISABLE_THREADS -DNOT_BUILD_SHARED -DRUBY_EXTENSIONS_AVAILABLE=NO -B${RTEMS_BUILD}/${BSP}/lib/ -specs bsp_specs -qrtems ${IMPORT_CPPFLAGS}  ${DIR_CPPFLAGS} -I${RTEMS_BUILD}/${BSP}/lib/include  -I${RTEMS_BUILD}/include  -I${RTEMS_BUILD}/${BSP}/lib/include/networking  -I${RTEMS_BUILD}/${BSP}/lib/include/sys  -I${INSTALL_PREFIX}/omniORB/include  -I${INSTALL_PREFIX}/boost/include -fno-omit-frame-pointer ")
    #set(CFLAGS "--pipe -DBOOST_SYSTEM_NO_DEPRECATED  -DBOOST_THREAD_POSIX -D__BSD_VISIBLE  -D__RTEMS__  -DBoost_USE_STATIC_LIBS  -DNOT_BUILD_SHARED -DRUBY_EXTENSIONS_AVAILABLE=NO -B${RTEMS_BUILD}/${BSP}/lib/ -specs bsp_specs -qrtems ${IMPORT_CPPFLAGS}  ${DIR_CPPFLAGS} -I${RTEMS_BUILD}/${BSP}/lib/include  -I${RTEMS_BUILD}/include  -I${RTEMS_BUILD}/${BSP}/lib/include/networking  -I${RTEMS_BUILD}/${BSP}/lib/include/sys  -I${INSTALL_PREFIX}/omniORB/include  -I${INSTALL_PREFIX}/boost/include -fno-omit-frame-pointer ")

    SET(DCMAKE_SYSTEM_NAME RTEMS)

    SET(CMAKE_SHARED_LIBRARY_C_FLAGS "")              # -pic 
    SET(CMAKE_SHARED_LIBRARY_CREATE_C_FLAGS "")       # -shared
    SET(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS "")         # +s, flag for exe link to use shared lib
    SET(CMAKE_SHARED_LIBRARY_RUNTIME_C_FLAG "")       # -rpath
    SET(CMAKE_SHARED_LIBRARY_RUNTIME_C_FLAG_SEP "")   # : or empty

    SET(CMAKE_LINK_LIBRARY_SUFFIX "")
    SET(CMAKE_STATIC_LIBRARY_PREFIX "lib")
    SET(CMAKE_STATIC_LIBRARY_SUFFIX ".a")
    SET(CMAKE_SHARED_LIBRARY_PREFIX "lib")          # lib
    SET(CMAKE_SHARED_LIBRARY_SUFFIX ".a")           # .a
    SET(CMAKE_EXECUTABLE_SUFFIX ".bin")             #
    SET(CMAKE_DL_LIBS "" )

    SET(CMAKE_FIND_LIBRARY_PREFIXES "lib")
    SET(CMAKE_FIND_LIBRARY_SUFFIXES ".a")

    # RTEMS doesn't support shared libs
    SET_PROPERTY(GLOBAL PROPERTY TARGET_SUPPORTS_SHARED_LIBS FALSE)
    SET(CMAKE_CXX_LINK_SHARED_LIBRARY )
    SET(CMAKE_CXX_LINK_MODULE_LIBRARY )
    SET(CMAKE_C_LINK_SHARED_LIBRARY )
    SET(CMAKE_C_LINK_MODULE_LIBRARY )
    
    #set(CFLAGS "--pipe -DBOOST_SYSTEM_NO_DEPRECATED  -D__BSD_VISIBLE  -D__RTEMS__  -DBoost_USE_STATIC_LIBS  -DBOOST_THREAD_POSIX  -DNOT_BUILD_SHARED -DRUBY_EXTENSIONS_AVAILABLE=NO -B${RTEMS_BUILD}/${BSP}/lib/ -specs bsp_specs -qrtems ${IMPORT_CPPFLAGS}  ${DIR_CPPFLAGS} -I${RTEMS_BUILD}/${BSP}/lib/include  -I${RTEMS_BUILD}/include  -I${RTEMS_BUILD}/${BSP}/lib/include/networking  -I${RTEMS_BUILD}/${BSP}/lib/include/sys  -I${INSTALL_PREFIX}/include  -I${INSTALL_PREFIX}/include -fno-omit-frame-pointer ")
    set(CFLAGS "--pipe -DBOOST_SYSTEM_NO_DEPRECATED  -D__BSD_VISIBLE  -D__RTEMS__  -DBoost_USE_STATIC_LIBS -DNOT_BUILD_SHARED -DRUBY_EXTENSIONS_AVAILABLE=NO -B${RTEMS_BUILD}/${BSP}/lib/ -specs bsp_specs -qrtems ${IMPORT_CPPFLAGS}  ${DIR_CPPFLAGS} -I${RTEMS_BUILD}/${BSP}/lib/include  -I${RTEMS_BUILD}/include  -I${RTEMS_BUILD}/${BSP}/lib/include/networking  -I${RTEMS_BUILD}/${BSP}/lib/include/sys  -fno-omit-frame-pointer -lboost_math_c99f -lboost_math_tr1l -lboost_graph -lboost_prg_exec_monitor -lboost_serialization -lboost_test_exec_monitor -lboost_thread -lboost_program_options -lboost_math_tr1 -lboost_unit_test_framework -lboost_math_tr1f -lboost_math_c99 -lboost_regex -lboost_system -lboost_math_c99l -lboost_wserialization -lboost_filesystem ${ADD_CFLAGS}")


    if ("${RTEMS_CC_PREFIX}" MATCHES "i?86-*")
        set (CFLAGS "${CFLAGS} -D__i386__ -march=i486  -Wl,-Ttext,0x00100000")
        add_definitions(" -D__i386__ -march=i486  -Wl,-Ttext,0x00100000")
    elseif("${RTEMS_CC_PREFIX}" MATCHES "sparc-*")
        set (CFLAGS "${CFLAGS} -D__SPARC__")
        add_definitions(" -D__SPARC__")
    endif()

    set(CXXFLAGS "${CFLAGS} -fexceptions -fpermissive")

    set(CMAKE_C_FLAGS ${CFLAGS})
    set(CMAKE_CXX_FLAGS ${CXXFLAGS})


    #add_definitions("-DBOOST_DISABLE_THREADS")

    include_directories (${RTEMS_INSTALL_DIR}/${TARGET}/${BSP}/lib/include ${RTEMS_INSTALL_DIR}/${TARGET}/include ${RTEMS_INSTALL_DIR}/${TARGET}/${BSP}/lib/include/networking /home/goldhoorn/limes/clean-rock-rtems/install/i386-rtems/i386-rtems/pc486/lib/include)
    
    add_definitions("-D__RTEMS__ -DBOOST_SYSTEM_NO_DEPRECATED -D__BSD_VISIBLE -DBoost_USE_STATIC_LIBS -DRUBY_EXTENSIONS_AVAILABLE=NO -B${RTEMS_BUILD}/${BSP}/lib/ -specs bsp_specs -qrtems ${IMPORT_CPPFLAGS}  ${DIR_CPPFLAGS} -I${RTEMS_BUILD}/${BSP}/lib/include  -I${RTEMS_BUILD}/include  -I${RTEMS_BUILD}/${BSP}/lib/include/networking  -I${RTEMS_BUILD}/${BSP}/lib/include/sys -fno-omit-frame-pointer --pipe")
    
    add_definitions("-lboost_math_c99f -lboost_math_tr1l -lboost_graph -lboost_prg_exec_monitor -lboost_serialization -lboost_test_exec_monitor -lboost_thread -lboost_program_options -lboost_math_tr1 -lboost_unit_test_framework -lboost_math_tr1f -lboost_math_c99 -lboost_regex -lboost_system -lboost_math_c99l -lboost_wserialization -lboost_filesystem #{ADD_DEF}")

  else(EXISTS ${RTEMS_INSTALL_DIR}/${RTEMS_CC_PREFIX}/${BSP}/lib/include/rtems/system.h)
    message(FATAL_ERROR "-- Looking for RTEMS - not found (tried: ${RTEMS_INSTALL_DIR}/${RTEMS_CC_PREFIX}/${BSP}/lib/include/rtems/system.h)")
    set(RTEMS_FOUND FALSE)
  endif(EXISTS ${RTEMS_INSTALL_DIR}/${RTEMS_CC_PREFIX}/${BSP}/lib/include/rtems/system.h)


