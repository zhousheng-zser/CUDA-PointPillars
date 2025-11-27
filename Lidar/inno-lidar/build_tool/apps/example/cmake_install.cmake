# Install script for directory: /home/zser/inno-lidar/code/apps/example

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}/home/zser/inno-lidar/code/apps/example/demo" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/zser/inno-lidar/code/apps/example/demo")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}/home/zser/inno-lidar/code/apps/example/demo"
         RPATH "")
  endif()
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/zser/inno-lidar/code/apps/example/demo")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/home/zser/inno-lidar/code/apps/example" TYPE EXECUTABLE FILES "/home/zser/inno-lidar/code/build/apps/example/demo")
  if(EXISTS "$ENV{DESTDIR}/home/zser/inno-lidar/code/apps/example/demo" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/zser/inno-lidar/code/apps/example/demo")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}/home/zser/inno-lidar/code/apps/example/demo"
         OLD_RPATH "/home/zser/inno-lidar/code/apps/example/../../lib:/home/zser/inno-lidar/code/apps/example/../../../lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/home/zser/inno-lidar/code/apps/example/demo")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}/home/zser/inno-lidar/code/apps/example/sphere2xyz" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/zser/inno-lidar/code/apps/example/sphere2xyz")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}/home/zser/inno-lidar/code/apps/example/sphere2xyz"
         RPATH "")
  endif()
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/zser/inno-lidar/code/apps/example/sphere2xyz")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/home/zser/inno-lidar/code/apps/example" TYPE EXECUTABLE FILES "/home/zser/inno-lidar/code/build/apps/example/sphere2xyz")
  if(EXISTS "$ENV{DESTDIR}/home/zser/inno-lidar/code/apps/example/sphere2xyz" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/zser/inno-lidar/code/apps/example/sphere2xyz")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}/home/zser/inno-lidar/code/apps/example/sphere2xyz"
         OLD_RPATH "/home/zser/inno-lidar/code/apps/example/../../lib:/home/zser/inno-lidar/code/apps/example/../../../lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/home/zser/inno-lidar/code/apps/example/sphere2xyz")
    endif()
  endif()
endif()

