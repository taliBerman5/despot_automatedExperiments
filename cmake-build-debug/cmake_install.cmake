# Install script for directory: /home/tali/CLionProjects/despot_automatedExperiments

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
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
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

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdespot.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdespot.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdespot.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/tali/CLionProjects/despot_automatedExperiments/cmake-build-debug/libdespot.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdespot.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdespot.so")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdespot.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/despot" TYPE DIRECTORY FILES "/home/tali/CLionProjects/despot_automatedExperiments/include/despot/")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/despot/cmake/DespotTargets.cmake")
    file(DIFFERENT _cmake_export_file_changed FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/despot/cmake/DespotTargets.cmake"
         "/home/tali/CLionProjects/despot_automatedExperiments/cmake-build-debug/CMakeFiles/Export/d494a154d393af2dfead40a461a90ec4/DespotTargets.cmake")
    if(_cmake_export_file_changed)
      file(GLOB _cmake_old_config_files "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/despot/cmake/DespotTargets-*.cmake")
      if(_cmake_old_config_files)
        string(REPLACE ";" ", " _cmake_old_config_files_text "${_cmake_old_config_files}")
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/despot/cmake/DespotTargets.cmake\" will be replaced.  Removing files [${_cmake_old_config_files_text}].")
        unset(_cmake_old_config_files_text)
        file(REMOVE ${_cmake_old_config_files})
      endif()
      unset(_cmake_old_config_files)
    endif()
    unset(_cmake_export_file_changed)
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/despot/cmake" TYPE FILE FILES "/home/tali/CLionProjects/despot_automatedExperiments/cmake-build-debug/CMakeFiles/Export/d494a154d393af2dfead40a461a90ec4/DespotTargets.cmake")
  if(CMAKE_INSTALL_CONFIG_NAME MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/despot/cmake" TYPE FILE FILES "/home/tali/CLionProjects/despot_automatedExperiments/cmake-build-debug/CMakeFiles/Export/d494a154d393af2dfead40a461a90ec4/DespotTargets-debug.cmake")
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/despot/cmake" TYPE FILE FILES "/home/tali/CLionProjects/despot_automatedExperiments/cmake-build-debug/DespotConfig.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/tali/CLionProjects/despot_automatedExperiments/cmake-build-debug/examples/cpp_models/adventurer/cmake_install.cmake")
  include("/home/tali/CLionProjects/despot_automatedExperiments/cmake-build-debug/examples/cpp_models/bridge/cmake_install.cmake")
  include("/home/tali/CLionProjects/despot_automatedExperiments/cmake-build-debug/examples/cpp_models/chain/cmake_install.cmake")
  include("/home/tali/CLionProjects/despot_automatedExperiments/cmake-build-debug/examples/cpp_models/navigation/cmake_install.cmake")
  include("/home/tali/CLionProjects/despot_automatedExperiments/cmake-build-debug/examples/cpp_models/pocman/cmake_install.cmake")
  include("/home/tali/CLionProjects/despot_automatedExperiments/cmake-build-debug/examples/cpp_models/reg_demo/cmake_install.cmake")
  include("/home/tali/CLionProjects/despot_automatedExperiments/cmake-build-debug/examples/cpp_models/rock_sample/cmake_install.cmake")
  include("/home/tali/CLionProjects/despot_automatedExperiments/cmake-build-debug/examples/cpp_models/simple_rock_sample/cmake_install.cmake")
  include("/home/tali/CLionProjects/despot_automatedExperiments/cmake-build-debug/examples/cpp_models/tag/cmake_install.cmake")
  include("/home/tali/CLionProjects/despot_automatedExperiments/cmake-build-debug/examples/cpp_models/tiger/cmake_install.cmake")
  include("/home/tali/CLionProjects/despot_automatedExperiments/cmake-build-debug/examples/pomdpx_models/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/tali/CLionProjects/despot_automatedExperiments/cmake-build-debug/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
