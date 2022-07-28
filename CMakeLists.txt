# Set the minimum version of CMake that can be used
# To find the cmake version run
# $ cmake --version
cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

# Set up the project
project(telnet-rtt
    VERSION 1.0.0 
    LANGUAGES C
)

# Set c standard
set(CMAKE_C_STANDARD 11)

# Command to output information to the console
# Useful for displaying errors, warnings, and debugging
message ("c Flags: " ${CMAKE_C_FLAGS})

# Set c++ standard
set(CMAKE_Cxx_STANDARD 20)

# Command to output information to the console
# Useful for displaying errors, warnings, and debugging
message ("cxx Flags: " ${CMAKE_CXX_FLAGS})

# Create a sources variable with a link to all cpp files to compile
set(SOURCES
        main.c
        )

# Set Properties->General->Configuration Type to Application(.exe)
# Creates app.exe with the listed sources (main.cxx)
# Adds sources to the Solution Explorer
# Add an executable with the above sources
add_executable(${PROJECT_NAME} ${SOURCES})

# Pick up the common stuff
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/tcapi)
# add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/util)
# add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/shell)
if (WIN32)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/getopt)
endif (WIN32)

# Set the directories that should be included in the build command for this target
# when running g++ these will be included as -I/directory/path/
# Properties->C/C++->General->Additional Include Directories
target_include_directories(${PROJECT_NAME}
        PRIVATE
        ${PROJECT_SOURCE_DIR}/
        )

# Set compile definitions
if (WIN32)
target_compile_definitions(${PROJECT_NAME} 
        PRIVATE
#         "HAVE_CONFIG_H"
        " _CRT_SECURE_NO_WARNINGS"
        "_WINSOCK_DEPRECATED_NO_WARNINGS"
#         "LargestIntegralType=unsigned long"
#         LargestIntegralTypePrintfFormat="%llx"
        )
endif (WIN32)

# Set compile Linker lib
if (WIN32)
target_link_libraries(${PROJECT_NAME}
        PRIVATE
        ws2_32.lib
        )
endif (WIN32)

# Set compile Linker lib
if (UNIX)
target_link_libraries(${PROJECT_NAME}
        PRIVATE
        -ludev
        -lSDL2
        -lm
        )
endif (UNIX)

if (UNIX)
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD COMMAND ${SIZE} "${PROJECT_NAME}")
endif (UNIX)