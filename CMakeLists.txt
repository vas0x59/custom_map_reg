cmake_minimum_required(VERSION 2.8.12)

project(aruco)

if(MINGW)
    add_definitions("-D_USE_MATH_DEFINES") # For M_PI
endif()

if(MSVC)
    add_definitions("-std=c++11 -WX -W2")
    add_definitions("-D_USE_MATH_DEFINES") # For M_PI
else()
    add_definitions("-std=c++11 -Wall -Wextra -Werror")
endif()


find_package( OpenCV REQUIRED )

add_executable(
    aruco main.cpp
)

include_directories(
./markers
)

target_link_libraries(aruco
    ${OpenCV_LIBS}
)
