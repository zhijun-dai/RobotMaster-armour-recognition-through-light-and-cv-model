# 设置最低 CMake 版本
cmake_minimum_required(VERSION 3.10.0)

# 定义项目名称和版本
project(rb-vision-task-2 VERSION 0.1.0 LANGUAGES C CXX)

# 查找 OpenCV 库 (这是关键！)
find_package(OpenCV REQUIRED)

# 添加可执行文件，指定源文件
add_executable(umain src/usage/main.cpp)
add_executable(pmain src/practical/main.cpp)
add_executable(find4main src/practical/find4main.cpp)
add_executable(main2 src/practical/main2.cpp)
add_executable(testmain src/practical/testmain.cpp)

# 链接 OpenCV 库到可执行文件
target_link_libraries(umain ${OpenCV_LIBS})
target_link_libraries(pmain ${OpenCV_LIBS})
target_link_libraries(find4main ${OpenCV_LIBS})
target_link_libraries(main2 ${OpenCV_LIBS})
target_link_libraries(testmain ${OpenCV_LIBS})

