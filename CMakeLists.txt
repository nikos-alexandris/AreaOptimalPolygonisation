cmake_minimum_required(VERSION 3.16)
project(K23g_2)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "-Wall -Wextra")

find_package(CGAL)

add_executable(evaluate source/main.cpp include/Common.hpp source/Common.cpp include/Config/ArgParser.hpp include/Config/Error.hpp include/Drivers/Global/Algorithm.hpp source/Config/ArgParser.cpp source/Drivers/Global/Algorithm.cpp include/Drivers/Global/LocalSearch.hpp source/Drivers/Global/LocalSearch.cpp include/Drivers/Global/SimulatedAnnealing.hpp source/Drivers/Global/SimulatedAnnealing.cpp include/Drivers/Global/AntColony.hpp source/Drivers/Global/AntColonyDriver.cpp include/Drivers/Local/Algorithm.hpp include/Drivers/Local/Incremental.hpp include/Drivers/Local/ConvexHull.hpp include/Drivers/Local/Onion.hpp source/Drivers/Local/Algorithm.cpp source/Drivers/Local/Incremental.cpp source/Drivers/Local/ConvexHull.cpp source/Drivers/Local/Onion.cpp include/Evaluator.hpp source/Evaluator.cpp)
target_include_directories(evaluate PRIVATE include)

target_link_libraries(evaluate CGAL::CGAL)