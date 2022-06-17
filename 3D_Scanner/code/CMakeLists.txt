cmake_minimum_required(VERSION 3.16)

# Micro-Epsilon Genicam Client C++ interface lib examples
#
project(TestProject)

find_package(MEGCPP 1.6.2 REQUIRED)


set(LIBM_LIBRARIES m)
link_libraries(pthread)
link_directories(../Libs)

get_cmake_property(_variableNames VARIABLES)
list (SORT _variableNames)
foreach (_variableName ${_variableNames})
    message(STATUS "${_variableName}=${${_variableName}}")
endforeach()

set_property(DIRECTORY PROPERTY USE_FOLDERS ON)
foreach(example pcd_gen)
	add_executable(MEGCPP_${example} src/${example}.cpp)
	target_link_libraries(MEGCPP_${example} 3D-SensorSDK::MEGCPP)
	set_target_properties(MEGCPP_${example} PROPERTIES
		DEBUG_POSTFIX "d"
		FOLDER "MEGCPP"
		OUTPUT_NAME ${example})
	install(TARGETS MEGCPP_${example} RUNTIME DESTINATION 3D-SensorSDK/MEGCPP/${example})
	if(MSVC)
		install(FILES $<TARGET_PDB_FILE:MEGCPP_${example}> DESTINATION 3D-SensorSDK/MEGCPP/${example})
	endif()
endforeach()
