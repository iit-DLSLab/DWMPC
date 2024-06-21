# Add messages
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
add_library(${MSGS_LIBRARY_NAME} SHARED)
include(dls_message)		# message generation function
# Add idls here
dls_add_message(message) 	# generate message

# Add wrappers
add_library(${MSGS_WRAPPERS_LIBRARY_NAME} SHARED
	# add wrappers files here
)
set_property(TARGET ${MSGS_WRAPPERS_LIBRARY_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)
target_include_directories(${MSGS_WRAPPERS_LIBRARY_NAME}
	PUBLIC
		include
        ${CMAKE_CURRENT_BINARY_DIR}/include
)
target_link_directories(${MSGS_WRAPPERS_LIBRARY_NAME}
	PUBLIC
		/usr/lib/dls2
)
target_link_libraries(${MSGS_WRAPPERS_LIBRARY_NAME}
	PUBLIC
		# add wrappers dependencies here
		dls_message_wrappers
		${MSGS_LIBRARY_NAME}
)

# ================================== Install =================================
install(
    TARGETS ${MSGS_LIBRARY_NAME}
    DESTINATION /usr/lib
    COMPONENT ${MSGS_LIBRARY_NAME}
)

install(
	DIRECTORY   include/
	DESTINATION /usr/include/dls2
	COMPONENT   ${MSGS_WRAPPERS_LIBRARY_NAME}_dev
)

install(
	TARGETS             ${MSGS_WRAPPERS_LIBRARY_NAME}
	DESTINATION         /usr/lib/dls2/controllers/${PLUGIN_NAME}
	COMPONENT           ${MSGS_WRAPPERS_LIBRARY_NAME}
)
