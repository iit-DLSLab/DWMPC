# Add topics
add_library(${TOPICS_LIBRARY_NAME}
	SHARED
	src/topics.cpp
)
target_include_directories(${TOPICS_LIBRARY_NAME}
PUBLIC
    include
	#${CMAKE_CURRENT_BINARY_DIR}/../messages/include
)
target_link_libraries(${TOPICS_LIBRARY_NAME}
	PUBLIC
	#dls_messages
	#${MSGS_LIBRARY_NAME} 
	dls_topics
)
target_link_directories(${TOPICS_LIBRARY_NAME}
	PRIVATE
        /usr/lib/dls2
)
# ================================== Install =================================
install(
	DIRECTORY   include/
	DESTINATION /usr/include/dls2
	COMPONENT   ${TOPICS_LIBRARY_NAME}_dev
)

install(
	TARGETS             ${TOPICS_LIBRARY_NAME}
	LIBRARY DESTINATION /usr/lib/dls2/controllers/${PLUGIN_NAME}
	COMPONENT           ${TOPICS_LIBRARY_NAME}
)
