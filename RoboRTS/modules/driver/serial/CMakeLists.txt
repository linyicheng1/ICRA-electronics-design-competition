project(serial_comm)

file(GLOB ProtoFiles "${CMAKE_CURRENT_SOURCE_DIR}/proto/*.proto")
rrts_protobuf_generate_cpp(${CMAKE_CURRENT_SOURCE_DIR}/proto SerialProtoSrc SerialProtoHds ${ProtoFiles})

add_executable(serial_com_node serial_com_node.cpp ${SerialProtoSrc} ${SerialProtoHds} curltest.h curltest.cpp )
target_include_directories(serial_com_node PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

FIND_PACKAGE(CURL)
IF(CURL_FOUND)
    INCLUDE_DIRECTORIES(${CURL_INCLUDE_DIR})
    target_link_libraries(serial_com_node
            PRIVATE
            common::rrts
            common::io
            ${catkin_LIBRARIES}
            ${PROTOBUF_LIBRARIES}
            ${GLOG_LIBRARY}
            ${CURL_LIBRARY}
            )

ELSE(CURL_FOUND)
    target_link_libraries(serial_com_node
            PRIVATE
            common::rrts
            common::io
            ${catkin_LIBRARIES}
            ${PROTOBUF_LIBRARIES}
            ${GLOG_LIBRARY}
            )
    MESSAGE(FATAL_ERROR ”CURL library not found”)
ENDIF(CURL_FOUND)
