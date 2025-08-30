idf_build_get_property(target IDF_TARGET)

if(${IDF_TARGET} STREQUAL "esp32")
    set(links   "-L ${CMAKE_CURRENT_SOURCE_DIR}/lib/esp32")

elseif(${IDF_TARGET} STREQUAL "esp32s2")
    set(links   "-L ${CMAKE_CURRENT_SOURCE_DIR}/lib/esp32s2")

elseif(${IDF_TARGET} STREQUAL "esp32s3")
    set(links   "-L ${CMAKE_CURRENT_SOURCE_DIR}/lib/esp32s3")

elseif(${IDF_TARGET} STREQUAL "esp32c3")    
    set(links   "-L ${CMAKE_CURRENT_SOURCE_DIR}/lib/esp32c3")

endif()

set(INCLUDE_DIRS 
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include/tool
    ${CMAKE_CURRENT_SOURCE_DIR}/include/typedef
    ${CMAKE_CURRENT_SOURCE_DIR}/include/image
    ${CMAKE_CURRENT_SOURCE_DIR}/include/math
    ${CMAKE_CURRENT_SOURCE_DIR}/include/nn
    ${CMAKE_CURRENT_SOURCE_DIR}/include/tvm
    ${CMAKE_CURRENT_SOURCE_DIR}/include/layer
    ${CMAKE_CURRENT_SOURCE_DIR}/include/detect
    ${CMAKE_CURRENT_SOURCE_DIR}/include/model_zoo)
idf_component_register(
    INCLUDE_DIRS 
        ${INCLUDE_DIRS}
        REQUIRES 
        lvgl 
        freertos
        main

)

target_link_libraries(${COMPONENT_TARGET}   INTERFACE   ${links})

set(lib     libcat_face_detect.a
            libhuman_face_detect.a
	        libcolor_detect.a
            libmfn.a
            libdl.a)
target_link_libraries(${COMPONENT_TARGET}   INTERFACE   ${lib})


