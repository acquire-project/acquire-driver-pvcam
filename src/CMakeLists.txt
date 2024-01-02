if (NOT TARGET acquire-core-logger)
    add_subdirectory(../acquire-common/acquire-core-libs ${CMAKE_CURRENT_BINARY_DIR}/acquire-core-libs)
endif ()

set(tgt acquire-driver-pvcam)

if (TARGET pvcam)
    add_library(${tgt} MODULE pvcam.cpp)
    target_link_libraries(${tgt} PRIVATE
            pvcam
            acquire-core-logger
            acquire-core-platform
            acquire-device-kit
            acquire-device-hal
            )
else ()
    add_library(${tgt} MODULE
            unsupported.driver.c
            )
    target_link_libraries(${tgt} PRIVATE
            acquire-core-logger
            acquire-device-kit
    )
endif ()

target_add_git_versioning(${tgt})
set_target_properties(${tgt} PROPERTIES
        MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>"
        )
target_enable_simd(${tgt})
install(TARGETS ${tgt} LIBRARY DESTINATION lib)
