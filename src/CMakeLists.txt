if (NOT TARGET acquire-core-logger)
    add_subdirectory(acquire-core-libs)
endif ()

set(tgt acquire-driver-pvcam)

if (TARGET pvcam)
    add_library(${tgt} MODULE pvcam.cpp)
    target_link_libraries(${tgt} PRIVATE
            acquire-core-logger
            acquire-core-platform
            acquire-device-kit
            pvcam
            )
else ()
    add_library(${tgt} MODULE unsupported.driver.c)
    target_link_libraries(${tgt} PRIVATE
            acquire-core-logger
            acquire-core-platform
            acquire-device-kit
            )
endif ()

target_add_git_versioning(${tgt})
set_target_properties(${tgt} PROPERTIES
        MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>"
        )
target_enable_simd(${tgt})
install(TARGETS ${tgt} LIBRARY DESTINATION lib)