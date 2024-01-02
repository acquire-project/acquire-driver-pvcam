if (${NOTEST})
    message(STATUS "Skipping test targets")
else ()
    set(NOTEST "TRUE")
    add_subdirectory(../acquire-common/acquire-driver-common ${CMAKE_CURRENT_BINARY_DIR}/acquire-driver-common)
    add_subdirectory(../acquire-common/acquire-video-runtime ${CMAKE_CURRENT_BINARY_DIR}/acquire-video-runtime)
    set(NOTEST "FALSE")

    #
    # PARAMETERS
    #
    set(project acquire-driver-pvcam) # CMAKE_PROJECT_NAME gets overridden if this is a subtree of another project

    #
    # Tests
    #
    set(tests
            list-devices
            bsi-express-metadata
            bsi-express-one-video-stream
            bsi-express-repeat-start
            bsi-express-repeat-start-no-stop
            bsi-express-configure-properties
            prime-bsi-metadata
            prime-bsi-one-video-stream
            prime-bsi-repeat-start
            prime-bsi-repeat-start-no-stop
            prime-bsi-configure-properties
    )

    foreach (name ${tests})
        set(tgt "${project}-${name}")
        add_executable(${tgt} ${name}.cpp)
        target_compile_definitions(${tgt} PUBLIC "TEST=\"${tgt}\"")
        set_target_properties(${tgt} PROPERTIES
                MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>"
        )
        target_include_directories(${tgt} PRIVATE "${CMAKE_CURRENT_LIST_DIR}/../")
        target_link_libraries(${tgt}
                acquire-core-logger
                acquire-core-platform
                acquire-video-runtime
        )

        add_test(NAME test-${tgt} COMMAND ${tgt})
        if (name MATCHES "^prime-bsi")
            set_tests_properties(test-${tgt} PROPERTIES LABELS "acquire-driver-pvcam;prime-bsi")
        elseif (name MATCHES "^bsi-express")
            set_tests_properties(test-${tgt} PROPERTIES LABELS "acquire-driver-pvcam;bsi-express")
        else () # list-devices
            set_tests_properties(test-${tgt} PROPERTIES LABELS "acquire-driver-pvcam;prime-bsi;bsi-express")
        endif ()
    endforeach ()

    #
    # Copy driver to tests
    #
    list(POP_FRONT tests onename)

    foreach (driver
            acquire-driver-common
            acquire-driver-pvcam
    )
        add_custom_target(${project}-copy-${driver}-for-tests
                COMMAND ${CMAKE_COMMAND} -E copy
                $<TARGET_FILE:${driver}>
                $<TARGET_FILE_DIR:${project}-${onename}>
                DEPENDS ${driver}
                COMMENT "Copying ${driver} to $<TARGET_FILE_DIR:${project}-${onename}>"
        )

        foreach (name ${tests})
            add_dependencies(${tgt} ${project}-copy-${driver}-for-tests)
        endforeach ()
    endforeach ()
endif ()
