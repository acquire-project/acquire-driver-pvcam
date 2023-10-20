# PVCAM SDK library

find_path(pvcam_include_dir "pvcam.h"
    PATH_SUFFIXES
        "Photometrics/PVCamSDK/Inc"  # windows
    DOC "Directory that contains pvcam.h"
    NO_CACHE)

if(pvcam_include_dir)
    message(STATUS "PVCAM ${pvcam_include_dir}")

    set(tgt pvcam)
    add_library(${tgt} STATIC IMPORTED)
    target_include_directories(${tgt} INTERFACE ${pvcam_include_dir})

    # See the following guide for this definition:
    # https://cmake.org/cmake/help/latest/guide/importing-exporting/index.html#importing-libraries
    if(WIN32)
        set_target_properties(${tgt} PROPERTIES
            IMPORTED_LOCATION "${pvcam_include_dir}../Lib/amd64/pvcam64.lib"
        )
    endif()
else()
    message(STATUS "Could not find pvcam.h")
endif()
