cmake_minimum_required(VERSION 3.5)
include(CMakePrintHelpers)

project(RemoteConnect VERSION 0.1 LANGUAGES CXX)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_VERBOSE_MAKEFILE ON)

option(BUILD_GUI "build gui" ON)
set(CU_ROOT "CppUtility")
cmake_print_variables(CU_ROOT)
file(GLOB_RECURSE CU_FILES "${CU_ROOT}/*.hpp")

find_package(nlohmann_json CONFIG REQUIRED)
find_package(FFMPEG REQUIRED)
find_package(asio CONFIG REQUIRED)

if (${BUILD_GUI})
    find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS 
        Widgets 
        OpenGLWidgets
        Network
    )
    find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS 
        Widgets 
        OpenGLWidgets
        Network
    )

    set(TS_FILES RemoteConnect_zh_CN.ts)

    set(PROJECT_SOURCES
        main.cpp
        mainwindow.hpp
        OGLWidget.hpp
        RcUtility.hpp
        ${CU_FILES}
    )

    if(${QT_VERSION_MAJOR} GREATER_EQUAL 6)
        qt_add_executable(RemoteConnect
            MANUAL_FINALIZATION
            ${PROJECT_SOURCES}
        )
        # Define target properties for Android with Qt 6 as:
        #    set_property(TARGET RemoteConnect APPEND PROPERTY QT_ANDROID_PACKAGE_SOURCE_DIR
        #                 ${CMAKE_CURRENT_SOURCE_DIR}/android)
        # For more information, see https://doc.qt.io/qt-6/qt-add-executable.html#target-creation

        # qt_create_translation(QM_FILES ${CMAKE_SOURCE_DIR} ${TS_FILES})
    else()
        if(ANDROID)
            add_library(RemoteConnect SHARED
                ${PROJECT_SOURCES}
            )
            # Define properties for Android with Qt 5 after find_package() calls as:
            #    set(ANDROID_PACKAGE_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/android")
        else()
            add_executable(RemoteConnect
                ${PROJECT_SOURCES}
            )
        endif()

        qt5_create_translation(QM_FILES ${CMAKE_SOURCE_DIR} ${TS_FILES})
    endif()

    target_include_directories(RemoteConnect PRIVATE
        ${FFMPEG_INCLUDE_DIRS} 
        ${CU_ROOT}
    )

    target_link_directories(RemoteConnect PRIVATE ${FFMPEG_LIBRARY_DIRS})
    target_compile_definitions(RemoteConnect PRIVATE CU_IMG_HAS_QT)
    target_link_libraries(RemoteConnect PRIVATE
        Qt${QT_VERSION_MAJOR}::Widgets Qt${QT_VERSION_MAJOR}::OpenGLWidgets Qt${QT_VERSION_MAJOR}::Network
        ${FFMPEG_LIBRARIES}
            nlohmann_json::nlohmann_json
    )

    set_target_properties(RemoteConnect PROPERTIES
        MACOSX_BUNDLE_GUI_IDENTIFIER my.example.com
        MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
        MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
        MACOSX_BUNDLE TRUE
        WIN32_EXECUTABLE TRUE
    )

    install(TARGETS RemoteConnect
        BUNDLE DESTINATION .
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})

    if(QT_VERSION_MAJOR EQUAL 6)
        qt_finalize_executable(RemoteConnect)
    endif()
endif ()

set(RCS_LIBS
    ${FFMPEG_LIBRARIES} 
    asio::asio
    nlohmann_json::nlohmann_json
)

if (UNIX)
    list(APPEND RCS_LIBS X11)
endif()

add_executable(RemoteConnectServer 
    server.cpp 
    RcUtility.hpp 
    ${CU_FILES}
)
target_include_directories(RemoteConnectServer PRIVATE ${CU_ROOT})
target_link_directories(RemoteConnectServer PRIVATE ${FFMPEG_LIBRARY_DIRS})
target_link_libraries(RemoteConnectServer PRIVATE ${RCS_LIBS})
