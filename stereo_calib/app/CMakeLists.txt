SET(TEST_SOURCES 
    stereo_calib
    stereo_capture
    mono_calib
    fmt_test)

FOREACH (test_src ${TEST_SOURCES})
    ADD_EXECUTABLE(${test_src} ${test_src}.cpp)
    TARGET_LINK_LIBRARIES(${test_src} ${OpenCV_LIBS} fmt)
ENDFOREACH (test_src)
