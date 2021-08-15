SET(TEST_FILE
        TEST_config)

FOREACH(test_src ${TEST_FILE})
    ADD_EXECUTABLE(${test_src} ${test_src}.cpp)
    TARGET_LINK_LIBRARIES(${test_src} ${THIRD_PARTY_LIBS} myslam)
    ADD_TEST(${test_src} ${test_src})
ENDFOREACH(test_src)