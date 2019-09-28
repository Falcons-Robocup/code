FIND_PACKAGE(Boost COMPONENTS system filesystem thread REQUIRED)


ADD_LIBRARY(rutils SHARED rtime.cpp tprintf.cpp)
SET_TARGET_PROPERTIES(rutils PROPERTIES COMPILE_FLAGS "-fPIC")
TARGET_LINK_LIBRARIES(rutils)

# testers
ADD_EXECUTABLE(test_rtime tst/test_rtime.cpp)
TARGET_LINK_LIBRARIES(test_rtime 
    rtdb2 
    rutils
)

