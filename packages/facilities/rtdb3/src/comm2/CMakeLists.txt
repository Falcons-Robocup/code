FIND_PACKAGE(Boost COMPONENTS system filesystem thread REQUIRED)


ADD_LIBRARY(comm2lib SHARED comm2.cpp timer.cpp statistics.cpp MulticastSocket.cpp)
SET_TARGET_PROPERTIES(comm2lib PROPERTIES COMPILE_FLAGS "-fPIC")
TARGET_LINK_LIBRARIES(comm2lib rtdb2 rutils)

SET(comm2_SRC
    comm2_main.cpp
)

SET(comm2_OBJ comm2)
ADD_EXECUTABLE(${comm2_OBJ} ${comm2_SRC})
TARGET_LINK_LIBRARIES(${comm2_OBJ} 
    rtdb2 
    comm2lib
    rutils
    ${Boost_FILESYSTEM_LIBRARY} 
    ${Boost_THREAD_LIBRARY} 
    ${Boost_SYSTEM_LIBRARY}
)

SET_TARGET_PROPERTIES(${comm2_OBJ} PROPERTIES OUTPUT_NAME comm2)


