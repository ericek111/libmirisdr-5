# SoapySDR module, built when SoapySDR is found
enable_language(CXX)

SOAPY_SDR_MODULE_UTIL(
    TARGET mirisdrSupport
    SOURCES Registration.cpp Settings.cpp Streaming.cpp
    LIBRARIES mirisdr_shared
)
target_include_directories(mirisdrSupport PRIVATE ${CMAKE_SOURCE_DIR}/include ${CMAKE_BINARY_DIR}/include)
