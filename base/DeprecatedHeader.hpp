#if(ROCK_RELEASE_ID && ROCK_RELEASE_ID > ROCK_DEPRECATED_HEADER_SINCE)
#error "The header including DeprecatedHeader.hpp is obsolete and will be removed in the next Rock release. You can still use it by setting the CMake ROCK_USE_OBSOLETE_HEADERS define to 1, but even that option will be removed in the next Rock release"
#elif ROCK_RELEASE_ID
#error "The header including DeprecatedHeader.hpp is deprecated. This warning will be changed into an error in the next Rock release, and the header will be removed one release after that"
#else
#error "The header including DeprecatedHeader.hpp is deprecated"
#endif

