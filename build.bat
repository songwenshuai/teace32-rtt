git clean -xfd
cmake -AWin32 -Bbuild -DCMAKE_TOOLCHAIN_FILE=./cmake/win32.cmake .
cmake --build ./build --config Release