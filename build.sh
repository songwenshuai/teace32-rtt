clear
git clean -xfd
cmake -Bbuild -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=./cmake/linux.cmake -GNinja .
cmake --build ./build