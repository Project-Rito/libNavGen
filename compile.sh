#!/bin/bash

cd $(dirname $0)

cmake -S . -B ./build/ &&
	make -C ./build/ &&
	cp ./build/libNavGen.so ./compiled/

rm -rf ./build/

cmake -S . -B ./build/ -DCMAKE_TOOLCHAIN_FILE=./cmake/toolchains/mingw-linux.cmake &&
	make -C ./build/ &&
	cp ./build/libNavGen.dll ./compiled/

rm -rf ./build/

cp /usr/x86_64-w64-mingw32/bin/libgcc_s_seh-1.dll ./compiled/
cp /usr/x86_64-w64-mingw32/bin/libgomp-1.dll ./compiled/
cp /usr/x86_64-w64-mingw32/bin/libstdc++-6.dll ./compiled/
cp /usr/x86_64-w64-mingw32/bin/libwinpthread-1.dll ./compiled/
