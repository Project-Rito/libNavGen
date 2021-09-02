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
