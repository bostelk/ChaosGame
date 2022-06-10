mkdir lib
mkdir lib\simple
mkdir lib\simple\release
ispc src/simple.ispc -h src/simple.h -o lib/simple/release/simple.obj --arch=x86-64 --target=avx1 --target-os=windows
