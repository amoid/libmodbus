export CC=arm-linux-gnueabihf-gcc
./autogen.sh --host=arm-linux
./configure --prefix=$(pwd)/_install --host=arm-linux
make 
make install

