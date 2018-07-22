./autogen.sh
CFLAGS=-lpthread ./configure --host=arm-fsl-linux-gnueabi --prefix=${PWD}/.install
make clean
make 
make install

