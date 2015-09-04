file=$1

g++ `pkg-config --cflags opencv` ${file}.cpp `pkg-config --libs opencv` -o ${file}
