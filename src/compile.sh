CC=gcc-6
CFLAGS="-O3 -ffast-math"
OCVFLAGS=`pkg-config opencv --cflags --libs`
$CC $CFLAGS camflow.c -o camflow $OCVFLAGS
