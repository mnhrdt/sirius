CC=gcc-6
WFLAGS="-Wall -Wextra -Wno-unused -Wno-sign-compare"
OFLAGS="-g -O3 -ffast-math"
CFLAGS="-std=gnu99 $WFLAGS $OFLAGS"
OCVFLAGS=`pkg-config opencv --cflags --libs`
$CC $CFLAGS camflow.c -o camflow $OCVFLAGS
