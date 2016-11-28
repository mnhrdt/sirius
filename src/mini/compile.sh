CC=gcc-6
WFLAGS="-Wall -Wextra -Wno-unused -Wno-sign-compare"
OFLAGS="-O3 -ffast-math -march=native -mtune=native"
CFLAGS="-std=gnu99 $WFLAGS $OFLAGS"
OCVFLAGS=`pkg-config opencv --cflags --libs`
IIOFLAGS="-ltiff -lpng -ljpeg"
#$CC $CFLAGS camflow.c iio.o -o camflow $OCVFLAGS -lm
$CC $CFLAGS -DENABLE_SCREENSHOTS camflow.c iio.o -o camflow $OCVFLAGS $IIOFLAGS -lm
