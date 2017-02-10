CC="gcc-6 -std=gnu99"
CC=clang
WFLAGS="-Wall -Wextra -Wno-sign-compare"

OFLAGS="-Ofast -march=native -DNDEBUG"
OFLAGS="-O0"
OFLAGS="-Ofast -march=native"

CFLAGS="$WFLAGS $OFLAGS"
OCVFLAGS=`pkg-config opencv --cflags --libs`
$CC $CFLAGS camflow.c -o camflow $OCVFLAGS -lm

# only for the version with enabled screenshots
#IIOFLAGS="-ltiff -lpng -ljpeg"
#$CC $CFLAGS -DENABLE_SCREENSHOTS camflow.c iio.o -o camflow $OCVFLAGS $IIOFLAGS -lm
