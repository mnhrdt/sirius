CFLAGS ?= -march=native -O3

OCVFLAGS = `pkg-config opencv --cflags --libs`
IIOFLAGS = -ltiff -lpng -ljpeg

BIN = camflow harrpoints

default: $(BIN)

camflow: camflow.c harressian.c tracker.c
	$(CC) $(CFLAGS) -o $@ camflow.c $(OCVFLAGS) -lm

harrpoints: harrpoints.c harressian.c tracker.c iio.c
	$(CC) $(CFLAGS) -o $@ harrpoints.c iio.c $(IIOFLAGS) -lm

.PHONY:
clean:
	$(RM) $(BIN)
