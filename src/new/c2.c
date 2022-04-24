// https://gist.github.com/bellbind/6813905
// capturing from UVC cam using video4linux2
// build: cc c2.c -o c2

#include <stdbool.h> // true, false
#include <stdint.h> // uint8_t, uint32_t
#include <stdlib.h> // exit, malloc, calloc, free
#include <stdio.h> // FILE, fprintf, stderr
#include <string.h> // strerror, memset, memcpy
#include <errno.h> // errno, EINTR

#include <fcntl.h> // open
#include <unistd.h> // close
#include <sys/ioctl.h> // ioctl
#include <sys/mman.h> // mmap, munmap

#include <linux/videodev2.h> // v4l2_*, V4L2_*

static void quit(const char * msg)
{
	fprintf(stderr, "[%s] %d: %s\n", msg, errno, strerror(errno));
	exit(1);
}

// keep trying to ioctl until success (or too many trials)
static int xioctl(int fd, int request, void* arg)
{
	for (int i = 0; i < 100; i++) {
		int r = ioctl(fd, request, arg);
		if (r != -1 || errno != EINTR)
		{
			fprintf(stderr, "xioctl req=%d, i=%d r=%d\n",
					request, i, r);
			return r;
		}
	}
	fprintf(stderr, "xioctl req=%d, i=%d r=%d\n", request, -1, -1);
	return -1;
}

// used only inside the camera struct
struct buffer_t {
	uint8_t* start;
	size_t length;
};

// internal struct, just for this program
struct camera_t {
	int fd;
	uint32_t width;
	uint32_t height;
	size_t buffer_count;
	struct buffer_t* buffers;
	struct buffer_t head;
};

struct camera_t* camera_open(
		const char *device,
	       	uint32_t width,
		uint32_t height
		)
{
	int fd = open(device, O_RDWR | O_NONBLOCK, 0);
	if (fd == -1) quit("open");
	struct camera_t *c = malloc(sizeof*c);
	c->fd = fd;
	c->width = width;
	c->height = height;
	c->buffer_count = 0;
	c->buffers = NULL;
	c->head.length = 0;
	c->head.start = NULL;
	return c;
}

void camera_init(struct camera_t* c)
{
	struct v4l2_capability cap;
	if (xioctl(c->fd, VIDIOC_QUERYCAP, &cap) == -1)
		quit("VIDIOC_QUERYCAP");
	if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE))
		quit("no capture");
	if (!(cap.capabilities & V4L2_CAP_STREAMING))
		quit("no streaming");

	struct v4l2_cropcap cropcap;
	memset(&cropcap, 0, sizeof cropcap);
	cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if (xioctl(c->fd, VIDIOC_CROPCAP, &cropcap) == 0) {
		struct v4l2_crop crop;
		crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		crop.c = cropcap.defrect;
		if (xioctl(c->fd, VIDIOC_S_CROP, &crop) == -1) {
			// cropping not supported
		}
	}

	struct v4l2_format format;
	memset(&format, 0, sizeof format);
	format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	format.fmt.pix.width = c->width;
	format.fmt.pix.height = c->height;
	format.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
	format.fmt.pix.field = V4L2_FIELD_NONE;
	if (xioctl(c->fd, VIDIOC_S_FMT, &format) == -1)
		quit("VIDIOC_S_FMT");

	struct v4l2_requestbuffers req;
	memset(&req, 0, sizeof req);
	req.count = 4;
	req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	req.memory = V4L2_MEMORY_MMAP;
	if (xioctl(c->fd, VIDIOC_REQBUFS, &req) == -1)
		quit("VIDIOC_REQBUFS");
	c->buffer_count = req.count;
	c->buffers = calloc(req.count, sizeof (struct buffer_t));

	size_t buf_max = 0;
	for (size_t i = 0; i < c->buffer_count; i++) {
		struct v4l2_buffer buf;
		memset(&buf, 0, sizeof buf);
		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_MMAP;
		buf.index = i;
		if (xioctl(c->fd, VIDIOC_QUERYBUF, &buf) == -1)
			quit("VIDIOC_QUERYBUF");
		if (buf.length > buf_max) buf_max = buf.length;
		c->buffers[i].length = buf.length;
		c->buffers[i].start = 
			mmap(NULL, buf.length, PROT_READ | PROT_WRITE,
					MAP_SHARED, c->fd, buf.m.offset);
		if (c->buffers[i].start == MAP_FAILED) quit("mmap");
	}
	c->head.start = malloc(buf_max);
}

void camera_start(struct camera_t* c)
{
	for (size_t i = 0; i < c->buffer_count; i++) {
		struct v4l2_buffer buf;
		memset(&buf, 0, sizeof buf);
		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_MMAP;
		buf.index = i;
		if (xioctl(c->fd, VIDIOC_QBUF, &buf) == -1)
			quit("VIDIOC_QBUF");
	}

	enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if (xioctl(c->fd, VIDIOC_STREAMON, &type) == -1)
		quit("VIDIOC_STREAMON");
}

void camera_stop(struct camera_t* c)
{
	enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if (xioctl(c->fd, VIDIOC_STREAMOFF, &type) == -1)
		quit("VIDIOC_STREAMOFF");
}

void camera_finish(struct camera_t* c)
{
	for (size_t i = 0; i < c->buffer_count; i++) {
		munmap(c->buffers[i].start, c->buffers[i].length);
	}
	free(c->buffers);
	c->buffer_count = 0;
	c->buffers = NULL;
	free(c->head.start);
	c->head.length = 0;
	c->head.start = NULL;
}

void camera_close(struct camera_t* c)
{
	if (close(c->fd) == -1)
		quit("close");
	free(c);
}

int camera_capture(struct camera_t* c)
{
	struct v4l2_buffer buf;
	memset(&buf, 0, sizeof buf);
	buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buf.memory = V4L2_MEMORY_MMAP;
	if (xioctl(c->fd, VIDIOC_DQBUF, &buf) == -1)
		return false;
	memcpy(c->head.start, c->buffers[buf.index].start, buf.bytesused);
	c->head.length = buf.bytesused;
	if (xioctl(c->fd, VIDIOC_QBUF, &buf) == -1)
		return false;
	return true;
}

int camera_frame(struct camera_t* c, struct timeval timeout)
{
	fd_set fds;
	FD_ZERO(&fds);
	FD_SET(c->fd, &fds);
	int r = select(c->fd + 1, &fds, 0, 0, &timeout);
	if (r == -1) quit("select");
	if (r == 0) return false;
	return camera_capture(c);
}

static void ppm(FILE* dest, uint8_t* rgb, uint32_t w, uint32_t h)
{
	fprintf(dest, "P6\n%d %d 255\n", w, h);
	fwrite(rgb, 3, w*h, dest);
}

static int bclamp(int v)
{
	if (v < 0) return 0;
	if (v > 255) return 255;
	return v;
}

static uint8_t* yuyv2rgb(uint8_t* yuyv, uint32_t width, uint32_t height)
{
	uint8_t* rgb = calloc(width * height * 3, sizeof (uint8_t));
	for (size_t i = 0; i < height; i++) {
		for (size_t j = 0; j < width; j += 2) {
			size_t ij = i * width + j;
			int y0 = yuyv[ij * 2 + 0] << 8;
			int u = yuyv[ij * 2 + 1] - 128;
			int y1 = yuyv[ij * 2 + 2] << 8;
			int v = yuyv[ij * 2 + 3] - 128;
			rgb[3*ij + 0 + 0] = bclamp((y0 + 359*v) >> 8);
			rgb[3*ij + 0 + 3] = bclamp((y1 + 359*v) >> 8);
			rgb[3*ij + 1 + 0] = bclamp((y0 + 88*v - 183*u) >> 8);
			rgb[3*ij + 1 + 3] = bclamp((y1 + 88*v - 183*u) >> 8);
			rgb[3*ij + 2 + 0] = bclamp((y0 + 454*u) >> 8);
			rgb[3*ij + 2 + 3] = bclamp((y1 + 454*u) >> 8);
		}
	}
	return rgb;
}


int main()
{
	struct camera_t* c = camera_open("/dev/video0", 800, 600);
	camera_init(c);
	camera_start(c);

	struct timeval timeout;
	timeout.tv_sec = 1;
	timeout.tv_usec = 0;
	/* skip 5 frames for booting a cam */
	for (int i = 0; i < 5; i++) {
		camera_frame(c, timeout);
	}
	for (int i = 0; i < 100; i++) {
		camera_frame(c, timeout);

		uint8_t *rgb = yuyv2rgb(c->head.start, c->width, c->height);
		char filename[FILENAME_MAX];
		sprintf(filename, "result_%03d.ppm", i);
		FILE* out = fopen(filename, "w");
		ppm(out, rgb, c->width, c->height);
		fclose(out);
		free(rgb);
	}

	camera_stop(c);
	camera_finish(c);
	camera_close(c);
	return 0;
}
