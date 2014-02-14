include ../common.mk

# flags for building the gtk library
CFLAGS = $(CFLAGS_STD) $(CFLAGS_COMMON) $(CFLAGS_VX) $(CFLAGS_GTK) $(CFLAGS_LCMTYPES) $(CFLAGS_LCM) $(CFLAGS_VX) -fPIC -O2
LDFLAGS =  $(LDFLAGS_VX_GTK) $(LDFLAGS_VX) $(LDFLAGS_GTK) $(LDFLAGS_IMAGESOURCE) $(LDFLAGS_COMMON) $(LDFLAGS_LCMTYPES) $(LDFLAGS_LCM) $(LDFLAGS_STD)

BINS=maebot_video_teleop

all: $(BINS)

maebot_video_teleop: maebot_video_teleop.o
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS)


clean:
	@rm -rf  $(BINS) *.o *~
