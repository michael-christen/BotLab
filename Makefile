include ../common.mk

CFLAGS = $(CFLAGS_STD) $(CFLAGS_COMMON) $(CFLAGS_VX) $(CFLAGS_GTK) $(CFLAGS_LCMTYPES) $(CFLAGS_LCM) $(CFLAGS_VX) -fPIC -O2
LDFLAGS =  $(LDFLAGS_VX_GTK) $(LDFLAGS_VX) $(LDFLAGS_GTK) $(LDFLAGS_IMAGESOURCE) $(LDFLAGS_COMMON) $(LDFLAGS_LCMTYPES) $(LDFLAGS_LCM) $(LDFLAGS_STD)

BINARIES = ../../bin/maebot_app
LIB = ../../lib

all: $(BINARIES)

drive_test: ../../bin/drive_test

../../bin/maebot_app: maebot_app.o gui.o disjoint.o blob_detection.o \
	calibration.o odometry.o barrel_distortion.o drive_ctrl.o \
	pid_ctrl.o haz_map.o world_map.o image.o line_detection.o mapping.o \
	pixel.o explorer.o path.o map.o
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS)

../../bin/color_app: color_app.o color_gui.o image.o
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS)

../../bin/drive_test: drive_ctrl.o test_drive_ctrl.o pid_ctrl.o
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS)

clean:
	@rm -f *.o *~ *.a
	@rm -f $(BINARIES)
