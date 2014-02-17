include ../common.mk

CFLAGS = $(CFLAGS_COMMON) $(CFLAGS_GLIB) $(CFLAGS_VX) $(CFLAGS_GTK) $(CFLAGS_STD) -msse2 -fPIC -O4 -g
LDFLAGS = $(LDFLAGS_VX) $(LDFLAGS_VX_GTK) $(LDFLAGS_IMAGESOURCE) $(LDFLAGS_COMMON) $(LDFLAGS_GLIB) $(LDFLAGS_GTK) $(LDFLAGS_LCMTYPES) $(LDFLAGS_STD) 

MAEBOT_APP = ../../bin/maebot_app
LIB = ../../lib

all: $(MAEBOT_APP)

$(MAEBOT_APP): maebot_app.o gui.o
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS)

clean:
	@rm -f *.o *~ *.a
	@rm -f $(MAEBOT_APP)
