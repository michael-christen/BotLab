#ifndef PATH_H
#define PATH_H

typedef struct position_t position_t;
typedef struct path_t path_t;

struct position_t {
    double x, y;
};

struct path_t {
    unsigned int length, position; 
	 double distance;
    position_t *waypoints;
};

void path_destroy(path_t *path);

#endif
