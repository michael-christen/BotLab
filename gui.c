//////////////
// INCLUDES
//////////////

// C Libraries
#include "maebot_app.h"
#include <stdio.h>
#include <unistd.h>
#include <pthread.h>
#include <stdlib.h>
#include <lcm/lcm.h>
#include <signal.h>

// VX
#include "vx/vx.h"
#include "vx/vxo_drawables.h"
#include "vx/vx_remote_display_source.h"

// EECS 467 Libraries
#include "common/getopt.h"
#include "common/image_util.h"
#include "common/timestamp.h"
#include "imagesource/image_source.h"
#include "imagesource/image_convert.h"

void display_finished(vx_application_t * app, vx_display_t * disp)
{
    state_t * state = app->impl;
    pthread_mutex_lock(&state->layer_mutex);

    vx_layer_t * layer = NULL;

    // store a reference to the world and layer that we associate with each vx_display_t
    zhash_remove(state->layer_map, &disp, NULL, &layer);

    vx_layer_destroy(layer);

    pthread_mutex_unlock(&state->layer_mutex);
}

void display_started(vx_application_t * app, vx_display_t * disp)
{
    state_t * state = app->impl;

    vx_layer_t * layer = vx_layer_create(state->vw);
    vx_layer_set_display(layer, disp);
    vx_layer_add_event_handler(layer, &state->veh);

    pthread_mutex_lock(&state->layer_mutex);
    // store a reference to the world and layer that we associate with each vx_display_t
    zhash_put(state->layer_map, &disp, &layer, NULL, NULL);
    pthread_mutex_unlock(&state->layer_mutex);
}

int renderCameraPOVLayer(void *data) {
    state_t * state = data;

    if (state->getopt_options.verbose) {
        printf("Starting run_camera\n");
    }

    image_source_t *isrc = state->isrc;
    image_u32_t *im = NULL;
    image_source_data_t isdata;

    int res = isrc->get_frame(isrc, &isdata);
    if (!res) {
        im = image_convert_u32(&isdata);
    } else {
        return 0;
    }

    isrc->release_frame(isrc, &isdata);

    if (state->getopt_options.verbose) {
        printf("Got frame %p\n", im);
    }

    if (im != NULL) {
        double decimate = state->getopt_options.decimate;

        if (decimate != 1.0) {
            image_u32_t * im2 = image_util_u32_decimate(im, decimate);
            image_u32_destroy(im);
            im = im2;
        }

        vx_object_t * vo = vxo_image_from_u32(im, VXO_IMAGE_FLIPY, VX_TEX_MIN_FILTER);

        // show downsampled image, but scale it so it appears the
        // same size as the original
        vx_buffer_t *vb = vx_world_get_buffer(state->vw, "image");
        vx_buffer_add_back(vb, vxo_pix_coords(VX_ORIGIN_TOP_LEFT,
                                              vxo_chain (vxo_mat_scale(decimate),
                                                         vxo_mat_translate3 (0, -im->height, 0),
                                                         vo)));
        vx_buffer_swap(vb);
    }

    image_u32_destroy(im);

    return 1;
}

int initCameraPOVLayer(void *data) {
    state_t * state = data;

    if (!state->getopt_options.no_video) {
        const zarray_t *args = getopt_get_extra_args(state->gopt);
        if (zarray_size(args) > 0) {
            zarray_get(args, 0, &state->url);
        } else {
            zarray_t *urls = image_source_enumerate();

            printf("Cameras:\n");
            for (int i = 0; i < zarray_size(urls); i++) {
                char *url;
                zarray_get(urls, i, &url);
                printf("  %3d: %s\n", i, url);
            }

            if (zarray_size(urls) == 0) {
                printf("No cameras found.\n");
                return 1;
            }
            zarray_get(urls, 0, &state->url);
        }

        state->isrc = image_source_open(state->url);
        if (state->isrc == NULL) {
            printf("Unable to open device %s\n", state->url);
            return 1;
        }

        image_source_t *isrc = state->isrc;

        if (isrc->start(isrc)) {
            return 1;
        }
    }

    return 0;
}

void destroyCameraPOVLayer(void *data) {
    state_t * state = data;

    if (!state->getopt_options.no_video) {
        state->isrc->close(state->isrc);
    }
}

void* renderLayers(void *data) {
    state_t * state = data;

    // initWorldLayer(&state);

    if (!initCameraPOVLayer(&state)) {
        printf("Failed to init CameraPOVLayer");
        return NULL;
    }

    // initWorldPOVLayer(&state);

    while(state->running) {
        // renderWorldLayer(&state);
        renderCameraPOVLayer(&state);
        // renderWorldPOVLayer(&state);
    }

    // destroyWorldLayer(&state);
    destroyCameraPOVLayer(&state);
    // destroyWorldPOVLayer(&state);

    return NULL;
}

void* gui_create(void *data) {
    state_t * state = data;

    vx_remote_display_source_attr_t remote_attr;
    vx_remote_display_source_attr_init(&remote_attr);
    remote_attr.max_bandwidth_KBs = state->getopt_options.limitKBs;
    remote_attr.advertise_name = "Maebot Teleop";

    vx_remote_display_source_t * remote = vx_remote_display_source_create_attr(&state->app, &remote_attr);

    renderLayers(data);

    vx_remote_display_source_destroy(remote);

    return NULL;
}