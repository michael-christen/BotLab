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
#include "vx/vx_types.h"
#include "vx/vx_event.h"
#include "vx/vx_event_handler.h"
#include "vx/default_camera_mgr.h"

// EECS 467 Libraries
#include "common/getopt.h"
#include "common/image_util.h"
#include "common/timestamp.h"
#include "imagesource/image_source.h"
#include "imagesource/image_convert.h"

#include "blob_detection.h"

ball_t balls[MAX_NUM_BALLS];
int num_balls;

void display_finished(vx_application_t * app, vx_display_t * disp)
{
    zhash_iterator_t it;
    vx_layer_t *value;

    state_t * state = app->impl;

    //printf("disp end: %d\n", disp);

    zhash_iterator_init(state->layer_map, &it);

    pthread_mutex_lock(&state->layer_mutex);

    while (zhash_iterator_next(&it, &disp, &value)) {
        vx_layer_destroy(value);
        zhash_iterator_remove(&it);
    }

    pthread_mutex_unlock(&state->layer_mutex);

    printf("hash table size after remove: %d\n", zhash_size(state->layer_map));
}

void display_started(vx_application_t * app, vx_display_t * disp)
{
    int i;

    state_t * state = app->impl;

    //printf("disp start: %d\n", disp);

    for (i = 0; i < NUM_LAYERS; i++) {
        layer_data_t *layerData = &(state->layers[i]);

        vx_layer_t * layer = vx_layer_create(layerData->world);

        layerData->layer = layer;

        vx_layer_set_display(layer, disp);

        pthread_mutex_lock(&state->layer_mutex);
        // store a reference to the world and layer that we associate with each vx_display_t
        zhash_put(state->layer_map, &disp, &layer, NULL, NULL);
        pthread_mutex_unlock(&state->layer_mutex);

        layerData->displayInit(state, layerData);
    }
    printf("hash table size after insert: %d\n", zhash_size(state->layer_map));
}

int initCameraPOVLayer(state_t *state, layer_data_t *layerData) {
    layerData->world = vx_world_create();

    /*
    const zarray_t *args = getopt_get_extra_args(state->gopt);
    if (zarray_size(args) > 0) {
        zarray_get(args, 0, &state->url);
    } else {*/
    zarray_t *urls = image_source_enumerate();

    printf("Cameras:\n");
    for (int i = 0; i < zarray_size(urls); i++) {
        char *url;
        zarray_get(urls, i, &url);
        printf("  %3d: %s\n", i, url);
    }

    if (zarray_size(urls) == 0) {
        printf("No cameras found.\n");
        return 0;
    }
    zarray_get(urls, 0, &state->url);
    //}

    state->isrc = image_source_open(state->url);
    if (state->isrc == NULL) {
        printf("Unable to open device %s\n", state->url);
        return 0;
    }

    image_source_t *isrc = state->isrc;

    if (isrc->start(isrc)) {
        printf("Can't start image source\n");
        return 0;
    }

    return 1;
}

int displayInitCameraPOVLayer(state_t *state, layer_data_t *layerData) {
    image_source_format_t isrc_format;
    state->isrc->get_format(state->isrc, 0, &isrc_format);

    float lowLeft[2] = {0, 0};
    float upRight[2] = {isrc_format.width, isrc_format.height};

    vx_layer_camera_fit2D(layerData->layer, lowLeft, upRight, 1);
    vx_layer_set_viewport_rel(layerData->layer, layerData->position);
    vx_layer_add_event_handler(layerData->layer, &state->veh);
    return 1;
}

int renderCameraPOVLayer(state_t *state, layer_data_t *layerData) {
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

        num_balls = blob_detection(im, balls);
        vx_object_t * vo = vxo_image_from_u32(im, VXO_IMAGE_FLIPY,
		VX_TEX_MIN_FILTER | VX_TEX_MAG_FILTER);

        // show downsampled image, but scale it so it appears the
        // same size as the original
        vx_buffer_t *vb = vx_world_get_buffer(layerData->world, "image");

        vx_buffer_add_back(vb, vo);
        vx_buffer_swap(vb);
    }
    
    image_u32_destroy(im);

    return 1;
}

int destroyCameraPOVLayer(state_t *state, layer_data_t *layerData) {
    if (!state->getopt_options.no_video) {
        state->isrc->close(state->isrc);
    }

    vx_world_destroy(layerData->world);
    return 1;
}

int initWorldTopDownLayer(state_t *state, layer_data_t *layerData) {
    layerData->world = state->vw;
    return 1;
}

int displayInitWorldTopDownLayer(state_t *state, layer_data_t *layerData) {
    vx_layer_set_viewport_rel(layerData->layer, layerData->position);
    vx_layer_add_event_handler(layerData->layer, &state->veh);
    return 1;
}

int renderWorldTopDownLayer(state_t *state, layer_data_t *layerData) {
    vx_buffer_t *gridBuff = vx_world_get_buffer(layerData->world, "grid");
    vx_buffer_add_back(gridBuff, vxo_grid());

    vx_buffer_t *bruceBuff = vx_world_get_buffer(layerData->world, "bruce");
    vx_object_t *vo = vxo_chain(
                                vxo_mat_translate3(state->pos_x, state->pos_y, state->pos_z),
                                vxo_mat_scale3(BRUCE_DIAMETER, BRUCE_DIAMETER, BRUCE_HEIGHT),
                                vxo_cylinder(vxo_mesh_style(vx_blue))                                    
                                );
    vx_buffer_add_back(bruceBuff, vo);

    //vx_buffer_swap(gridBuff);
    vx_buffer_swap(bruceBuff);
    return 1;
}

int destroyWorldTopDownLayer(state_t *state, layer_data_t *layerData) {
    return 1;
}

int initWorldPOVLayer(state_t *state, layer_data_t *layerData) {
    layerData->world = state->vw;
    return 1;
}

int displayInitWorldPOVLayer(state_t *state, layer_data_t *layerData) {
    //const double eye[3] = {state->pos_x, state->pos_y, state->pos_z};
    //const double lookat[3] = {state->pos_x, state->pos_y, state->pos_z};
    //const double up[3] = {0, 0, 1};
    vx_layer_set_viewport_rel(layerData->layer, layerData->position);
    vx_layer_add_event_handler(layerData->layer, &state->veh);
    //vx_layer_camera_lookat(layerData->layer, &eye, &lookat, &up, 1);
    return 1;
}

int renderWorldPOVLayer(state_t *state, layer_data_t *layerData) {
    return 1;
}

int destroyWorldPOVLayer(state_t *state, layer_data_t *layerData) {
    return 1;
}

void* renderLayers(state_t *state) {
    int i;

    // Init
    for (i = 0; i < NUM_LAYERS; i++) {
        layer_data_t *layer = &(state->layers[i]);
        if (!layer->init(state, layer)) {
            printf("Failed to init layer: %s\n", layer->name);
            return NULL;
        }
    }

    // Render Loop
    while(state->running) {
        for (i = 0; i < NUM_LAYERS; i++) {
            layer_data_t *layer = &(state->layers[i]);
            if (layer->enable && !layer->render(state, layer)) {
                printf("Failed to render layer: %s\n", layer->name);
                return NULL;
            }
        }
    }

    // Destroy/Clean up
    for (i = 0; i < NUM_LAYERS; i++) {
        layer_data_t *layer = &(state->layers[i]);
        if (!layer->destroy(state, layer)) {
            printf("Failed to destroy layer: %s\n", layer->name);
            return NULL;
        }
    }

    return NULL;
}

// Main Pthread  GUI funtion
void* gui_create(void *data) {
    state_t * state = data;

    vx_remote_display_source_attr_t remote_attr;
    vx_remote_display_source_attr_init(&remote_attr);
    remote_attr.max_bandwidth_KBs = state->getopt_options.limitKBs;
    remote_attr.advertise_name = "Maebot Teleop";


    // Init layer data structs
    state->layers[0].enable = 1;
    state->layers[0].name = "WorldTopDown";
    state->layers[0].position[0] = 0;
    state->layers[0].position[1] = 0.333f;
    state->layers[0].position[2] = 0.666f;
    state->layers[0].position[3] = 0.666f;
    state->layers[0].init = initWorldTopDownLayer;
    state->layers[0].displayInit = displayInitWorldTopDownLayer;
    state->layers[0].render = renderWorldTopDownLayer;
    state->layers[0].destroy = destroyWorldTopDownLayer;


    state->layers[1].enable = !state->getopt_options.no_video;
    state->layers[1].name = "CameraPOV";
    state->layers[1].position[0] = 0.666f;
    state->layers[1].position[1] = 0.5f;
    state->layers[1].position[2] = 0.333f;
    state->layers[1].position[3] = 0.5f;
    state->layers[1].lowLeft[0] = 0;
    state->layers[1].lowLeft[1] = 0;
    state->layers[1].upRight[0] = 300;
    state->layers[1].upRight[1] = 200;
    state->layers[1].init = initCameraPOVLayer;
    state->layers[1].displayInit = displayInitCameraPOVLayer;
    state->layers[1].render = renderCameraPOVLayer;
    state->layers[1].destroy = destroyCameraPOVLayer;

    state->layers[2].name = "WorldPOV";
    state->layers[2].position[0] = 0.666f;
    state->layers[2].position[1] = 0;
    state->layers[2].position[2] = 0.333f;
    state->layers[2].position[3] = 0.5f;
    state->layers[2].init = initWorldPOVLayer;
    state->layers[2].displayInit = displayInitWorldPOVLayer;
    state->layers[2].render = renderWorldPOVLayer;
    state->layers[2].destroy = destroyWorldPOVLayer;

    vx_remote_display_source_t * remote = vx_remote_display_source_create_attr(&state->app, &remote_attr);

    // Handles layer init, rendering, and destruction
    renderLayers(state);

    vx_remote_display_source_destroy(remote);

    return NULL;
}
