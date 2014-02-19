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
#include "common/matd.h"
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
        if (layerData->enable == 1) {

            vx_layer_t * layer = vx_layer_create(layerData->world);

            layerData->layer = layer;

            vx_layer_set_display(layer, disp);

            pthread_mutex_lock(&state->layer_mutex);
            // store a reference to the world and layer that we associate with each vx_display_t
            zhash_put(state->layer_map, &disp, &layer, NULL, NULL);
            pthread_mutex_unlock(&state->layer_mutex);

            layerData->displayInit(state, layerData);
        }
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
		correctDistortion(im, state->lookupTable);

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
    //Draw Grid
    vx_buffer_t *gridBuff = vx_world_get_buffer(layerData->world, "grid");
    vx_buffer_add_back(gridBuff, vxo_grid());
    //Draw Axes
    int npoints = 4;
    float axes[12] = {-1000, 0, 0, 1000, 0, 0, 0, -1000, 0, 0, 1000, 0};
    vx_resc_t *verts = vx_resc_copyf(axes, npoints*3);
    vx_buffer_add_back(gridBuff, vxo_lines(verts, npoints, GL_LINES, vxo_points_style(vx_red, 2.0f)));
    //Draw Bruce
    vx_buffer_t *bruceBuff = vx_world_get_buffer(layerData->world, "bruce");
    vx_object_t *vo = vxo_chain(
                                vxo_mat_translate3(state->pos_x, state->pos_y - BRUCE_LENGTH/2, state->pos_z),
                                vxo_mat_rotate_z(-state->pos_theta),
                                vxo_mat_rotate_x(-M_PI/2),
				vxo_mat_scale3(BRUCE_WIDTH, BRUCE_HEIGHT, BRUCE_LENGTH),
                                vxo_square_pyramid(vxo_mesh_style(vx_blue),
                                                    vxo_lines_style(vx_cyan, 2.0f))                                 
                                );
    vx_buffer_add_back(bruceBuff, vo);

    //Draw Gaussian Ellipse
    //95% confidence ellipse from 1-sigma error ellipse
    double scalefactor = 2.4477;
    //just for show now
    scalefactor = 10;
    double covX = 1.0;
    double covY = 0.5;
    matd_t *covMatrix = matd_create(2,2);
    matd_put(covMatrix, 0, 0, pow(covX, 2));
    matd_put(covMatrix, 0, 1, covX * covY);
    matd_put(covMatrix, 1, 0, pow(covY, 2));
    matd_put(covMatrix, 1, 1, covY * covX);
    //Calculate Eigen Values with quadratic formula
    double trace = matd_get(covMatrix, 0,0) + matd_get(covMatrix, 1,1);
    double gap   = sqrt(pow(trace,2) - 4 * matd_det(covMatrix));
    double eigX = (trace + gap)/2;
    double eigY = (trace - gap)/2;
    matd_destroy(covMatrix);
    double min_eig = (eigX < eigY) ? eigX : eigY;
    double max_eig = (eigX == min_eig) ? eigY : eigX;
    //Semi-major -> x, Semi-minor -> y, axes
    //lengths = sqrt(eigenvalues), larger eigenvalue -> larger uncertainty
    //therefore
    double semi_major_length = sqrt(max_eig) * scalefactor;
    double semi_minor_length = sqrt(min_eig) * scalefactor;
    double x_length, y_length;
    double aspect_ratio      = 0.5;
    double sig_xy = 0.5, sig_x = 0.25, sig_y = 0.125;
    if(covX > covY) {
	x_length = semi_major_length;
	y_length = semi_minor_length;
    } else {
	x_length = semi_minor_length;
	y_length = semi_major_length;
    }
    //rotate phi ccw from original orientation
    double phi = 1/2 * atan( 
	    (1/aspect_ratio) * 
	    ( (2*sig_xy) / 
	      ( pow(sig_x,2)-pow(sig_y,2) ) 
	    )
    );
    phi = M_PI/2;

    npoints = 35;
    float points[npoints*3];
    for (int i = 0; i < npoints; i++) {
	float angle = 2*M_PI*i/npoints;

	float x = x_length*cosf(angle);
	float y = y_length*sinf(angle);
	float z = 0.0f;

	points[3*i + 0] = x;
	points[3*i + 1] = y;
	points[3*i + 2] = z;
    }
    vx_buffer_t *ellipseBuff = vx_world_get_buffer(layerData->world,
	    "error_ellipse");
    vo = vxo_chain(
	vxo_mat_translate3(
	    state->pos_x, 
	    state->pos_y, 
	    state->pos_z
	),
	vxo_mat_rotate_z(phi - state->pos_theta),
	vxo_lines( 
	    vx_resc_copyf(points, npoints*3),
	    npoints,
	    GL_LINE_LOOP,
	    vxo_lines_style(vx_purple, 1.0f)
	)
    );
    vx_buffer_add_back(ellipseBuff, vo);

    //Swap buffers
    vx_buffer_swap(gridBuff);
    vx_buffer_swap(bruceBuff);
    vx_buffer_swap(ellipseBuff);
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

int initDebugLayer(state_t *state, layer_data_t *layerData) {
    layerData->world = vx_world_create();
    return 1;
}

int displayInitDebugLayer(state_t *state, layer_data_t *layerData) {
    //const double eye[3] = {state->pos_x, state->pos_y, state->pos_z};
    //const double lookat[3] = {state->pos_x, state->pos_y, state->pos_z};
    //const double up[3] = {0, 0, 1};
    float black[4] = {0.0f, 0.0f, 0.0f, 1.0f};
    vx_layer_set_background_color(layerData->layer, black);
    vx_layer_set_viewport_rel(layerData->layer, layerData->position);
    vx_layer_add_event_handler(layerData->layer, &state->veh);
    //vx_layer_camera_lookat(layerData->layer, &eye, &lookat, &up, 1);
    return 1;
}

int renderDebugLayer(state_t *state, layer_data_t *layerData) {
    vx_buffer_t *textBuff = vx_world_get_buffer(layerData->world, "text");

    char debugText[80];
    const char* formatting = "<<left,#ffffff,serif>>X: %f\nY: %f\nTheta: %f";
    sprintf(debugText, formatting, state->pos_x, state->pos_y, state->pos_theta);
    vx_object_t *vo = vxo_text_create(VXO_TEXT_ANCHOR_TOP_LEFT, debugText);
    vx_buffer_add_back(textBuff, vxo_pix_coords(VX_ORIGIN_TOP_LEFT, vo));
    vx_buffer_swap(textBuff);
    return 1;
}

int destroyDebugLayer(state_t *state, layer_data_t *layerData) {
    return 1;
}

void* renderLayers(state_t *state) {
    int i;

    // Init
    printf("Entering layer init\n");
    for (i = 0; i < NUM_LAYERS; i++) {
        layer_data_t *layer = &(state->layers[i]);
        if (layer->enable == 1) {
            if (layer->init(state, layer) == 0) {
                printf("Failed to init layer: %s\n", layer->name);
                return NULL;
            }
        }
    }
    printf("Entering render loop\n");
    // Render Loop
    while(state->running) {
        for (i = 0; i < NUM_LAYERS; i++) {
            layer_data_t *layer = &(state->layers[i]);
            //printf("layer %d enable %d\n", i, layer->enable);
            if (layer->enable == 1) {
                if (layer->render(state, layer) == 0) {
                    printf("Failed to render layer: %s\n", layer->name);
                    return NULL;
                }
            }
        }
    }
    printf("Entering layer destroy\n");
    // Destroy/Clean up
    for (i = 0; i < NUM_LAYERS; i++) {
        layer_data_t *layer = &(state->layers[i]);
        if (layer->enable == 1) {
            if (layer->destroy(state, layer) == 0) {
                printf("Failed to destroy layer: %s\n", layer->name);
                return NULL;
            }
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


    //state->layers[1].enable = !state->getopt_options.no_video;
    state->layers[1].enable = 1;
    state->layers[1].name = "CameraPOV";
    state->layers[1].position[0] = 0.666f;
    state->layers[1].position[1] = 0.5f;
    state->layers[1].position[2] = 0.333f;
    state->layers[1].position[3] = 0.5f;
    state->layers[1].init = initCameraPOVLayer;
    state->layers[1].displayInit = displayInitCameraPOVLayer;
    state->layers[1].render = renderCameraPOVLayer;
    state->layers[1].destroy = destroyCameraPOVLayer;

    state->layers[2].enable = 0;
    state->layers[2].name = "WorldPOV";
    state->layers[2].position[0] = 0.666f;
    state->layers[2].position[1] = 0;
    state->layers[2].position[2] = 0.333f;
    state->layers[2].position[3] = 0.5f;
    state->layers[2].init = initWorldPOVLayer;
    state->layers[2].displayInit = displayInitWorldPOVLayer;
    state->layers[2].render = renderWorldPOVLayer;
    state->layers[2].destroy = destroyWorldPOVLayer;

    state->layers[3].enable = 1;
    state->layers[3].name = "Debug";
    state->layers[3].position[0] = 0;
    state->layers[3].position[1] = 0;
    state->layers[3].position[2] = 0.666f;
    state->layers[3].position[3] = 0.333f;
    state->layers[3].init = initDebugLayer;
    state->layers[3].displayInit = displayInitDebugLayer;
    state->layers[3].render = renderDebugLayer;
    state->layers[3].destroy = destroyDebugLayer;

    vx_remote_display_source_t * remote = vx_remote_display_source_create_attr(&state->app, &remote_attr);

    // Handles layer init, rendering, and destruction
    renderLayers(state);

    vx_remote_display_source_destroy(remote);

    return NULL;
}
