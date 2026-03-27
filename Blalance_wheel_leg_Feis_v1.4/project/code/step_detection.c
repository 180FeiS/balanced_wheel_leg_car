#include "zf_common_headfile.h"
#include "step_detection.h"

step_info_t step_data = {0, 0, 0, 0, 0.0f, 0.0f};

static uint8 image_binary[MT9V03X_H][MT9V03X_W];
static uint16 edge_histogram[MT9V03X_H];
static uint8 threshold = 80;

void step_detection_init(void)
{
    step_data.detected = 0;
    step_data.step_row = 0;
    step_data.step_top_row = 0;
    step_data.step_height_pix = 0;
    step_data.distance_mm = 0.0f;
    step_data.distance_cm = 0.0f;
}

static void image_threshold(void)
{
    for (int i = 0; i < MT9V03X_H; i++)
    {
        for (int j = 0; j < MT9V03X_W; j++)
        {
            if (mt9v03x_image[i][j] > threshold)
                image_binary[i][j] = 255;
            else
                image_binary[i][j] = 0;
        }
    }
}

static void compute_edge_histogram(void)
{
    for (int i = 0; i < MT9V03X_H; i++)
    {
        edge_histogram[i] = 0;
        for (int j = 1; j < MT9V03X_W - 1; j++)
        {
            int gradient = abs((int)image_binary[i][j+1] - (int)image_binary[i][j-1]);
            if (gradient > 100)
                edge_histogram[i]++;
        }
    }
}

static int find_step_edge(void)
{
    uint16 max_edges = 0;
    int step_row = -1;
    
    for (int i = MT9V03X_H / 3; i < MT9V03X_H - 5; i++)
    {
        if (edge_histogram[i] > max_edges)
        {
            max_edges = edge_histogram[i];
            step_row = i;
        }
    }
    
    if (max_edges > 10)
        return step_row;
    else
        return -1;
}

static int find_step_top_edge(int bottom_row)
{
    for (int i = bottom_row - 5; i >= bottom_row - 30; i--)
    {
        if (i < 5) break;
        
        int edge_count = 0;
        for (int j = 1; j < MT9V03X_W - 1; j++)
        {
            int gradient = abs((int)image_binary[i][j+1] - (int)image_binary[i][j-1]);
            if (gradient > 100)
                edge_count++;
        }
        
        if (edge_count > 5)
            return i;
    }
    
    return bottom_row - 10;
}

float calculate_step_distance(uint16 step_height_pix)
{
    if (step_height_pix == 0) return 0.0f;
    
    float distance = (STEP_HEIGHT_MM * FOCAL_LENGTH_MM) / (step_height_pix * PIXEL_SIZE_MM);
    
    distance = distance * cos(CAMERA_ANGLE_DEG * 3.14159f / 180.0f);
    
    distance = distance - CAMERA_HEIGHT_MM * sin(CAMERA_ANGLE_DEG * 3.14159f / 180.0f);
    
    return distance;
}

void step_detect(void)
{
    image_threshold();
    
    compute_edge_histogram();
    
    int step_bottom = find_step_edge();
    
    if (step_bottom > 0)
    {
        int step_top = find_step_top_edge(step_bottom);
        
        uint16 height_pix = (uint16)abs(step_bottom - step_top);
        
        if (height_pix >= 3 && height_pix <= 40)
        {
            step_data.detected = 1;
            step_data.step_row = (uint16)step_bottom;
            step_data.step_top_row = (uint16)step_top;
            step_data.step_height_pix = height_pix;
            step_data.distance_mm = calculate_step_distance(height_pix);
            step_data.distance_cm = step_data.distance_mm / 10.0f;
        }
        else
        {
            step_data.detected = 0;
        }
    }
    else
    {
        step_data.detected = 0;
    }
}

void step_set_threshold(uint8 new_threshold)
{
    if (new_threshold > 0 && new_threshold < 255)
    {
        threshold = new_threshold;
    }
}
