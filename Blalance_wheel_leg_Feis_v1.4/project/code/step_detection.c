#include "zf_common_headfile.h"

step_info_t step_data = {0, 0, 0, 0, 0.0f, 0.0f};

static uint8 image_binary[MT9V03X_H][MT9V03X_W];
static uint16 edge_histogram[MT9V03X_H];
static uint8 threshold;

#define HISTORY_SIZE 10
static float distance_history[HISTORY_SIZE] = {0};
static uint16 height_history[HISTORY_SIZE] = {0};
static uint8 history_index = 0;
static int8 detect_counter = 0;
static uint8 last_valid_height = 0;
static float last_valid_distance = 0;
static float min_distance_recorded = 9999.0f;
static uint8 fail_counter = 0;

void step_detection_init(void)
{
    step_data.detected = 0;
    step_data.step_row = 0;
    step_data.step_top_row = 0;
    step_data.step_height_pix = 0;
    step_data.distance_mm = 0.0f;
    step_data.distance_cm = 0.0f;
    
    for (int i = 0; i < HISTORY_SIZE; i++)
    {
        distance_history[i] = 0;
        height_history[i] = 0;
    }
    history_index = 0;
    detect_counter = 0;
    last_valid_height = 0;
    last_valid_distance = 0;
    min_distance_recorded = 9999.0f;
    fail_counter = 0;
}

static void compute_horizontal_edge_histogram(void)
{
    for (int i = 5; i < MT9V03X_H - 5; i++)
    {
        edge_histogram[i] = 0;
        for (int j = 10; j < MT9V03X_W - 10; j++)
        {
            int gradient = abs((int)mt9v03x_image[i+5][j] - (int)mt9v03x_image[i-5][j]);
            if (gradient > 25)
                edge_histogram[i]++;
        }
    }
}

static int find_strong_edge_row(int start_row, int end_row, int min_edges)
{
    int best_row = -1;
    uint16 max_edges = 0;
    
    for (int i = start_row; i < end_row; i++)
    {
        if (edge_histogram[i] > max_edges && edge_histogram[i] >= min_edges)
        {
            int is_local_max = 1;
            for (int k = -2; k <= 2; k++)
            {
                if (k != 0 && (i+k) >= start_row && (i+k) < end_row)
                {
                    if (edge_histogram[i+k] > edge_histogram[i])
                    {
                        is_local_max = 0;
                        break;
                    }
                }
            }
            
            if (is_local_max)
            {
                max_edges = edge_histogram[i];
                best_row = i;
            }
        }
    }
    
    return best_row;
}

static int find_step_bottom_edge(void)
{
    return find_strong_edge_row(MT9V03X_H / 3, MT9V03X_H - 10, MT9V03X_W / 5);
}

static int find_step_top_edge(int bottom_row)
{
    return find_strong_edge_row(20, bottom_row - 5, MT9V03X_W / 5);
}

static float get_filtered_distance(float new_distance)
{
    distance_history[history_index] = new_distance;
    height_history[history_index] = step_data.step_height_pix;
    history_index = (history_index + 1) % HISTORY_SIZE;
    
    float sum = 0;
    uint8 count = 0;
    for (int i = 0; i < HISTORY_SIZE; i++)
    {
        if (distance_history[i] > 0)
        {
            sum += distance_history[i];
            count++;
        }
    }
    
    if (count >= 3)
        return sum / count;
    else if (last_valid_distance > 0)
        return last_valid_distance;
    else
        return new_distance;
}

float calculate_step_distance(uint16 step_height_pix)
{
    if (step_height_pix == 0) return 0.0f;
    
    float distance = (STEP_HEIGHT_MM * FOCAL_LENGTH_MM) / (step_height_pix * PIXEL_SIZE_MM);
    
    distance = distance * cos(CAMERA_ANGLE_DEG * 3.14159f / 180.0f);
    
    distance = distance - CAMERA_HEIGHT_MM * sin(CAMERA_ANGLE_DEG * 3.14159f / 180.0f);
    
    if (distance < 0) distance = 0;
    
    return distance;
}

uint8 step_detect(void)
{
    if (mt9v03x_finish_flag)
    {
        mt9v03x_finish_flag = 0;

        compute_horizontal_edge_histogram();
        
        int step_bottom = find_step_bottom_edge();
        
        if (step_bottom > 0)
        {
            int step_top = find_step_top_edge(step_bottom);
            
            if (step_top > 0 && step_bottom > step_top)
            {
                uint16 height_pix = (uint16)(step_bottom - step_top);
                
                if (height_pix >= MIN_STEP_HEIGHT_PIX && height_pix <= MAX_STEP_HEIGHT_PIX)
                {
                    float new_distance = calculate_step_distance(height_pix);
                    
                    uint8 is_valid = 1;
                    
                    if (last_valid_distance > 0)
                    {
                        if (new_distance > last_valid_distance + 50.0f)
                        {
                            is_valid = 0;
                        }
                        
                        if (min_distance_recorded < 9999.0f && new_distance > min_distance_recorded + 100.0f)
                        {
                            is_valid = 0;
                        }
                    }
                    
                    if (is_valid)
                    {
                        if (new_distance < min_distance_recorded)
                        {
                            min_distance_recorded = new_distance;
                        }
                        
                        step_data.step_row = (uint16)step_bottom;
                        step_data.step_top_row = (uint16)step_top;
                        step_data.step_height_pix = height_pix;
                        step_data.distance_mm = get_filtered_distance(new_distance);
                        step_data.distance_cm = step_data.distance_mm / 10.0f;
                        
                        last_valid_height = height_pix;
                        last_valid_distance = step_data.distance_mm;
                        
                        detect_counter++;
                        fail_counter = 0;
                        if (detect_counter > 15) detect_counter = 15;
                        
                        if (detect_counter >= 3)
                        {
                            step_data.detected = 1;
                        }
                        return 1;
                    }
                }
            }
        }
        
        fail_counter++;
        detect_counter--;
        if (detect_counter < 0) detect_counter = 0;
        
        if (fail_counter > 30 || detect_counter < 2)
        {
            step_data.detected = 0;
        }
        
        if (fail_counter > 50)
        {
            min_distance_recorded = 9999.0f;
            last_valid_distance = 0;
            last_valid_height = 0;
        }
        else if (last_valid_distance > 0)
        {
            step_data.step_height_pix = last_valid_height;
            step_data.distance_mm = last_valid_distance;
            step_data.distance_cm = last_valid_distance / 10.0f;
        }
        
        return 0;
    }
    return 0;
}

void step_reset_distance_tracking(void)
{
    min_distance_recorded = 9999.0f;
    last_valid_distance = 0;
    last_valid_height = 0;
    fail_counter = 0;
}
