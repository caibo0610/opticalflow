#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

#include "pz_ae.h"

#define max(a,b) (((a) > (b)) ? (a) : (b))
#define min(a,b) (((a) < (b)) ? (a) : (b))

struct Exposure_table_type exposure_table =
{
	85,
	{
		//{	256 ,	1	},//	1.000 
		//{	282 ,	1	},//	1.102 
		//{	310 ,	1	},//	1.211 
		//{	341 ,	1	},//	1.332 
		//{	375 ,	1	},//	1.465 
		//{	412 ,	1	},//	1.609 
		//{	454 ,	1	},//	1.773
		//{	499 ,	1	},//	1.949 
		//{	274 ,	2	},//	1.070 
		//{	302 ,	2	},//	1.180 //10
		{	332 ,	2	},//	1.297 
		{	365 ,	2	},//	1.426 
		{	268 ,	3	},//	1.047 
		{	295 ,	3	},//	1.152 
		{	324 ,	3	},//	1.266 
		{	267 ,	4	},//	1.043 
		{	294 ,	4	},//	1.148 
		{	259 ,	5	},//	1.012 
		{	285 ,	5	},//	1.113 
		{	261 ,	6	},//	1.020 
		{	287 ,	6	},//	1.121 
		{	271 ,	7	},//	1.059 
		{	260 ,	8	},//	1.016 
		{	287 ,	8	},//	1.121 
		{	280 ,	9	},//	1.094 
		{	277 ,	10	},//	1.082 
		{	277 ,	11	},//	1.082 
		{	258 ,	13	},//	1.008 
		{	264 ,	14	},//	1.031 
		{	271 ,	15	},//	1.059 
		{	263 ,	17	},//	1.027 
		{	259 ,	19	},//	1.012 
		{	257 ,	21	},//	1.004 
		{	259 ,	23	},//	1.012 
		{	262 ,	25	},//	1.023 
		{	257 ,	28	},//	1.004 
		{	264 ,	30	},//	1.031 
		{	256 ,	34	},//	1.000 
		{	259 ,	37	},//	1.012 
		{	257 ,	41	},//	1.004 
		{	257 ,	45	},//	1.004 
		{	260 ,	49	},//	1.016 
		{	280 ,	50	},//	1.094 
		{	308 ,	50	},//	1.203 
		{	339 ,	50	},//	1.324 
		{	373 ,	50	},//	1.457 
		{	411 ,	50	},//	1.605 
		{	452 ,	50	},//	1.766 
		{	497 ,	50	},//	1.941 
		{	546 ,	50	},//	2.133 
		{	601 ,	50	},//	2.348 
		{	661 ,	50	},//	2.582 
		{	727 ,	50	},//	2.840 
		{	768 ,	52	},//	3.000 
		{	768 ,	57	},//	3.000 
		{	768 ,	63	},//	3.000 
		{	768 ,	69	},//	3.000 
		{	768 ,	76	},//	3.000 
		{	768 ,	84	},//	3.000 
		{	768 ,	92	},//	3.000 
		{	768 ,	101 },//	3.000 
		{	768 ,	112 },//	3.000 
		{	768 ,	123 },//	3.000 
		{	768 ,	135 },//	3.000 
		{	768 ,	149 },//	3.000 
		{	768 ,	163 },//	3.000 
		{	768 ,	180 },//	3.000 
		{	768 ,	198 },//	3.000 
		{	768 ,	218 },//	3.000 
		{	768 ,	239 },//	3.000 
		{	768 ,	263 },//	3.000 
		{	768 ,	290 },//	3.000 
		{	768 ,	319 },//	3.000 
		{	768 ,	350 },//	3.000 
		{	768 ,	385 },//	3.000 
		{	768 ,	424 },//	3.000 
		{	768 ,	466 },//	3.000 
		{	768 ,	513 },//	3.000 
		{	768 ,	564 },//	3.000 
		{	768 ,	621 },//	3.000 
		{	768 ,	683 },//	3.000 
		{	768 ,	751 },//	3.000 
		{	768 ,	826 },//	3.000 
		{	768 ,	909 },//	3.000 
		{	768 ,	1000	},//	3.000 
		{	845 ,	1000	},//	3.301 
		{	929 ,	1000	},//	3.629 
		{	1022	,	1000	},//	3.992 
		{	1124	,	1000	},//	4.391 
		{	1236	,	1000	},//	4.828 
		{	1360	,	1000	},//	5.313 
		{	1496	,	1000	},//	5.844 
		{	1646	,	1000	},//	6.430 
		{	1810	,	1000	},//	7.070 
		{	1991	,	1000	},//	7.777 
		//{	2191	,	1000	},//	8.559 
		//{	2410	,	1000	},//	9.414 //97
	},	
};

void ae_calc_current_luma_center(Ae_t *aeCtrl, int w, int h, unsigned char* image)
{
	unsigned int sum_Y = 0, i = 0, sum_N = 0;
	unsigned int center_roi_left_top_x = 3 * w / 8, center_roi_left_top_y = 3 * h / 8, center_roi_right_bottom_x = 5 * w / 8, center_roi_right_bottom_y = 5 * h / 8, x = 0, y = 0;
	//unsigned int side_roi_left_top_x = w / 4, side_roi_left_top_y = h / 4, side_roi_right_bottom_x = 3 * w / 4, side_roi_right_bottom_y = 3 * h / 4;
	unsigned int len = w * h;

	for(i = 0; i < len; i++) {
		y = i / w;
		x = i % w;
		if(((x >= center_roi_left_top_x) && (x <= center_roi_right_bottom_x)) 
			&& (y >= center_roi_left_top_y) && (y <= center_roi_right_bottom_y)) {
			sum_Y += image[i] * 2;
			sum_N += 2;
		//} else if(((x >= side_roi_left_top_x) && (x <= side_roi_right_bottom_x)) 
			//&& (y >= side_roi_left_top_y) && (y <= side_roi_right_bottom_y)) {
			//sum_Y += image[i] * 2;
			//sum_N += 2;

		} else {
			sum_Y += image[i];
			sum_N += 1;
		}
	}

	aeCtrl->cur_luma = (unsigned int)(sum_Y / sum_N);
}

void ae_calc_current_luma_hist(Ae_t *aeCtrl, int w, int h, unsigned char* image)
{
	unsigned int phist[256] = {0}, i = 0, gray = 0, mid = 0, num_Y = 0, sum_Y = 0, bright_threshod = 255;
	unsigned int len = w * h;

	for (i = 0; i < len; i++) {
		gray = image[i];
		phist[gray] = phist[gray] + 1;
	}

	for(mid = 255; (mid > 0) && (num_Y < len/2); mid--){
		num_Y += phist[mid];
	}

	num_Y = 0;
	for(i = 0; i < len; i++){
		gray = image[i];
		if((gray > mid) && (gray <= bright_threshod)){
			sum_Y += gray;
			num_Y++;
		}
	}

	if(num_Y){
		aeCtrl->cur_luma = sum_Y/num_Y;
	}
	else
		aeCtrl->cur_luma = mid;	

	return;
}

int calc_exposure_step_size_slow(Ae_t *aeCtrl, Chromatix_ae_type *aeParams)
{
	int index = 0, exp_step_index = 0;
	float exp_step_size = 0.0;
	float speed = 1.0;
	int exposure_step_size_lut[2][14] = {
											{-22, -18, -14, -10, -6, -3, -1, 0, 1, 3, 8, 12, 16, 20},//fast
											{-14, -10,  -8,  -7, -4, -2, -1, 0, 1, 2, 4,  6,  9, 13}//slow
										};
	unsigned int bright_torance = 120, dark_torance = 100;

	speed = aeParams->convergence_speed;
	//bright_torance = (unsigned int)(aeParams->target_luma + aeParams->luma_torance);
	bright_torance = (unsigned int)(aeParams->target_luma + 2 * aeParams->luma_torance);
	//dark_torance = (unsigned int)(aeParams->target_luma - aeParams->luma_torance);
	dark_torance = (unsigned int)(aeParams->target_luma);
	//printf("%s: bright_thred = %d, dark_thred = %d\n", __func__, bright_torance, dark_torance);

	if((aeCtrl->cur_luma >= aeParams->target_luma + 90) && aeCtrl->cur_luma <= 255)               //-14
		index = 0;
	else if((aeCtrl->cur_luma <= aeParams->target_luma + 90) && (aeCtrl->cur_luma > aeParams->target_luma + 64))//-10
		index = 1;
	else if((aeCtrl->cur_luma <= aeParams->target_luma + 64) && (aeCtrl->cur_luma > aeParams->target_luma + 48))//-7
		index = 2;
	else if((aeCtrl->cur_luma <= aeParams->target_luma + 48) && (aeCtrl->cur_luma > aeParams->target_luma + 36))//-6
		index = 3;
	else if((aeCtrl->cur_luma <= aeParams->target_luma + 36) && (aeCtrl->cur_luma > aeParams->target_luma + 24))//-4
		index = 4;
	else if((aeCtrl->cur_luma <= aeParams->target_luma + 24) && (aeCtrl->cur_luma > aeParams->target_luma + 12))//-3
		index = 5;
	else if((aeCtrl->cur_luma <= aeParams->target_luma + 12) && (aeCtrl->cur_luma > bright_torance))//-2
		index = 6;
	else if((aeCtrl->cur_luma <= bright_torance) && (aeCtrl->cur_luma >= dark_torance))//0
		index = 7;
	else if((aeCtrl->cur_luma < dark_torance) && (aeCtrl->cur_luma >= aeParams->target_luma - 12))//1
		index = 8;
	else if((aeCtrl->cur_luma < aeParams->target_luma - 12) && (aeCtrl->cur_luma >= aeParams->target_luma - 24))//2
		index = 9;
	else if((aeCtrl->cur_luma < aeParams->target_luma - 24) && (aeCtrl->cur_luma >= aeParams->target_luma - 36))//4
		index = 10;
	else if((aeCtrl->cur_luma < aeParams->target_luma - 36) && (aeCtrl->cur_luma >= aeParams->target_luma - 48))//7
		index = 11;
	else if((aeCtrl->cur_luma < aeParams->target_luma - 48) && (aeCtrl->cur_luma >= aeParams->target_luma - 64))//9
		index = 12;
	else if(aeCtrl->cur_luma < aeParams->target_luma - 64)//13
		index = 13;

	if(aeCtrl->last_exp_index <= 12)
		speed = 0.5;
	if(aeCtrl->last_exp_index <= 8)
		speed = 0.2;

	exp_step_size =  speed * exposure_step_size_lut[1][index];
	exp_step_index = (int)exp_step_size;

    if (exp_step_size < 0) {
      if (exp_step_index == 0)
        exp_step_index = exp_step_index - 1;
    } else {
      /* halve speed to go to darker scene.. */
      if (exp_step_index == 0)
        exp_step_index = exp_step_index + 1;
    }

	return exp_step_index;
}


int calc_exposure_step_size_fast(Ae_t *aeCtrl, Chromatix_ae_type *aeParams)
{
	float exp_step_size = 0;
	int delta_luma = aeCtrl->cur_luma - aeParams->target_luma, exp_step_index = 0;
	float exposure_index_adj_step = 24.16;//47.2; //1/log1.1  //1/log1.05
	float speed = aeParams->convergence_speed;

	if(abs(delta_luma) <= aeParams->luma_torance)
		return 0;

	if(aeCtrl->cur_luma == 0)
		aeCtrl->cur_luma = 1;

    exp_step_size = (float) ((log10((double) aeParams->target_luma / aeCtrl->cur_luma)) * exposure_index_adj_step);

	if(exp_step_size < 0)
		exp_step_size *= 1.2;

	if(aeCtrl->last_exp_index <= 12)
			speed = 0.5;
		if(aeCtrl->last_exp_index <= 5)
			speed = 0.2;
	
	exp_step_size =  speed * exp_step_size;
	exp_step_index = (int)exp_step_size;

    if (exp_step_size < 0) {
      if (exp_step_index == 0)
        exp_step_index = exp_step_index - 1;
    } else {
      /* halve speed to go to darker scene.. */
      if (exp_step_index == 0)
        exp_step_index = exp_step_index + 1;
    }

	return exp_step_index;
}

void ae_process_simple(Ae_t * aeCtrl, Chromatix_ae_type *aeParams, int w, int h, unsigned char *image)
{
	//int delta_luma = 0;//, i = 0, j = 0, value = 0;
	int exp_step_size = 0;
	float real_gain = 1.0;

	aeCtrl->last_luma = aeCtrl->cur_luma;
	ae_calc_current_luma_center(aeCtrl, w, h, image);


	if(aeCtrl->skip_frame) {
		aeCtrl->skip_frame--;
		//printf("skip!!!\n");
		return;
	}

	unsigned int bright_torance = (unsigned int)(aeParams->target_luma + 2 * aeParams->luma_torance);
	unsigned int dark_torance = (unsigned int)(aeParams->target_luma);

	if(((aeCtrl->cur_luma <= bright_torance) && (aeCtrl->cur_luma >= dark_torance)) ||
		((aeCtrl->cur_luma < dark_torance) && (aeCtrl->last_exp_index >= exposure_table.valid_entries - 1)) ||
		((aeCtrl->cur_luma > bright_torance) && (aeCtrl->last_exp_index <= 0))) {

		aeCtrl->ae_settled = 1;
		aeCtrl->skip_frame = 0;

		//printf("cur_lum = %d, target_luma = %d, ae_settled = %d\n", aeCtrl->cur_luma, aeParams->target_luma, aeCtrl->ae_settled);
		return;
	}

	aeCtrl->ae_settled = 0;
	aeCtrl->last_exp_index = aeCtrl->cur_exp_index;

	if(aeParams->ae_convergence_type == CONV_FAST)
		exp_step_size = calc_exposure_step_size_fast(aeCtrl, aeParams);
	else if(aeParams->ae_convergence_type == CONV_SLOW)
		exp_step_size = calc_exposure_step_size_slow(aeCtrl, aeParams);

	aeCtrl->cur_exp_index += exp_step_size;

	if(aeCtrl->cur_exp_index > exposure_table.valid_entries - 1)
		aeCtrl->cur_exp_index = exposure_table.valid_entries - 1;
	if(aeCtrl->cur_exp_index < 0)
		aeCtrl->cur_exp_index = 0;

	real_gain = (float)(exposure_table.exposure_entries[aeCtrl->cur_exp_index].gain) / 256.0;
	if(real_gain < 1.0)
		real_gain = 1.0;
	if(real_gain > 8.0)
		real_gain = 8.0;

	aeCtrl->cur_gain = (unsigned short)(real_gain * 16 + 0.5);
	aeCtrl->cur_linecnt = exposure_table.exposure_entries[aeCtrl->cur_exp_index].linecnt;

	aeCtrl->skip_frame = 2;

	//printf("cur_lum = %d, target_luma = %d, step_index = %d, cur_index = %d, cur_line = %d, cur_gain = %d, ae_settled = %d\n",
		//aeCtrl->cur_luma, aeParams->target_luma, exp_step_size, aeCtrl->cur_exp_index, aeCtrl->cur_linecnt, aeCtrl->cur_gain, aeCtrl->ae_settled);

	return;
}


void ae_init_simple(Ae_t * aeCtrl, Chromatix_ae_type *aeParams)
{
	aeCtrl->ae_settled = FALSE;
	aeCtrl->cur_luma = 0;
	aeCtrl->last_luma = 0;
	aeCtrl->cur_exp_index = aeParams->default_index;
	aeCtrl->last_exp_index = aeCtrl->cur_exp_index;
	aeCtrl->cur_gain = exposure_table.exposure_entries[aeCtrl->cur_exp_index].gain;
	aeCtrl->cur_linecnt = exposure_table.exposure_entries[aeCtrl->cur_exp_index].linecnt;
	aeCtrl->skip_frame = 0;
	aeCtrl->last_gain = aeCtrl->cur_gain;
	aeCtrl->last_linecnt = aeCtrl->cur_linecnt;

	return;
}

void ae_deinit(Ae_t *aeCtrl)
{
	memset(aeCtrl, 0, sizeof(Ae_t));

	return;
}
