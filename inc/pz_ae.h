/********************************************************************
	created:	2018/1/2
	filename: 	pz_ae.h
	file ext:	
	author:  	
	purpose:  
*********************************************************************/

#ifndef _PZ_AE_H
#define _PZ_AE_H


#include <stdio.h>

#define FALSE 0
#define TRUE 1
#define MAX_EXPOSURE_TABLE_SIZE 100

struct Exposure_entries_type
{
	unsigned short gain;
	unsigned long linecnt;
};

struct Exposure_table_type
{
	int valid_entries;
	struct Exposure_entries_type exposure_entries[MAX_EXPOSURE_TABLE_SIZE];
};

typedef enum {
  CONV_SLOW = 0,
  CONV_FAST,
} ae_conv_type;

typedef enum {
	UPDATE_WAIT,
	UPDATING,
	UPDATED,
} ae_update_state_type;

struct Chromatix_ae_type
{
	unsigned int default_index;
	unsigned int target_luma;
	int luma_torance;
	unsigned int bright_threshod;
	unsigned int sync_exp_gain;
	ae_conv_type ae_convergence_type;
	float convergence_speed;
};

struct Ae_t
{
	unsigned int last_linecnt;
	unsigned int cur_linecnt;
	unsigned short last_gain;
	unsigned short cur_gain;
	unsigned int cur_luma;
	unsigned int last_luma;
	unsigned int skip_frame;
	int last_exp_index;
	int cur_exp_index;
	bool ae_settled;
};

void ae_init_simple(Ae_t * aeCtrl, Chromatix_ae_type *aeParams);
void ae_process_simple(Ae_t * aeCtrl, Chromatix_ae_type *aeParams, int w, int h, unsigned char *image);
void ae_deinit(Ae_t *aeCtrl);


#endif //_PZ_AE_H

