/*************************************************************************
	> File Name: shmdata.h
	> Author: guosc
	> Mail: guosc@znxtech.com 
	> Created Time: 20180509
 ************************************************************************/

#ifndef __SHMDATA_H__
#define __SHMDATA_H__

struct shared_use_st
{
	int counter;
	int wait_pic_renew;
};

#endif
