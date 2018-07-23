/*************************************************************************
  > File Name: optical_flow.c
  > Author: guosc
  > Mail: guosc@znxtech.com 
  > Created Time: Thu Jul 14 16:38:26 2016
 ************************************************************************/

#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/shm.h>
#include <signal.h>
#include <syslog.h>
#include "shmdata.h"
#include "opticflow.h"
//#include <pthread.h>
#include <signal.h>

char *of_program_name = "./opticflow";

/**********************************
 *  get pid from the processname
 * *********************************/
pid_t getPidFromName(char *procName)
{
	FILE *fp;
	char buf[255];
	char cmd[255] = {'\0'};
	pid_t pid = -1;

	//get process id 
	sprintf(cmd,"pidof %s",procName);
	if ((fp = popen(cmd,"r")) != NULL)
	{
		if(fgets(buf,255,fp) != NULL)
		{
			pid = atoi(buf);
			printf("pid = %d\n",pid);
		} 
	} else
		fprintf(stderr, "popen error %s\n",cmd);

	pclose(fp);
	return pid;
}

int main()
{
	void *shm = NULL;
	struct shared_use_st *shared;
	int shmid;
	pid_t pid;

	int sleep_counter = 0;
#if 1
    sigset_t blockSet;
    sigfillset(&blockSet);
   	pthread_sigmask(SIG_BLOCK,& blockSet, NULL);
#endif
    
    if (setvbuf(stdout,NULL,_IOLBF,0))
        ERROR("set cache buf for stdout\n");

	//get share memmory id
	shmid = shmget((key_t)6538, sizeof(struct shared_use_st), 0666|IPC_CREAT);
	if (shmid == -1)
	{
		fprintf(stderr,"shmget failed\n");
		exit(EXIT_FAILURE);
	}

	//map the share memory to local
	shm = shmat(shmid,0,0);
	if (shm == NULL)
	{
		fprintf(stderr, "shmat failed\n");
		exit(EXIT_FAILURE);
	}

	shared = (struct shared_use_st *)shm;
	shared->counter = 0;

	shared->wait_pic_renew = 0;
	
	//start overloop
	while(1)
	{
		if (shared->counter != 0)
		{
			shared->counter = 0;
		}else
		{
			ERROR("get counter zero, should restart opticflow process\n");
			pid = getPidFromName(of_program_name);
			if (pid > 0) 
			{
				ERROR("opticflow process still exist pid = %d\n", pid);
				if (kill(pid,SIGKILL))
				{
					fprintf(stderr,"kill(SIGKILL)error\n");
					continue;
				}
				wait(NULL);
			}else
				ERROR("opticflow process is over\n");

			if (0 == fork())
			{
				ERROR("start opticflow again\n");
				if (execl(of_program_name,of_program_name,NULL))
					fprintf(stderr,"exec %s error\n",of_program_name);
			} 

			usleep(100000);
		}

		//will check optical flow process each 3s
		 //
		 while(sleep_counter < 30)
	 	 {
	 		if(shared->wait_pic_renew > 50)
 			{
 				shared->wait_pic_renew = 0;
				ERROR("get pic error, should restart opticflow process\n");
				pid = getPidFromName(of_program_name);
				if (pid > 0) 
				{
					ERROR("opticflow process still exist pid = %d\n", pid);
					if (kill(pid,SIGKILL))
					{
						fprintf(stderr,"kill(SIGKILL) error\n");
						continue;
					}
					wait(NULL);
				}else
					ERROR("opticflow process is over\n");

				if (0 == fork())
				{
					ERROR("start opticflow again\n");
					if (execl(of_program_name,of_program_name,NULL)){
						fprintf(stderr,"exec %s error\n",of_program_name);
							 shared->wait_pic_renew = 0;
						}
				}
				usleep(100000);
 			}
			sleep_counter++;
			usleep(100000);
	 	 }
		 sleep_counter = 0;
	}

	//unmap share memory
	if (shmdt(shm) == -1)
	{
		fprintf(stderr,"shmdt failed\n");
		exit(EXIT_FAILURE);
	}

	//remove share memory
	if (shmctl(shmid,IPC_RMID, 0) == -1)
	{
		fprintf(stderr,"shmctl(IPC_RMID) failed\n");
		exit(EXIT_FAILURE);
	}

	exit(EXIT_SUCCESS);
}


