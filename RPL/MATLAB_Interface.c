/*
*
* This is a simple program that illustrates how to call the MATLAB
* Engine functions from NetSim C Code.
*
*/
#include <windows.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "main.h"
#include "RPL.h"
#include "engine.h"
#include "mat.h"
#include "mex.h"
char buf[100];
Engine *ep;
int status;
mxArray *out;
mxArray *theArray, *nID, *Xc, *Yc, *H, *nRank;
NETSIM_ID dim =0,dcount=0,dcounti,dcountj;
char sensor1[50];
char sinknode1[50];
bool result1,result2;
bool **array;
NETSIM_ID *Node_Id;
double *xc, *yc;
NETSIM_ID *h;
unsigned short *nrank;

bool is_prefparent(NETSIM_ID s, NETSIM_ID t);

bool is_prefparent(NETSIM_ID s, NETSIM_ID t)
{
	PRPL_NODE rpl_s = GET_RPL_NODE(s);
	if (rpl_s == NULL)
	{
		return FALSE;
	}
	if (rpl_s->joined_dodag == NULL)
	{
		return FALSE;
	}
	UINT16 pcount;
	pcount = rpl_s->joined_dodag->parent_count;
	
		if (rpl_s->joined_dodag->pref_parent->nodeId == t)
		{
			return TRUE;
		}
	return FALSE;
}



double fn_netsim_matlab_init()
{
	/*
	* Start the MATLAB engine
	*/
	if (!(ep = engOpen(NULL))) {
		MessageBox((HWND)NULL, (LPCWSTR)"Can't start MATLAB engine",
			(LPCWSTR) "MATLAB_Interface.c", MB_OK);
		exit(-1);
	}
	engEvalString(ep, "desktop");
	return 0;
}
double fn_netsim_matlab_DODDAG_Init()

{
	//write your own implementation here

	dcount = NETWORK->nDeviceCount;
	strcpy(sensor1, "SENSOR");
	strcpy(sinknode1, "SINKNODE");

	while (dcount > 0)
	{
		result1 = strcmp(NETWORK->ppstruDeviceList[dcount - 1]->type, sensor1);
		result2 = strcmp(NETWORK->ppstruDeviceList[dcount - 1]->type, sinknode1);
		if (result1 == false || result2 == false)
		{
			dim++;
		}
		dcount--;
	}
	unsigned int i, j = 0;



	theArray = mxCreateNumericMatrix((mwSize)dim, (mwSize)dim, mxLOGICAL_CLASS, mxREAL);
	array = (bool **)mxMalloc(dim * sizeof(*array));
	array[0] = (bool*)mxGetPr(theArray);

	nRank = mxCreateNumericMatrix(1, (mwSize)dim, mxUINT16_CLASS, mxREAL);
	nrank = (unsigned short*)mxMalloc(dim * sizeof(nrank));
	nrank = (unsigned short*)mxGetPr(nRank);

	nID = mxCreateNumericMatrix(1, (mwSize)dim, mxUINT32_CLASS, mxREAL);
	Node_Id = (NETSIM_ID*)mxMalloc(dim * sizeof(Node_Id));
	Node_Id = (NETSIM_ID*)mxGetPr(nID);

	Xc = mxCreateNumericMatrix(1, (mwSize)dim, mxDOUBLE_CLASS, mxREAL);
	xc = (double*)mxMalloc(dim * sizeof(xc));
	xc = mxGetPr(Xc);

	Yc = mxCreateNumericMatrix(1, (mwSize)dim, mxDOUBLE_CLASS, mxREAL);
	yc = (double*)mxMalloc(dim * sizeof(yc));
	yc = mxGetPr(Yc);

	H = mxCreateNumericMatrix(1, 1, mxUINT32_CLASS, mxREAL);
	h = (NETSIM_ID*)mxMalloc(dim * sizeof(h));
	h = (NETSIM_ID*)mxGetPr(H);

	for (i = 1; i<dim; i++)
	{
		array[i] = array[i - 1] + dim;
	}
	return 0;

	
}


double fn_netsim_matlab_DODDAG_run()
{
	/*Modified*/
	unsigned int i, j = 0;
	i = 0;
	dcounti = 0; 
	while (dcounti < NETWORK->nDeviceCount)
	{
		result1 = strcmp(NETWORK->ppstruDeviceList[dcounti]->type, sensor1);
		result2 = strcmp(NETWORK->ppstruDeviceList[dcounti]->type, sinknode1);
		if (!(result1==false||result2==false))
		{
			dcounti++;
			continue;
		}
		j = 0;
		dcountj = 0;
		while (dcountj < NETWORK->nDeviceCount)
		{
			result1 = strcmp(NETWORK->ppstruDeviceList[dcountj]->type, sensor1);
			result2 = strcmp(NETWORK->ppstruDeviceList[dcountj]->type, sinknode1);
			if (!(result1 == false || result2 == false))
			{
				dcountj++;
				continue;
			}
			
			if (is_prefparent(NETWORK->ppstruDeviceList[dcounti]->nDeviceId, NETWORK->ppstruDeviceList[dcountj]->nDeviceId))
			{
				array[j][i] = TRUE;
			}
			else
			{
				array[j][i] = FALSE;
			}
			
			j++;
			dcountj++;
		}
		PRPL_NODE rpl = GET_RPL_NODE(NETWORK->ppstruDeviceList[dcounti]->nDeviceId);
		result2 = strcmp(NETWORK->ppstruDeviceList[dcounti]->type, sinknode1);
		if (result2 == false)
		{
			nrank[i] = 1;
			
		}
		else if(rpl->joined_dodag !=NULL)
		{
			nrank[i] = rpl->joined_dodag->rank;
		}
		else
		{
			nrank[i] = 0;
		}
		xc[i] = DEVICE_POSITION(NETWORK->ppstruDeviceList[dcounti]->nDeviceId)->X;
		yc[i] = DEVICE_POSITION(NETWORK->ppstruDeviceList[dcounti]->nDeviceId)->Y;
		Node_Id[i] =NETWORK->ppstruDeviceList[dcounti]->nConfigDeviceId;
		
		if (result2 == false)
		{
			h[0] = i+1;
		}
		i++;
		dcounti++;
		
	}
	engPutVariable(ep, "nID", nID);
	out = engGetVariable(ep, "nID");
	engPutVariable(ep, "nRank", nRank);
	out = engGetVariable(ep, "nRank");
	engPutVariable(ep, "Xc", Xc);
	out = engGetVariable(ep, "Xc");
	engPutVariable(ep, "Yc", Yc);
	out = engGetVariable(ep, "Yc");
	engPutVariable(ep, "H",H );
	out = engGetVariable(ep, "H");
	engPutVariable(ep, "theArray", theArray);
	out = engGetVariable(ep, "theArray");
	status = engEvalString(ep, "PlotDAG(theArray,Xc,Yc,H,nID,nRank)");
	
	return 0;

	
}
double fn_netsim_matlab_finish()
{
	status = engEvalString(ep, "exit");
	return 0;
}
