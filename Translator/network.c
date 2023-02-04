/***********************************************************************************************************************
* DISCLAIMER
* This software is supplied by Renesas Electronics Corporation and is only intended for use with Renesas products. No 
* other uses are authorized. This software is owned by Renesas Electronics Corporation and is protected under all 
* applicable laws, including copyright laws. 
* THIS SOFTWARE IS PROVIDED 'AS IS' AND RENESAS MAKES NO WARRANTIES REGARDING
* THIS SOFTWARE, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY, 
* FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. ALL SUCH WARRANTIES ARE EXPRESSLY DISCLAIMED. TO THE MAXIMUM
* EXTENT PERMITTED NOT PROHIBITED BY LAW, NEITHER RENESAS ELECTRONICS CORPORATION NOR ANY OF ITS AFFILIATED COMPANIES 
* SHALL BE LIABLE FOR ANY DIRECT, INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR ANY REASON RELATED TO THIS 
* SOFTWARE, EVEN IF RENESAS OR ITS AFFILIATES HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
* Renesas reserves the right, without notice, to make changes to this software and to discontinue the availability of 
* this software. By using this software, you agree to the additional terms and conditions found by accessing the 
* following link:
* http://www.renesas.com/disclaimer 
*
* Changed from original python code to C source code.
* Copyright (C) 2017 Renesas Electronics Corporation. All rights reserved.
***********************************************************************************************************************/
/**********************************************************************************************************************
* Copyright 2015 The TensorFlow Authors. All Rights Reserved.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
***********************************************************************************************************************/

/***********************************************************************************************************************
* File Name    : network.c
* Version      : 1.00
* Description  : Definitions of all functions
***********************************************************************************************************************/
/**********************************************************************************************************************
* History : DD.MM.YYYY Version  Description
*         : 16.06.2017 1.00     First Release
***********************************************************************************************************************/

#include "stdlib.h"
#include "Typedef.h"
#include "math.h"
#include "layer_graph.h"
#define C_FP32 1
/***********************************************************************************************************************
* Function Name: padding
* Description  : - Padding operation
*				 - Creates a new output matrix with zero's padded on the original input 
* Arguments    : dData		- Pointer to the input data
*				 dPad		- Pointer to the padding output
*				 iShapes	- Dimensions 
*					( N, input channels, input height, input width, output height, output width ,pad top, pad left)
*                errorcode - errorcode if any issue in memory allocation
* Return Value : no return value
***********************************************************************************************************************/
void padding(TPrecision *dData, TPrecision *dPad, TsInt *iShapes, TsInt *errorcode)
{
	if (*errorcode!=1)
	{
		TsInt iD1,iD2,iD3,iD4;
		TsInt iC = iShapes[1];		// Number of channels
		TsInt iH = iShapes[2];		// Input image height
		TsInt iW = iShapes[3];		// Input image width
		TsInt iPH = iShapes[4];		// Output height
		TsInt iPW = iShapes[5];		// Output width
		TsInt pad_top = iShapes[6];	// Number of pixels to be added to the height
		TsInt pad_left = iShapes[7];	// Number of pixels to be added to the width

		// Padding operation
		for(iD1=0; iD1 < iC*iPH*iPW; iD1++)
			dPad[iD1]=0;

		for(iD2 = 0; iD2 < iC; iD2++)
		{
			for(iD3 = pad_top; iD3 < iH + pad_top; iD3++)
			{				
				for(iD4 = pad_left; iD4 < iW + pad_left; iD4++)
				{
					dPad[(iD2*iPH*iPW)+(iD3*iPW)+(iD4)] = dData[(iD2*iH*iW)+((iD3-pad_top)*iW)+(iD4-pad_left)];
				}
			}
		}
	}	
}

/***********************************************************************************************************************
* Function Name: averae pooling padding
* Description  : - Padding operation
*				 - Creates a new output matrix with zero's padded on the original input 
* Arguments    : dData		- Pointer to the input data
*				 dPad		- Pointer to the padding output
*				 iShapes	- Dimensions 
*					( N, input channels, input height, input width, output height, output width ,pad top, pad left)
*                errorcode - errorcode if any issue
* Return Value : no return value
***********************************************************************************************************************/
void avgpool_padding(TPrecision *dData, TPrecision *dPad, TsInt *iShapes,TsInt *errorcode)
{
	if (*errorcode!=1)
	{
		TsInt iD1,iD2,iD3,iD4;
		TsInt iC = iShapes[1];		// Number of channels
		TsInt iH = iShapes[2];		// Input image height
		TsInt iW = iShapes[3];		// Input image width
		TsInt iPH = iShapes[4];		// Output height
		TsInt iPW = iShapes[5];		// Output width
		TsInt pad_top = iShapes[6];	// Number of pixels to be added to the height
		TsInt pad_left = iShapes[7];	// Number of pixels to be added to the width

		// Padding operation
		for(iD1=0; iD1 < iC*iPH*iPW; iD1++)
			dPad[iD1]=-1000;

		for(iD2 = 0; iD2 < iC; iD2++)
		{
			for(iD3 = pad_top; iD3 < iH + pad_top; iD3++)
			{				
				for(iD4 = pad_left; iD4 < iW + pad_left; iD4++)
				{
					dPad[(iD2*iPH*iPW)+(iD3*iPW)+(iD4)] = dData[(iD2*iH*iW)+((iD3-pad_top)*iW)+(iD4-pad_left)];
				}
			}
		}
	}
	
}


/***********************************************************************************************************************
* Function Name: convolution
* Description  : - Convolution layer
*				 - Performs elementwise multiplication of selected input data and weights and add them up with biases 
*				   by taking in the filter size and strides as the convolution parameters
* Arguments    : dData		- Pointer to the input data
*				 dPad		- Pointer to the padding output
*			 	 dWeights	- Pointer to the weights
*				 dBiases	- Pointer to the biases
*				 dOut		- Pointer to the convolution output (to be filled with values during convolution operation)
*				 iShapes	- Dimensions 
*					( N, Channels, input height, input width, No. of filters, channels, filter height, filter width, 
*					  output height,output width,pad top, pad bottom, pad left, pad right, stride height,stride width)
*                errorcode - errorcode if any issue in memory allocation
* Return Value : no return value
***********************************************************************************************************************/
void convolution(TPrecision *dData,TPrecision *dPad,const TPrecision *dWeights,const TPrecision *dBiases,TPrecision *dOut,TsInt *iShapes,TsInt *errorcode){
	if (*errorcode!=1)
	{
		TsInt iD2,iD3,iD4;   // Dimensions
		TsInt iItr1,iItr2,iItr3;
		TsInt iC = iShapes[1];     // Number of channels
		TsInt iH = iShapes[2];     // Input image height
		TsInt iW = iShapes[3];     // Input image width
		TsInt iF = iShapes[4];     // Number of filters
		TsInt iFH = iShapes[6];    // Filter height
		TsInt iFW = iShapes[7];    // Filter width
		TsInt iHp = iShapes[8];    // Output height
		TsInt iWp = iShapes[9];    // Output width

		TsInt pad_top = iShapes[10];	//No. of pixels to pad on top of input
		TsInt pad_bottom = iShapes[11];	//No. of pixels to pad on bottom of input
		TsInt pad_left = iShapes[12];	//No. of pixels to pad on left of input
		TsInt pad_right = iShapes[13];	//No. of pixels to pad on right of input

		TsInt iSH = iShapes[14];  // Stride height
		TsInt iSW = iShapes[15];  // Stride width

		TsInt iws, ihs, iPH, iPW ;
		TsInt pad_shapes[] = {1, iC, iH, iW, iH + pad_top + pad_bottom, iW + pad_left + pad_right, pad_top, pad_left};
		
		iPH = iH + pad_top + pad_bottom;   // Padded input height
		iPW =iW + pad_left + pad_right;	   // Padded input width

		if (pad_top!=0 || pad_bottom!=0 || pad_left!=0 || pad_right!=0)
		{
			padding(dData, dPad, pad_shapes, errorcode);
		}
		else
		{
			dPad = dData;
		}

		// Executes C Convolution or CCRX complier or CCRL complier
		#if defined C_FP32 || defined __CCRX__ || defined __CCRL__
		TPrecision dvalue;

		// Filtering operation

		for (iD2=0; iD2<iF; iD2++)
		{
			for (iD3=0; iD3<iHp;iD3++)
			{
				ihs = iD3 * iSH;
				for (iD4=0; iD4<iWp; iD4++)
				{
					iws = iD4 * iSW;
					dvalue=0;
				for(iItr1=0; iItr1<iC; iItr1++)
				{
					for(iItr2=ihs; iItr2<(ihs+iFH); iItr2++)
					{
						for(iItr3=iws; iItr3<(iws+iFW); iItr3++)
						{
							dvalue += dPad[(iItr1*iPH*iPW)+(iItr2*iPW)+iItr3] * dWeights[(iD2*iC*iFH*iFW)+(iItr1*iFH*iFW)+((iItr2-ihs)*iFW)+(iItr3-iws)];
						}
					}
				}
				dOut[(iD2*iHp*iWp)+(iD3*iWp)+iD4] = dvalue + dBiases[iD2];	// out = (data*weights)+biases
				}
			}
		}	
	}
	
	
	#endif
}

/***********************************************************************************************************************
* Function Name: relu
* Description  : - Rectified Linear Unit (ReLU)
*                - element wise operation and replaces all negative values by zero to introduce non-linearity
* Arguments    : dData      - Array of input data
*                dOut       - Pointer to the output data
*                iShapes    - Size of the input array
*                errorcode - errorcode if any issue in memory allocation
* Return Value : no return value
***********************************************************************************************************************/
void relu(TPrecision *dData, TPrecision *dOut, TsInt iShapes, TsInt *errorcode)
{
   if (*errorcode!=1)
   {
        TsInt iRow;
        for (iRow=0; iRow<iShapes; iRow++)
        {
            if (dData[iRow] < 0)
            {
                dOut[iRow] = 0;
            }
            else
            {
                dOut[iRow] = dData[iRow];      // output
            }
        }
   }
   
}

/***********************************************************************************************************************
* Function Name: innerproduct
* Description  : - Fully connected layer
*                - Performs dot product of data and weights and add them up with biases
*                   (Matrix Multiplication of data and weights and addition of biases)
* Arguments    : data           - Array of input data
*                weights        - Array of weights (transposed)
*                biases 		- Array of biases
*                out            - Placeholder for the output
*                shapes         - Dimensions of data and weights (N, D, F, D)
*                errorcode - errorcode if any issue in memory allocation
* Return Value : no return value
***********************************************************************************************************************/
void innerproduct(TPrecision *data,const TPrecision *weights,const TPrecision *biases,TPrecision *out,TsInt *shapes,TsInt *errorcode)
{
	if (*errorcode!=1)
	{
		TsInt iColumn;
		TsInt iInneritr;
		TsInt D = shapes[1];
		TsInt F = shapes[3];
		
		// Execute C Innerproduct or CCRX complier or CCRL complier    
		#if defined C_FP32 || defined __CCRX__ || defined __CCRL__
		TPrecision dSum = 0;

		for(iColumn=0; iColumn<F; iColumn++)
		{
			dSum = 0;
			for(iInneritr=0; iInneritr<D;iInneritr++)
			{
				dSum += data[iInneritr] * weights[(iInneritr*F)+iColumn];
			}
			out[iColumn] = dSum + biases[iColumn];				// output
		}
	}
	
	#endif
}

/***********************************************************************************************************************
* Function Name: batchnormalization2D
* Description  : - Batchnormalization layer
*                - Performs 2D batchnormalization
* Arguments    : data           - Array of input data
*                mean           - Array of mean
*                variance           - Array of variance
*                beta           - Array of beta
*                gamma           - Array of gamma
*                output           - Array of output data
*                shape           - Array of shape [H,W,eps]
*                errorcode - errorcode if any issue
* Return Value : no return value
***********************************************************************************************************************/
void batchnormalization2D(TPrecision *data,const TPrecision *mean,const TPrecision *variance,const TPrecision *beta,const TPrecision *gamma,TPrecision *dOut,TFloat *fShapes,TsInt *errorcode){
  if(*errorcode!=1)
  {
    TsInt H = fShapes[0]; // Input image height
    TsInt W = fShapes[1]; // Input image width
    TFloat eps = fShapes[2]; // epsilon
    TPrecision var;
    TsInt c,r;
          
    for (c=0; c<W; c++) 
    {
      for (r=0; r<H; r++) 
      {
        dOut[r*W +c] = (TFloat)data[r*W + c] - (TPrecision)mean[c] ;
      }
    }

    for (c=0; c<W; c++) 
    {
      var = sqrt(variance[c] + eps);
        for (r=0; r< H; r++) 
        {
            dOut[r*W + c] /= var;
            dOut[r*W + c] *= (TPrecision)gamma[c];
            dOut[r*W + c] = dOut[r*W + c] + (TPrecision)beta[c];  // output
        }
    }
  }
  
}


/***********************************************************************************************************************
* Function Name: softmax
* Description  : - Activation function
*                - Squashes an array of arbitrary real values to an array of real values 
*                  in the range(0, 1) that add up to 1	
* Arguments    : dData      - Array of input data
*                dOut       - Pointer to the output data
*                iShapes	- Size of the input array
*                errorcode - errorcode if any issue in memory allocation
* Return Value : no return value
***********************************************************************************************************************/
void softmax(TPrecision *dData, TPrecision *dOut, TsInt iShapes, TsInt *errorcode)
{
    if (*errorcode!=1)
    {
        TPrecision dMax, dSum = 0;
        TsInt iRow;

        dMax = dData[0];
        for (iRow = 1; iRow < iShapes; iRow++)
        {
            if (dData[iRow] > dMax)
            {
                dMax = dData[iRow];
            }
        }
        for (iRow = 0; iRow < iShapes; iRow++)
        {
            dData[iRow] = dData[iRow] - dMax;
            dSum = dSum + exp(dData[iRow]);
        }
        for (iRow = 0; iRow < iShapes; iRow++)
        {
            dOut[iRow] = exp(dData[iRow])/dSum;    //output
        }
    }
    
}



