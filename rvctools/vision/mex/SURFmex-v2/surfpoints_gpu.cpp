#include <cstdlib>
#include <stdexcept>
#include <iostream>
#include <sstream>
using namespace std;

#include <opencv/cv.h>
#include <opencv/highgui.h>
using namespace cv;

#include "GpuSurfDetector.hpp"

#include "CudaSynchronizedMemory.hpp"
#include "gpu_globals.h"

//Define this macro if timing information should be calculated by startTime()--endTime()
//#define TIMING
//Prints the calculated timing information if defined
//#define PRINT_TIMING
#include "mextiming.h"

#include "mex.h"

#include "cppmatrix.h"



bool getOptionsField(const mxArray* mxoptions, int index, const char* name, double& value)
{
	const mxArray* mxfield   = mxGetFieldByNumber(mxoptions,0,index);
    const char*    fieldname = mxGetFieldNameByNumber(mxoptions,index);
    
    if (strcmp(fieldname,name)!=0) {
        return false;
    }
    
	if (mxfield != 0) {
		if  ( !mxIsNumeric(mxfield)  || 
			   mxGetNumberOfElements(mxoptions)!=1)
		{
			mexErrMsgTxt("SURFPOINTS: Invalid options field. Options must be scalars.");
			return false;
		}
		value = mxGetScalar(mxfield);
		return true;
	} else {
		return false;
	}
}

void mexFunctionReal(int			nlhs, 		/* number of expected outputs */
				 mxArray		*plhs[],	/* mxArray output pointer array */
				 int			nrhs, 		/* number of inputs */
				 const mxArray	*prhs[]		/* mxArray input pointer array */)
{
	using namespace std;
	
	if (nrhs < 1) {
		mexErrMsgTxt("Need arguments");
	}
	
	matrix<unsigned char> mximg(prhs[0]);
	ASSERT(mximg.O == 1);
	
	//
	// SURF Options
	//
	asrl::GpuSurfConfiguration configuration;
	bool noOrientation = false;
	bool fastOrientation = false;
	bool noDescriptor = false;
	if (nrhs >= 2) {
		const mxArray* mxoptions = prhs[1];
		if (mxGetClassID(mxoptions)!=mxSTRUCT_CLASS) {
			mexErrMsgTxt("SURFMEX: Options must be provided as struct");
		}	
		if (mxGetNumberOfElements(mxoptions)!=1) {
			mexErrMsgTxt("SURFMEX: numel(options) ~= 1");
		}
		
		double value;
        
        int nfield = mxGetNumberOfFields(mxoptions);
        for (int field=0;field<nfield;++field) {
            if (getOptionsField(mxoptions,field,"nOctaveLayers",value)) {
                //params.nOctaveLayers = int(value+0.5);
            }
            else if (getOptionsField(mxoptions,field,"nOctaves",value)) {
                //params.nOctaves = int(value+0.5);
            }
            else if (getOptionsField(mxoptions,field,"hessianThreshold",value)) {
                //params.hessianThreshold = value;
            }
            else if (getOptionsField(mxoptions,field,"extended",value)) {
                //params.extended = value!=0;
            }
            else {
                char message[256];
                stringstream sout;
                sout << "Unknown field in options structure: \"" << mxGetFieldNameByNumber(mxoptions,field) << "\"";
                mexErrMsgTxt(sout.str().c_str());
            } 
        }
		
	}

	/*cudaError_t err;
    int deviceCount;
	int device = 0;
    err = cudaGetDeviceCount(&deviceCount);
    ASSERT(err == cudaSuccess); //, "Unable to get the CUDA device count: " << cudaGetErrorString(err));
    ASSERT(deviceCount > 0); //,"There are no CUDA capable devices present");
    ASSERT(deviceCount > device); //,"Device index is out of bounds");
    err = cudaSetDevice(device);
    //ASSERT(err == cudaSuccess); //, "Unable to set the CUDA device: " << cudaGetErrorString(err));
	*/

	startTime();

	Mat cvImg(mximg.N,mximg.M,CV_8UC1, mximg.data);
	/*for (int i=0;i<mximg.M;++i) {
		for (int j=0;j<mximg.N;++j) {
			cvImg.at<unsigned char>(j,i) = mximg(i,j);
		}
	}*/
	endTime("Converting image");
	

	asrl::GpuSurfDetector detector;

	endTime("Starting detector");

	detector.buildIntegralImage(cvImg);
	endTime("Integral image");
	detector.detectKeypoints();
	endTime("Detecting keypoints");


	detector.findOrientation();

	endTime("Computing orientation ");

	detector.computeDescriptors();
	endTime("Computing descriptors");


	int n;
	asrl::Keypoint* keypoints = detector.getKeypointsPointer(n);
    float* descriptors = detector.getDescriptorsPointer();
	

	endTime("Downloading data : %d",n);

	const int length = 64;
   
	matrix<double> mxpoints(2, n);
	matrix<float> mxdescr(length, n);

	for (int i = 0; i < n; ++i) {
		asrl::Keypoint* k = keypoints + i;
		float* descriptor = descriptors + length*i;
		mxpoints(1,i) = k->x;
		mxpoints(0,i) = k->y;
		for (int d=0;d<length;++d) {
			mxdescr(d,i) = descriptor[d];
		}
	}

	
	if (nlhs >= 1) {
		plhs[0] = mxpoints;
	}
	if (nlhs >= 2) {
		plhs[1] = mxdescr;
	}
	if (nlhs >= 3) {
		matrix<signed char> mxsign(1, n);
		for (int i = 0; i < n; ++i) {
			mxsign(i) = 1;
		}
		plhs[2] = mxsign;
	}
	if (nlhs >= 4) {
		matrix<float> mxinfo(3, n);
		plhs[3] = mxinfo;
	}
	
	endTime("Writing output matrices");

	endTime("Releasing memory");
}


void mexFunction(int		nlhs, 		/* number of expected outputs */
                 mxArray	*plhs[],	/* mxArray output pointer array */
                 int		nrhs, 		/* number of inputs */
                 const mxArray	*prhs[]		/* mxArray input pointer array */)
{
	try {
		mexFunctionReal(nlhs,plhs,nrhs,prhs);
	}
	catch (bad_alloc& ) {
		mexErrMsgTxt("Out of memory");
	}
	catch (cv::Exception& e) {
		stringstream sout;
		sout << "Code     : " << e.code << endl;
		sout << "Error    : " << e.err  << endl;
		sout << "Function : " << e.func << endl;
		sout << "File     : " << e.file << endl;
		mexErrMsgTxt(sout.str().c_str());
	}
	catch (exception& e) {
		mexErrMsgTxt(e.what());
	}
	catch (...) {
		mexErrMsgTxt("Unknown exception");
	}
}
