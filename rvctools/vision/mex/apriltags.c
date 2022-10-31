
#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include <ctype.h>
#include <unistd.h>

#include "apriltag.h"
#include "image_u8.h"
#include "tag36h11.h"

#include "zarray.h"
#include "getopt.h"

#include "mex.h"

#ifdef __cplusplus
extern "C" {
#endif
    
    static const char *fields[] = {"id", "hamming", "goodness", "margin", "H", "centre", "corners"};

    
mxArray * getTag(int width, int height, unsigned char *image)
{

    apriltag_family_t *tf = NULL;
    tf = tag36h11_create();

    tf->black_border = 1;
    apriltag_detector_t *td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);
    td->quad_decimate = 1.0;
    td->quad_sigma = 0.0;
    td->nthreads = 4;
    td->debug = 0;
    td->refine_decode = 1;
    td->refine_pose = 1;
    int quiet = 0;
    int maxiters = 1;
    const int hamm_hist_max = 10;
    image_u8_t *im = NULL;
    mxArray *out;
    int ntags;

    int hamm_hist[hamm_hist_max];
    memset(hamm_hist, 0, sizeof(hamm_hist));
    
    // create the image structure, copy pixels to row-major order
    im = image_u8_create(width, height);

    for (int y=0; y < height; y++) {
        unsigned char *p = &im->buf[y*im->stride];
        unsigned char *q = &image[y];
        for (int x=0; x < width; x++) {
            *p++ = *q;
            q += height;
        }
    }

    // do the detections
    zarray_t *detections = apriltag_detector_detect(td, im);
    ntags = detections->size;
    
    if (ntags == 0)
        return NULL;
     
    // create return structure
    out = mxCreateStructMatrix(1, ntags, 7, fields);
    
    for (int i = 0; i < ntags; i++) {
        apriltag_detection_t *det;
        double *p, *q;
        
        // get the i'th tag
        zarray_get(detections, i, &det);
        
        // save results into a passed MATLAB strucutre
        mxSetField(out, i, "id",       mxCreateDoubleScalar( (double) det->id ) );
        mxSetField(out, i, "hamming",  mxCreateDoubleScalar( (double) det->hamming ) );
        mxSetField(out, i, "goodness", mxCreateDoubleScalar( (double) det->goodness ) );
        mxSetField(out, i, "margin",   mxCreateDoubleScalar( (double) det->decision_margin ) );

        // save the homography as a 3x3 matrix
        mxArray *H = mxCreateDoubleMatrix( 3, 3, mxREAL );
        p = mxGetPr(H);
        q = det->H->data;
        for (int row=0; row<3; row++)
            for (int col=0; col<3; col++)
                p[row+col*3] = *q++;
        mxSetField(out, i, "H", H);
        
        // save the cornersn as 2x4 matrix
        mxArray *corners = mxCreateDoubleMatrix( 2, 4, mxREAL );
        p = mxGetPr(corners);
        for (int row=0; row<4; row++)
            for (int col=0; col<2; col++) 
                p[col+row*2] = det->p[row][col];   // it's transposed
        mxSetField(out, i, "corners", corners );
        
        // save the centre
        mxArray *centre = mxCreateDoubleMatrix( 2, 1, mxREAL );
        p = mxGetPr(centre);
        p[0] = det->c[0];
        p[1] = det->c[1];
        mxSetField(out, i, "centre", centre);
        
	    hamm_hist[det->hamming]++;

	    apriltag_detection_destroy(det);
    }

    zarray_destroy(detections);
    image_u8_destroy(im);

    // don't deallocate contents of inputs; those are the argv
    apriltag_detector_destroy(td);

    tag36h11_destroy(tf);
    return out;
}


#define	IM_IN		prhs[0]

void mexFunction(int nlhs, mxArray *plhs[],
                 int nrhs, const mxArray *prhs[])
{
    // get number of pixels in the image
	int width = mxGetN(IM_IN);
	int height = mxGetM(IM_IN);
    
    unsigned char *im;
    
    if (mxGetNumberOfDimensions(IM_IN) > 2)
    	mexErrMsgTxt("Color images are not supported");

    switch (mxGetClassID(IM_IN)) {
        case mxUINT8_CLASS: {
            
            im = (unsigned char *)mxGetPr(IM_IN);	/* get pointer to image */
            break;
        }
        case mxDOUBLE_CLASS: {
            unsigned char *p = im = (unsigned char *)malloc( width * height);
            double        *q = mxGetPr(IM_IN);            

            // type convert
            for (int i=0; i<width*height; i++)
                    *p++ = (unsigned char) *q++ * 255;
            break;
        }
        default:
            mexErrMsgTxt("Only uint8 or double images allowed");
    }
    
    // find the tags
    plhs[0] = getTag(width, height, im);
    
    if (plhs[0] == NULL)
        plhs[0] = mxCreateDoubleMatrix( 0, 0, mxREAL );  // return []
}

#ifdef __cplusplus
}
#endif

