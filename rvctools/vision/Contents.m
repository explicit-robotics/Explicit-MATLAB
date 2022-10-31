% Machine Vision Toolbox for MATLAB
% Version 4.3  25-Jul-2018
%
%
% Color
%    blackbody                                 - Compute blackbody emission spectrum
%    ccdresponse                               - CCD spectral response
%    cie_primaries                             - Define CIE primary colors
%    cmfrgb                                    - RGB color matching function
%    cmfxyz                                    - matching function
%    ccxyz                                     - XYZ chromaticity coordinates
%    colordistance                             - Colorspace distance
%    colorname                                 - Map between color names and RGB values
%    colorspace                                - varargout = colorspace(Conversion,varargin)
%    lambda2rg                                 - RGB chromaticity coordinates
%    lambda2xy                                 - XY = LAMBDA2XY(LAMBDA) is the xy-chromaticity coordinate (1x2) for
%    loadspectrum                              - Load spectrum data
%    luminos                                   - Photopic luminosity function
%    rg_addticks                               - Label spectral locus
%    rgb2xyz                                   - RGB to XYZ color space
%    rgbcube                                   - rgbcube()
%    rluminos                                  - Relative photopic luminosity function
%    showcolorspace                            - Display spectral locus
%    tristim2cc                                - Tristimulus to chromaticity coordinates
%    yuv2rgb                                   - YUV format to RGB
%    yuv2rgb2                                  - YUV format to RGB
%
% Camera models
%    Camera                                    - Camera superclass
%    CentralCamera                             - Perspective camera class
%    CatadioptricCamera                        - Catadioptric camera class
%    FishEyeCamera                             - Fish eye camera class
%    SphericalCamera                           - Spherical camera class
%    camcald                                   - CAMCALD Camera calibration from data points
%    invcamcal                                 - camera calibration
%
% Image sources
%
%     Devices
%        AxisWebCamera                         - acquire from internet webcam
%        EarthView                             - acquire image from Google Earth
%        ImageSource                           - abstract superclass
%        Movie                                 - acquire from a local movie file
%        VideoCamera                           - acquire from attached video camera or webcam
%        VideoCamera_IAT                       - Class to read from local video camera
%        VideoCamera_fg                        - Class to read from local video camera
%        YUV                                   - Class to read YUV4MPEG file
%
%     Test patterns
%        mkcube                                - Create cube
%        mkgrid                                - Create grid of points
%        testpattern                           - Create test images
%
% Monadic operators
%    icolor                                    - Colorize a greyscale image
%    colorize                                  - Colorize a greyscale image
%    igamm                                     - correction
%    imono                                     - Convert color image to monochrome
%    inormhist                                 - Histogram normalization
%    istretch                                  - Image normalization
%
%     Type changing
%        idouble                               - Convert integer image to double
%        iint                                  - Convert image to integer class
%
% Diadic operators
%    ipixswitch                                - Pixelwise image merge
%
% Spatial operators
%
%     Linear operators
%        icanny                                - edge detection
%        iconvolve                             - Image convolution
%        ismooth                               - Gaussian smoothing
%        isobel                                - Sobel edge detector
%        radgrad                               - Radial gradient
%
%         Kernels
%            kcircle                           - Circular structuring element
%            kdgauss                           - Derivative of Gaussian kernel
%            kdog                              - Difference of Gaussian kernel
%            kgauss                            - Gaussian kernel
%            klaplace                          - Laplacian kernel
%            klog                              - Laplacian of Gaussian kernel
%            ksobel                            - Sobel edge detector
%            ktriangle                         - Triangular kernel
%
%     Non-linear operators
%        dtransform                            - Distance transform
%        irank                                 - Rank filter
%        ivar                                  - Pixel window statistics
%        iwindow                               - Generalized spatial operator
%
%     Morphological
%        idilate                               - Morphological dilation
%        ierode                                - Morphological erosion
%        iclose                                - closing
%        iopen                                 - Morphological opening
%        imorph                                - Morphological neighbourhood processing
%        hitormiss                             - Hit or miss transform
%        ithin                                 - Morphological skeletonization
%        iendpoint                             - Find end points in a binary skeleton image
%        itriplepoint                          - Find triple points
%        morphdemo                             - Demonstrate morphology using animation
%
%     Similarity
%        imatch                                - Template matching
%        isimilarity                           - Locate template in image
%        sad                                   - Sum of absolute differences
%        ssd                                   - Sum of squared differences
%        ncc                                   - Normalized cross correlation
%        zsad                                  - Sum of absolute differences
%        zssd                                  - Sum of squared differences
%        zncc                                  - Normalized cross correlation
%
% Features
%
%     Region features
%        RegionFeature                         - Region feature class
%        colorkmeans                           - Color image segmentation by clustering
%        colorseg                              - Color image segmentation using k-means
%        ithresh                               - Interactive image threshold
%        imoments                              - Image moments
%        ibbox                                 - Find bounding box
%        iblobs                                - features
%        igraphseg                             - Graph-based image segmentation
%        ilabel                                - Label an image
%        imser                                 - Maximally stable extremal regions
%        niblack                               - Adaptive thresholding
%        otsu                                  - Threshold selection
%
%     Line features
%        Hough                                 - Hough transform class
%        LineFeature                           - Line feature class
%
%         Point features
%            PointFeature                      - PointCorner feature object
%            OrientedScalePointFeature         - ScalePointCorner feature object
%            ScalePointFeature                 - ScalePointCorner feature object
%            SiftPointFeature                  - SIFT point corner feature object
%            SurfPointFeature                  - SURF point corner feature object
%            icorner                           - Corner detector
%            iscalespace                       - Scale-space image sequence
%            iscalemax                         - Scale space maxima
%            isift                             - SIFT feature extractor
%            isurf                             - SURF feature extractor
%            FeatureMatch                      - Feature correspondence object
%
%         Other features
%            apriltags                         - Read April tags from image
%            peak                              - Find peaks in vector
%            peak2                             - Find peaks in a matrix
%            ihist                             - Image histogram
%            hist2d                            - MEX file to compute 2-D histogram.
%            iprofile                          - Extract pixels along a line
%
% Multiview
%
%     Geometric
%        epidist                               - Distance of point from epipolar line
%        epiline                               - Draw epipolar lines
%        fmatrix                               - Estimate fundamental matrix
%        homography                            - Estimate homography
%
%     Stereo
%        istereo                               - Stereo matching
%        anaglyph                              - Convert stereo images to an anaglyph image
%        stdisp                                - Display stereo pair
%        irectify                              - Rectify stereo image pair
%
% Image sequence
%    BagOfWords                                - Bag of words class
%    BundleAdjust                              - BundleAdjust < matlab.mixin.Copyable
%    ianimate                                  - Display an image sequence
%    Tracker                                   - Track points in image sequence
%
% Shape changing
%    homwarp                                   - Warp image by an homography
%    idecimate                                 - an image
%    ipad                                      - Pad an image with constants
%    ipyramid                                  - Pyramidal image decomposition
%    ireplicate                                - Expand image
%    iroi                                      - Extract region of interest
%    irotate                                   - Rotate image
%    isamesize                                 - Automatic image trimming
%    iscale                                    - Scale an image
%    itrim                                     - Trim images
%
% Utility
%
%     Image utility
%        idisp                                 - image display tool
%        idisplabel                            - Display an image with mask
%        iread                                 - Read image from file
%        pnmfilt                               - Pipe image through PNM utility
%        showpixels                            - Show low resolution image
%
%     Image generation
%        iconcat                               - Concatenate images
%        iline                                 - Draw a line in an image
%        ipaste                                - Paste an image into an image
%
%     Moments
%        humoments                             - Hu moments
%        mpq                                   - Image moments
%        mpq_poly                              - Polygon moments
%        upq                                   - Central image moments
%        upq_poly                              - Central polygon moments
%        npq                                   - Normalized central image moments
%        npq_poly                              - Normalized central polygon moments
%
%     Plotting
%        plot_arrow                            - draw an arrow
%        plot_box                              - draw a box
%        plot_circle                           - draw a circle
%        plot_ellipse                          - draw an ellipse
%        plot_homline                          - plot homogeneous line
%        plot_point                            - plot points
%        plot_poly                             - plot polygon
%        plot_sphere                           - draw a sphere
%
%     Homogeneous coordinates
%        e2h                                   - Euclidean coordinates to homogeneous
%        h2e                                   - homogeneous coordinates to Euclidean
%        homline                               - 
%        homtrans                              - apply homogeneous transform to points
%        skew                                  - create skew symmetric matrix
%
%     Homogeneous coordinates in 2D
%        ishomog2                              - Test if SE(2) homogeneous transformation matrix
%        rot2                                  - SO(2) Rotation matrix
%        transl2                               - Create or unpack an SE(2) translational homogeneous transform
%        trexp2                                - matrix exponential for so(2) and se(2)
%        SE2                                   - Representation of 2D rigid-body motion
%        SO2                                   - Representation of 2D rotation
%
%     Homogeneous coordinates in 3D
%        angvec2r                              - Convert angle and vector orientation to a rotation matrix
%        delta2tr                              - Convert differential motion  to a homogeneous transform
%        ishomog                               - Test if SE(3) homogeneous transformation matrix
%        isrot                                 - Test if SO(3) rotation matrix
%        rotz                                  - Rotation about Z axis
%        rt2tr                                 - Convert rotation and translation to homogeneous transform
%        tr2angvec                             - Convert rotation matrix to angle-vector form
%        tr2delta                              - Convert homogeneous transform to differential motion
%        tr2rpy                                - Convert a homogeneous transform to roll-pitch-yaw angles
%        transl                                - Create or unpack an SE(3) translational homogeneous transform
%        trexp                                 - matrix exponential for so(3) and se(3)
%        trinterp                              - Interpolate SE(3) homogeneous transformations
%        trlog                                 - logarithm of SO(3) or SE(3) matrix
%        trnorm                                - Normalize a rotation matrix
%        Twist                                 - SE(2) and SE(3) Twist class
%        SE3                                   - SE(3) homogeneous transformation class
%        SO3                                   - Representation of 3D rotation
%        vex                                   - Convert skew-symmetric matrix to vector
%
%     3D geometry
%        icp                                   - Point cloud alignment
%        Plucker                               - Plucker coordinate class
%        Ray3D                                 - Ray in 3D space
%
%     Integral image
%        iisum                                 - Sum of integral image
%        intgimage                             - Compute integral image
%
%     Edges and lines
%        bresenham                             - Generate a line
%        edgelist                              - Return list of edge pixels for region
%
%     General
%        about                                 - Compact display of variable type
%        chi2inv_rtb                           - Inverse chi-squared function
%        closest                               - Find closest points in N-dimensional space.
%        col2im                                - Convert pixel vector to image
%        colnorm                               - Column-wise norm of a matrix
%        distance                              - Euclidean distances between sets of points
%        filt1d                                - 1-dimensional rank filter
%        iconv                                 - Image cross-correlation
%        im2col                                - Convert an image to pixel per row format
%        imeshgrid                             - Domain matrices for image
%        iscolor                               - Test for color image
%        isize                                 - Size of image
%        isvec                                 - Test if vector
%        kmeans                                - K-means clustering
%        numcols                               - Number of columns in matrix
%        numrows                               - Number of rows in matrix
%        pickregion                            - Pick a rectangular region of a figure using mouse
%        polydiff                              - Differentiate a polynomial
%        ransac                                - Random sample and consensus
%        tb_optparse                           - Standard option parser for Toolbox functions
%        unit                                  - Unitize a vector
%        usefig                                - figure windows
%        xaxis                                 - Set X-axis scaling
%        yaxis                                 - Y-axis scaling
%        xyzlabel                              - Label X, Y and Z axes
%        zcross                                - Zero-crossing detector
%        RTBPose                               - Superclass for SO2, SO3, SE2, SE3
%
% Copyright (C) 2011 Peter Corke
