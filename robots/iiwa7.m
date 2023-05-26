classdef iiwa7 < RobotPrimitive & handle
    % Constructs KUKA LBR iiwa 7 
    % 
    % The robot consists of 7 linkages, constructed from the .obj files.
    % 
    % Parameters
    % ----------
    %     n : int
    %         The number of linkages for the snakebot.
    %     m_arr : float array
    %         The mass (kg) array of the n linkages, size of 1 x n
    %     l_arr : float array
    %         The length (m) array of the n linkages, size of 1 x n
    % 
    % Error
    % -----
    %    `n` must be an integer. `m_arr` and `l_arr` must all have positive values.

    properties

        % ========================================== %
        % ===== Properties for KUKA LBR iiwa7 ====== %
        % ========================================== %
        q_init = [ 0 , pi/6 , 0 , -pi/3 , 0, pi/2, 0 ]';

        % The locations with respect to the joint location.
        COMs = [       0, -18.7e-3, 101.6e-3;
                -0.21e-3,  25.0e-3,  82.5e-3;
                -0.20e-3,  19.5e-3,  98.4e-3;
                -0.21e-3, -20.1e-3,    86e-3;
                -0.04e-3, -13.5e-3,    66e-3;
                -0.35e-3,  51.4e-3,  17.1e-3;
                -0.01e-3,   0.1e-3,    11e-3]';

        % End-effector origin
        AxisOriginFlange = [ 0, 0 , 31.4e-3 ]';


    end

    methods
        function obj = iiwa7( quality )
            % Currently, the varargin is passed to either choose ('high' or 'low' quality)

            % ======================================================= %
            % ============ BASIC PROPERTIES OF THE ROBOT ============ %
            % ======================================================= %
            obj.Name      = 'iiwa7';
            obj.nq        = 7;
            obj.ParentID  = 0:obj.nq;
            obj.Dimension = 3;

            % Quality of the animation should be either `high` or `low`.
            assert( strcmp( quality, 'high' ) || strcmp( quality, 'low' ) )
            obj.Quality = quality;

            % ======================================================= %
            % ====== GEOMETRIC/INERTIA PROPERTIES OF THE ROBOT ====== %
            % ======================================================= %

            % Mass of the robot 1xnq
            obj.Masses = [ 2.7426, 4.9464, 2.5451, 4.6376, 1.7140, 2.4272, 0.4219 ];

            % The (6 x nq) inertia matrix of the robot
            % Ordered in Ixx, Iyy, Izz, Ixy, Ixz, Iyz
            obj.Inertias = [  0.2400,  0.0240, 0.0128, 0, 0, 0;
                              0.0468,  0.0282, 0.0101, 0, 0, 0;
                              0.0200,  0.0200, 0.0600, 0, 0, 0;
                              0.0400,  0.0270, 0.0100, 0, 0, 0;
                              0.0190,  0.0160, 0.0120, 0, 0, 0;
                              0.0070,  0.0060, 0.0050, 0, 0, 0;
                              0.0003,  0.0003, 0.0005, 0, 0, 0 ]';

            % ================================ %
            % ======= Joint Properties ======= %
            % ================================ %

            % All of the joints are revolute joints
            obj.JointTypes = ones( 1, obj.nq );

            % max/min of q array robot
            obj.q_max  =  [ 170; 120; 170; 120; 170; 120; 175 ] * pi/180;
            obj.q_min  = -obj.q_max;

            % max/min of dq array robot
            obj.dq_max =  [ 100; 110; 100; 130; 130; 180; 180 ] * pi/180;
            obj.dq_min = -obj.dq_max;

            % max/min of ddq array robot
            obj.ddq_max = 300 * ones( obj.nq, 1 ) * pi/180;
            obj.ddq_min = -obj.ddq_max;

            % The (3xnq) axis origins of the robot at initial configuration
            % Each column array expresses the position of {i+1}-th joint
            % with respect to {i}-th joint, w.r.t. frame {S} attached at 
            % the base of the robot.
            obj.AxisOrigins = [ 0,      0, 152.5e-3;
                                0, -11e-3, 187.5e-3;
                                0,  11e-3, 212.5e-3;
                                0,  11e-3, 187.5e-3;
                                0, -11e-3, 212.5e-3;
                                0, -62e-3, 187.5e-3;
                                0,  62e-3,  79.6e-3 ]';

            % We conduct a cumsum along the 2nd-dimension 
            obj.AxisOrigins = cumsum( obj.AxisOrigins, 2 );

            % Axis Direction (3xnq)
            obj.AxisDirections = [ 0,  0, 1;
                                   0,  1, 0;
                                   0,  0, 1;
                                   0, -1, 0;
                                   0,  0, 1;
                                   0,  1, 0;
                                   0,  0, 1 ]';

            % ======================================================= %
            % =========== INITIAL H MATRICES OF THE ROBOT =========== %
            % ======================================================= %

            % Get initial transformation of point on each joint
            obj.H_init = repmat( eye( 4 ), [ 1, 1, obj.nq + 1 ] );
            obj.H_init( 1:3, 4, 1:obj.nq ) = obj.AxisOrigins;

            % The end-effector of the robot
            obj.H_init( 1:3, 4, end ) = obj.AxisOrigins( :, end ) + obj.AxisOriginFlange;

            % Get initial transformations of point on COM of each link
            obj.H_COM_init = repmat( eye( 4 ), [ 1, 1, obj.nq ] );
            obj.H_COM_init( 1:3, 4, : ) = obj.AxisOrigins + obj.COMs;

            % =================================== %
            % ====== Animation Properties ======= %
            % =================================== %
            
            % Read the .mat file
            % We simply use the relative directory to this .m file
            % This will allow us to run the iiwa7 file from anywhere 
            % Get the directory containing the script
            [ currentDir, ~, ~ ] = fileparts( mfilename('fullpath') );
            file_name = [ currentDir, '/../graphics/', obj.Quality, 'Quality/iiwa7.mat' ];
            obj.gObjs = load( file_name );

        end


    end
end