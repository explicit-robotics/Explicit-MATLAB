classdef iiwa14 < RobotPrimitive & handle

    properties

        % ========================================== %
        % ===== Properties for KUKA LBR iiwa14 ===== %
        % ========================================== %
        q_init = [ 0 , pi/6 , 0 , -pi/3 , 0, pi/2, 0 ]';

        % The locations
        COMs = [ 0.0, -14.0e-3, 102.0e-3;
                 0.0,  16.0e-3,  64.0e-3;
                 0.0,  19.0e-3,  98.0e-3;
                 0.0, -20.0e-3,  86.0e-3;
                 0.0, -13.0e-3,  66.0e-3;
                 0.0,  60.0e-3,  16.0e-3;
                 0.0,   0.0e-3,  11.0e-3 ]';

        % End-effector origin
        % AxisOriginFlange = [ 0, 0 , 31.4e-3 ]';
        AxisOriginFlange = [ 0, 0 , 0 ]';


    end

    methods
        function obj = iiwa14( quality )
            % Currently, the varargin is passed to either choose ('high' or 'low' quality)

            % ======================================================= %
            % ============ BASIC PROPERTIES OF THE ROBOT ============ %
            % ======================================================= %
            obj.Name      = 'iiwa14';
            obj.nq        = 7;
            obj.ParentID  = 0:obj.nq;
            obj.Dimension = 3;

            % Quality of the animation should be either `high` or `low`.
            assert( strcmp( quality, 'high' ) || strcmp( quality, 'low' ) )
            obj.Quality = quality;
            obj.Dimension = 3;

            % ======================================================= %
            % ====== GEOMETRIC/INERTIA PROPERTIES OF THE ROBOT ====== %
            % ======================================================= %

            % Mass of the robot 1xnq
            obj.Masses = [ 6.404, 7.89, 2.54, 4.82, 1.76, 2.5, 0.42 ];

            % The inertia matrix of the robot, nqx6
            % Ordered in Ixx, Iyy, Izz, Ixy, Ixz, Iyz
            obj.Inertias = [ 0.0690,  0.0710, 0.0200, 0, 0, 0;
                             0.0800,  0.0800, 0.0100, 0, 0, 0;
                             0.0200,  0.0200, 0.0600, 0, 0, 0;
                             0.0400,  0.0300, 0.0100, 0, 0, 0;
                             0.0100,  0.0100, 0.0100, 0, 0, 0;
                             0.0070,  0.0060, 0.0050, 0, 0, 0;
                             0.0003,  0.0003, 0.0005, 0, 0, 0 ];

            % ================================ %
            % ======= Joint Properties ======= %
            % ================================ %

            % All of the joints are revolute joints
            obj.JointTypes = ones( 1, obj.nq );

            % max/min of q array robot
            obj.q_max  =  func_deg2rad( [ 163; 113; 163; 115; 160; 110; 165 ], obj.JointTypes );
            obj.q_min  = -obj.q_max;

            % max/min of dq array robot
            obj.dq_max =  func_deg2rad( [ 150; 150; 150; 150; 150; 150; 150 ], obj.JointTypes );
            obj.dq_min = -obj.dq_max;

            % max/min of ddq array robot
            obj.ddq_max = deg2rad( 300 ) * ones( obj.nq, 1 );
            obj.ddq_min = -obj.ddq_max;

            % The axis origin of the robot at initial configuration
            obj.AxisOrigins = [ 0,      0, 152.5e-3;
                                0, -13e-3, 207.5e-3;
                                0,  13e-3, 232.5e-3;
                                0,  11e-3, 187.5e-3;
                                0, -11e-3, 212.5e-3;
                                0, -62e-3, 187.5e-3;
                                0,  62e-3,  79.6e-3 ]';

            % We conduct a cumsum to get the Axis Origin
            obj.AxisOrigins = cumsum( obj.AxisOrigins, 2 );

            % Axis Direction
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

            [ currentDir, ~, ~ ] = fileparts( mfilename('fullpath') );
            file_name = [ currentDir, '/../graphics/', obj.Quality, 'Quality/iiwa7.mat' ];
            obj.gObjs = load( file_name );
        end


    end
end