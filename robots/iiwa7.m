classdef iiwa7 < RobotPrimitive & handle

    properties

        % ========================================== %
        % ===== Properties for KUKA LBR iiwa7 ====== %
        % ========================================== %

        q_init = [ 0 , 45 , 0 , -70 , 0, 70, 10 ]';

        % The locations
        COMs = [       0, -18.7e-3, 101.6e-3;
            -0.21e-3,  25.0e-3,  82.5e-3;
            -0.20e-3,  19.5e-3,  98.4e-3;
            -0.21e-3, -20.1e-3,    86e-3;
            -0.04e-3, -13.5e-3,    66e-3;
            -0.35e-3,  51.4e-3,  17.1e-3;
            -0.01e-3,   0.1e-3,    11e-3]';

        % End-effector origin
        AxisOriginFlange = [ 0, 0 , 31.4e-3]';


    end

    methods
        function obj = iiwa7( varargin )
            % Currently, the varargin is passed to either choose ('high' or 'low' quality)

            % ======================================================= %
            % ============ BASIC PROPERTIES OF THE ROBOT ============ %
            % ======================================================= %
            obj.Name   = 'iiwa7';
            obj.nq     = 7;

            % Parents of the object
            obj.ParentID = 0:obj.nq;

            % Robot simulation in 3D
            obj.Dimension = 3;

            % ======================================================= %
            % ====== GEOMETRIC/INERTIA PROPERTIES OF THE ROBOT ====== %
            % ======================================================= %

            % Mass of the robot 1xnq
            obj.Masses = [ 2.7426, 4.9464, 2.5451, 4.6376, 1.7140, 2.4272, 0.4219 ];

            % The inertia matrix of the robot, nqx3
            obj.Inertias = [  0.24,  0.024, 0.0128;
                0.0468, 0.0282, 0.0101;
                0.02,   0.02, 0.0600;
                0.04,  0.027, 0.0100;
                0.019,  0.016, 0.0120;
                0.007,  0.006, 0.0050;
                0.0003, 0.0003, 0.0005 ]';

            % The generalized mass matrix
            obj.M_Mat = zeros( 6, 6, obj.nq );

            for i = 1:obj.nq
                obj.M_Mat( 1:3, 1:3, i ) = obj.Masses( i ) * eye( 3 );
                obj.M_Mat( 4:6, 4:6, i ) = diag( obj.Inertias( :, i ) );
            end

            % ================================ %
            % ======= Joint Properties ======= %
            % ================================ %

            % All of the joints are revolute joints
            obj.JointTypes = ones( 1, obj.nq );

            % max/min of q array robot
            obj.q_max  =  func_deg2rad( [ 170; 120; 170; 120; 170; 120; 175 ], obj.JointTypes );
            obj.q_min  = -obj.q_max;

            % max/min of dq array robot
            obj.dq_max =  func_deg2rad( [ 100; 110; 100; 130; 130; 180; 180 ], obj.JointTypes );
            obj.dq_min = -obj.dq_max;

            % max/min of ddq array robot
            obj.ddq_max = deg2rad( 300 ) * ones( obj.nq, 1 );
            obj.ddq_min = -obj.ddq_max;

            % The axis origin of the robot at initial configuration
            obj.AxisOrigins = [ 0,      0, 152.5e-3;
                0, -11e-3, 187.5e-3;
                0,  11e-3, 212.5e-3;
                0,  11e-3, 187.5e-3;
                0, -11e-3, 212.5e-3;
                0, -62e-3, 187.5e-3;
                0,  62e-3,  79.6e-3]';

            % We conduct a cumsum to get the Axis Origin
            obj.AxisOrigins = cumsum( obj.AxisOrigins, 2 );

            % Axis Direction
            obj.AxisDirections = [ 0,  0, 1;
                0,  1, 0;
                0,  0, 1;
                0, -1, 0;
                0,  0, 1;
                0,  1, 0;
                0,  0, 1]';

            % ======================================================= %
            % =========== INITIAL H MATRICES OF THE ROBOT =========== %
            % ======================================================= %

            % get initial transformation of point on each joint
            obj.H_init = repmat( eye( 4 ), [ 1, 1, obj.nq + 1 ] );
            obj.H_init( 1:3, 4, 1:obj.nq ) = obj.AxisOrigins;

            % The end-effector of the robot
            obj.H_init( 1:3, 4, end ) = obj.AxisOrigins( :, end ) + obj.AxisOriginFlange;

            % get initial transformations of point on COM of each link
            obj.H_COM_init = repmat( eye( 4 ), [ 1, 1, obj.nq ] );
            obj.H_COM_init( 1:3, 4, : ) = obj.AxisOrigins + obj.COMs;

            % =================================== %
            % ====== Animation Properties ======= %
            % =================================== %

            % We get the varargin as the directory to the mat file.
            % We read out fields with vertices and faces information.
            idx = find( strcmp( 'dirname', varargin ) );
            if ~isempty( idx )
                % If the directory exists, assign the .mat file
                file_name = varargin{ idx + 1 };
                obj.gObjs = load( file_name );
            end

        end


    end
end