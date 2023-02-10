classdef iiwa14 < RobotPrimitive & handle

    properties

        % ========================================== %
        % ===== Properties for KUKA LBR iiwa14 ===== %
        % ========================================== %
        q_init = [ 0 , pi/6 , 0 , -pi/3 , 0, pi/2, 0 ]';

        % The locations
        COMs = [ 0.0, -14.0e-3, 102.0e-3;
                 0.0, 16.0e-3, 64.0e-3;
                 0.0, 19.0e-3, 98.0e-3;
                 0.0, -20.0e-3, 86.0e-3;
                 0.0, -13.0e-3, 66.0e-3;
                 0.0, 60.0e-3, 16.0e-3;
                 0.0, 0.0e-3, 11.0e-3 ]';

        % End-effector origin
        AxisOriginFlange = [ 0, 0 , 31.4e-3 ]';


    end

    methods
        function obj = iiwa14( varargin )
            % Currently, the varargin is passed to either choose ('high' or 'low' quality)

            % ======================================================= %
            % ============ BASIC PROPERTIES OF THE ROBOT ============ %
            % ======================================================= %
            obj.Name   = 'iiwa14';
            obj.nq     = 7;

            % Parents of the object
            obj.ParentID = 0:obj.nq;

            % Robot simulation in 3D
            obj.Dimension = 3;

            % ======================================================= %
            % ====== GEOMETRIC/INERTIA PROPERTIES OF THE ROBOT ====== %
            % ======================================================= %

            % Mass of the robot 1xnq
            obj.Masses = [ 6.404, 7.89, 2.54, 4.82, 1.76, 2.5, 0.42 ];

            % The inertia matrix of the robot, nqx3
            obj.Inertias = [ 0.069,  0.071, 0.02;
                             0.08, 0.08, 0.01;
                             0.02,   0.02, 0.06;
                             0.04,  0.03, 0.01;
                             0.01,  0.01, 0.01;
                             0.007,  0.006, 0.005;
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
            obj.q_max  =  func_deg2rad( [ 163; 113; 163; 115; 160; 110; 165 ], obj.JointTypes );
            obj.q_min  = -obj.q_max;

            % max/min of dq array robot
            obj.dq_max =  func_deg2rad( [ 150; 150; 150; 150; 150; 150; 150 ], obj.JointTypes );
            obj.dq_min = -obj.dq_max;

            % max/min of ddq array robot
            obj.ddq_max = deg2rad( 300 ) * ones( obj.nq, 1 );
            obj.ddq_min = -obj.ddq_max;

            % The axis origin of the robot at initial configuration
            obj.AxisOrigins = [ 0, 0, 152.5e-3;
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

            % varargin can be used to specify the animation quality
            % Default is highQuality
            obj.Quality = 'highQuality';
            idq = find( strcmp( 'quality', varargin ) );
            
            if ~isempty( idq )
                if ( strcmp( 'low', varargin{ idq + 1 } ) )
                    obj.Quality = 'lowQuality';
                elseif ( strcmp( 'high', varargin{ idq + 1 } ) )
                    % kepp high quality
                else
                    warning( 'Only "low" and "high" quality implemented. Will keep high quality.' );
                end
            end

            % varargin can be used as the directory to the mat file
            % We read out fields with vertices and faces information.
            idx = find( strcmp( 'dirname', varargin ) );
         
            if ~isempty( idx )
                % If the directory exists, assign the .mat file
                file_name = varargin{ idx + 1 };
                obj.gObjs = load( file_name );
            else
                file_name = [ './graphics/', obj.Quality, '/iiwa14.mat' ];
                obj.gObjs = load( file_name );
            end

        end


    end
end