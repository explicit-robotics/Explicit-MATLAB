classdef franka < RobotPrimitive & handle

    properties

        % =================================================== %
        % =========== Properties for Franka robot =========== %
        % Kinematic Data from: https://frankaemika.github.io/docs/...
        % control_parameters.html#denavithartenberg-parameters%
        % Inertial data from: Identification of the Franka Emika
        % PandaRobot With Retrieval of Feasible Parameters Using
        % Penalty-Based Optimization by: Claudio Gaz, Marco Cognetti, Alexander Oliva,
        % Paolo Robuffo Giordano, Alessandro de Luca
        % =================================================== %

        
        % The initial configuration 
        q_init = deg2rad([ 0 , -30 , 0 , 90 , 0, -120, 0 ]');

        % The locations
        % Note that for Franka robot, the COM is the position 
        % with respect to the fixed {S} frame.
        COMs =  [ 0.0039, -0.0031, 0.0275,  0.0293, -0.0120, 0.0601, 0.0883;
                  0.0021,  0.0036, 0.0392, -0.0275,  0.0410, 0.0105, 0.0021;
                  0.2394,  0.3618, 0.5825,  0.7534,  0.9946, 1.0189, 0.9339 ];

        % End-effector origin
        AxisOriginFlange = [ 0, 0, -0.107 ]';   
        
    end

    methods
        function obj = franka(  )
            % ======================================================= %
            % ============ BASIC PROPERTIES OF THE ROBOT ============ %
            % ======================================================= %
            obj.Name      = 'franka';
            obj.nq        = 7;
            obj.ParentID  = 0:obj.nq;
            obj.Dimension = 3;

            % ======================================================= %
            % ====== GEOMETRIC/INERTIA PROPERTIES OF THE ROBOT ====== %
            % ======================================================= %

            % Mass of the robot 1xnq
            % Data from Fig. 4 of:
            % [2021 ICRA] RIL: Riemannian Incremental Learning of the ..
            %                  Inertial Properties of the Robot Body Schema
            obj.Masses = [ 2.7426, 4.9464, 2.5451, 4.6376, 1.7140, 2.4272, 0.4219 ];

            % The inertia matrix of the robot, 6xnq
            % Ordered in Ixx, Iyy, Izz, Ixy, Ixz, Iyz
            obj.Inertias = [  0.7470,  0.7503,  0.0092, -0.0002,  0.0086,  0.0201;
                              0.0085,  0.0265,  0.0281,  0.0103,  0.0040, -0.0008;
                              0.0565,  0.0529,  0.0182, -0.0082, -0.0055, -0.0044;
                              0.0677,  0.0776,  0.0324, -0.0039,  0.0277,  0.0016;
                              0.0394,  0.0315,  0.0109, -0.0015, -0.0046,  0.0022;
                              0.0025,  0.0118,  0.0106,  0.0001,  0.0015, -0.0001;
                              0.0308,  0.0284,  0.0067, -0.0004,  0.0007, -0.0005 ]';

            % ================================ %
            % ======= Joint Properties ======= %
            % ================================ %

            % All of the joints are revolute joints
            obj.JointTypes = ones( 1, obj.nq );

            % max/min of q array robot
            obj.q_max  =  [ 2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973 ];
            obj.q_min  = -obj.q_max;

            % max/min of dq array robot
            obj.dq_max =  [ 2.1750, 2.1750, 2.1750,  2.1750, 2.6100, 2.6100, 2.6100 ];
            obj.dq_min = -obj.dq_max;

            % max/min of ddq array robot
            obj.ddq_max = deg2rad( 300 ) * ones( obj.nq, 1 );
            obj.ddq_max = [ 15; 7.5; 10; 12.5; 15; 20; 20 ];
            obj.ddq_min = -obj.ddq_max;

            % The (3xnq) axis origin of the robot at initial configuration
            obj.AxisOrigins = [ 0.0000, 0, 0.000;
                                0.0000, 0, 0.333;
                                0.0000, 0, 0.316;
                                0.0825, 0, 0.000;
                               -0.0825, 0, 0.384;     
                                0.0000, 0, 0.000;
                                0.0880, 0, 0.000 ]';

            % We conduct a cumsum to get the Axis Origin
            obj.AxisOrigins = cumsum( obj.AxisOrigins, 2 );             

            % Axis Direction
            obj.AxisDirections = [ 0,   0,  1;
                                   0,  -1,  0;
                                   0,   0,  1;
                                   0,   1,  0;
                                   0,   0,  1;
                                   0,   1,  0;
                                   0,   0, -1 ]';

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

            % Again, we emphasize the for Franka robot, 
            % COMs are defined w.r.t. the {S} frame.
            obj.H_COM_init( 1:3, 4, : ) = obj.COMs;

            % =================================== %
            % ====== Animation Properties ======= %
            % =================================== %
            
            % Currently, we only have low quality file for franka
            [ currentDir, ~, ~ ] = fileparts( mfilename('fullpath') );
            file_name = [ currentDir, '/../graphics/lowQuality/franka.mat' ];
            obj.gObjs = load( file_name );

    
        end


    end
end