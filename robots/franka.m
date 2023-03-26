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

        
        q_init = zeros(7,1); % The initial configuration q_init = [ 0 , pi/6 , 0 , -pi/3 , 0, pi/2, 0 ]';

        % The locations
        COMs = [       0, -18.7e-3, 101.6e-3;
                -0.21e-3,  25.0e-3,  82.5e-3;
                -0.20e-3,  19.5e-3,  98.4e-3;
                -0.21e-3, -20.1e-3,    86e-3;
                -0.04e-3, -13.5e-3,    66e-3;
                -0.35e-3,  51.4e-3,  17.1e-3;
                -0.01e-3,   0.1e-3,    11e-3]';

        % End-effector origin
        % [TODO] [2023.03.25]
        % Currently, there is no tool attached to the robot.
        % Hence, we simply set as a zero array
        AxisOriginFlange = [ 0, 0, 0 ]';   
        
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
            obj.Masses = [ 2.7426, 4.9464, 2.5451, 4.6376, 1.7140, 2.4272, 0.4219 ];

            % The inertia matrix of the robot, nqx6
            obj.Inertias = [  0.2400, 0.0240, 0.0128, 0, 0, 0;
                              0.0468, 0.0282, 0.0101, 0, 0, 0;
                              0.0200, 0.0200, 0.0600, 0, 0, 0;
                              0.0400, 0.0270, 0.0100, 0, 0, 0;
                              0.0190, 0.0160, 0.0120, 0, 0, 0;
                              0.0070, 0.0060, 0.0050, 0, 0, 0;
                              0.0003, 0.0003, 0.0005, 0, 0, 0 ];

            % ================================ %
            % ======= Joint Properties ======= %
            % ================================ %

            % All of the joints are revolute joints
            obj.JointTypes = ones( 1, obj.nq );

            % max/min of q array robot
            obj.q_max  = [ 2.8973; 1.7628; 2.8973; -0.0698; 2.8973; 3.7525; 2.8973 ];
            obj.q_min  = -obj.q_max;

            % max/min of dq array robot
            obj.dq_max =  [ 2.1750; 2.1750; 2.1750; 2.1750; 2.6100; 2.6100; 2.6100 ];
            obj.dq_min = -obj.dq_max;

            % max/min of ddq array robot
            obj.ddq_max = deg2rad( 300 ) * ones( obj.nq, 1 );
            obj.ddq_max = [ 15; 7.5; 10; 12.5; 15; 20; 20 ];
            obj.ddq_min = -obj.ddq_max;

            % The axis origin of the robot at initial configuration
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
            obj.H_COM_init( 1:3, 4, : ) = obj.AxisOrigins + obj.COMs;

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