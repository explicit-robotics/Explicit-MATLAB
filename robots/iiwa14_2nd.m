classdef iiwa14_2nd < RobotPrimitive & handle

    properties

        % ========================================== %
        % ===== Properties for KUKA LBR iiwa14 ===== %
        % ========================================== %
        q_init = [ 0 , pi/6 , 0 , -pi/3 , 0, pi/2, 0 ]';

        % The locations
        % Data from the open Github repository of 
        % Toyota Research Institute:
        COMs = [ 0.0000, -0.0300, 0.1200;
                -0.0003,  0.0420, 0.2615;
                 0.0000,  0.0300, 0.5370;
                 0.0000, -0.0340, 0.6895;
                -0.0001, -0.0210, 0.8830;
                 0.0000,  0.0004, 1.0231;
                 0.0000,  0.0000, 1.1235]';

        % AxisOriginFlange = [ 0, 0 , 31.4e-3 ]';
        AxisOriginFlange = [ 0, 0 , 0 ]';             
    end

    methods
        function obj = iiwa14_2nd(  )
            % Currently, the varargin is passed to either choose ('high' or 'low' quality)

            % ======================================================= %
            % ============ BASIC PROPERTIES OF THE ROBOT ============ %
            % ======================================================= %
            obj.Name      = 'iiwa14_2nd';
            obj.nq        = 7;
            obj.ParentID  = 0:obj.nq;
            obj.Dimension = 3;

            % Quality of the animation should be either `high` or `low`.
            obj.Dimension = 3;

            % ======================================================= %
            % ====== GEOMETRIC/INERTIA PROPERTIES OF THE ROBOT ====== %
            % ======================================================= %

            % Mass of the robot 1xnq
            obj.Masses = [ 3.4525, 3.4821, 4.05623, 3.4822, 2.1633, 2.3466, 3.129 ];

            % The inertia matrix of the robot, 6xnq
            % Ordered in Ixx, Iyy, Izz, Ixy, Ixz, Iyz            
            % Data from the open Github repository of 
            % Toyota Research Institute:
            % [REF] https://github.com/RobotLocomotion/drake/blob/master/manipulation/models/iiwa_description/urdf/iiwa14_spheres_dense_collision.urdf
            obj.Inertias = [ 0.0218, 0.0077, 0.0208 0.     , 0.  ,   -0.0039;
                             0.0208, 0.0078, 0.0218 0.0036 , 0.  ,    0.    ;
                             0.0320, 0.0097, 0.0304 0.     , 0.  ,    0.0062;
                             0.0218, 0.0078, 0.0207 0.     , 0.  ,    0.0036;
                             0.0129, 0.0057, 0.0111 0.     , 0.  ,    0.0039;
                             0.0065, 0.0045, 0.0063 0.     , 0.  ,    0.0003;
                             0.0146, 0.0147, 0.0029 0.0006 , 0.  ,    0.    ]';

            % ================================ %
            % ======= Joint Properties ======= %
            % ================================ %

            % All of the joints are revolute joints
            obj.JointTypes = ones( 1, obj.nq );

            % max/min of q array robot
            obj.q_max  =  [ 163; 113; 163; 115; 160; 110; 165 ] * pi/180;
            obj.q_min  = -obj.q_max;

            % max/min of dq array robot
            obj.dq_max =  [ 150; 150; 150; 150; 150; 150; 150 ] * pi/180;
            obj.dq_min = -obj.dq_max;

            % max/min of ddq array robot
            obj.ddq_max = 300 * ones( obj.nq, 1 ) * pi/180;
            obj.ddq_min = -obj.ddq_max;

            % The axis origin of the robot at initial configuration
            obj.AxisOrigins = [ 0. , 0. , 0.0000; 
                                0. , 0. , 0.2025;
                                0. , 0. , 0.407;
                                0. , 0. , 0.6225;
                                0. , 0. , 0.807;
                                0. , 0. , 1.0225;
                                0. , 0. , 1.1035]';                 

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
            obj.H_COM_init( 1:3, 4, : ) = obj.COMs;

        end


    end
end