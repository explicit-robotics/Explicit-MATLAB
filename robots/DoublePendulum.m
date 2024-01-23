classdef DoublePendulum < RobotPrimitive & handle
    % Constructs a planar 2-DOF Double Pendulum robot.
    % 
    % The links are assumed to have uniform mass densities. 
    % 
    % Parameters
    % ----------
    %     m1 : float
    %          The mass (kg) of the first link.
    %     m2 : float
    %          The mass (kg) of the second link.
    %     l1 : float
    %          The length (m) of the first link.
    %     l2 : float
    %          The length (m) of the second link.  
    % 
    % 
    % Error
    % -----
    %    `m1`, `m2`, `l1`, `l2` must be positive values

    properties
        m1 
        l1 
        m2 
        l2
    end
    
    methods
        function obj = DoublePendulum( m1, m2, l1, l2 )
            
            % ======================================================= %
            % ============ BASIC PROPERTIES OF THE ROBOT ============ %
            % ======================================================= %
            obj.Name      = 'DoublePendulum';
            obj.nq        = 2;
            obj.ParentID  = 0 : 1 : 2;
            obj.Dimension = 2;


            assert( m1 > 0 && m2 > 0 && l1 > 0 && l2 > 0 )
            obj.m1 = m1;
            obj.m2 = m2;
            obj.l1 = l1;
            obj.l2 = l2;
            
            % ======================================================= %
            % ====== GEOMETRIC/INERTIA PROPERTIES OF THE ROBOT ====== %
            % ======================================================= %
           
            % The geometrical and inertia property of the Robots
            obj.Masses = [ m1, m2 ];

            % Order is Ixx, Iyy, Izz, Ixy, Ixz, Iyz
            obj.Inertias = [ 0, 0, 1/12 * m1 * l1^2, 0, 0, 0;
                             0, 0, 1/12 * m2 * l2^2, 0, 0, 0]';

            % ======================================================= %
            % ============ JOINT PROPERTIES OF THE ROBOT ============ %
            % ======================================================= %          
            % All of the joints are revolute joints
            obj.JointTypes = ones( 1, obj.nq );

            % The initial position of the joints (q_i_arr) are 
            % y = [ 0, -l1 ]
            % All other values are zero 
            % We define q_i_arr which is the initial position
            % q_i_arr is sized as 3xnq
            % First, calculating the x positions of the matrix
            obj.AxisOrigins = [ 0,   0, 0;
                                0, -l1, 0]';
                                
            % For the snake robot, the axes are along +z, (0,0,1)
            obj.AxisDirections = repmat( [ 0; 0; 1 ], 1, obj.nq );


            % ======================================================= %
            % =========== INITIAL H MATRICES OF THE ROBOT =========== %
            % ======================================================= %    
            % Calculating the initial H array, including end-effector (EE)
            % Since we have included the EE, size is nq + 1
            % Construct a (4 x 4 x (nq+1)) matrix
            obj.H_init = repmat( eye( 4 ), [ 1, 1, obj.nq + 1 ] ); 

            % Fill-in the H_init matrix with 
            % x locations of the joint (including EE) is located at:
            obj.H_init( 2, 4, 2 ) = -l1;
            obj.H_init( 2, 4, 3 ) = -l1-l2;
                       
            % Get initial transformations of point on COM of each link
            % The COM of each link is located at the geometric center,
            % i.e., the half of the distance.
            % which is (l1/2, l1 + l2/2, ..., l1 + ... + l(n-1) + ln/2
            obj.H_COM_init  = repmat( eye( 4 ), [ 1, 1, obj.nq ] ); 
            obj.H_COM_init( 2, 4, 1 ) = -0.5*l1;
            obj.H_COM_init( 2, 4, 2 ) = -l1-0.5*l2;             
            
            % ======================================================= %
            % =========== GRAPHIC PROPERTIES OF THE ROBOT =========== %
            % ======================================================= %
            % Specifying the joint marker size
            obj.gMarkerSize = [ 24, 24 ]; 
            
            % For all robots, we need to specify 
            % (1) the base graphic design
            % (2) the end-effector graphic design 
            % For this robot, there is no base object
            obj.gBase = {};
            
            % The end-effector will be a marker with same size of others
            % Collect all the details as a cell
            obj.gEE   = { @scatter, obj.H_init( 1, 4, 3 ), obj.H_init( 2, 4, 3 ), 180, ...
                          'o', 'MarkerFaceColor', 'k', ...
                          'MarkerEdgeColor', 'k'  };
            
        end

    end
    
end

