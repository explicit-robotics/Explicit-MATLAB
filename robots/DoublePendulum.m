classdef DoublePendulum < RobotPrimitive & handle
    
    properties

    end
    
    methods
        function obj = DoublePendulum( )

            % ======================================================= %
            % ============ BASIC PROPERTIES OF THE ROBOT ============ %
            % ======================================================= %
            obj.Name   = 'DoublePendulum';
            obj.nq     = 2;

            obj.ParentID = 0 : 1 : 2;
            
            % Robot simulation in 2D
            obj.Dimension = 2;
            
            % ======================================================= %
            % ====== GEOMETRIC/INERTIA PROPERTIES OF THE ROBOT ====== %
            % ======================================================= %
            % The mass, length properties of the robot
            m = 1;
            l = 1;
            I = 1/12 * m * l^2;

            % The geometrical and inertia property of the Robots
            obj.Masses      = m * ones( 1, obj.nq );
            obj.Inertias    = repmat( I * eye( 3 ), [ 1, 1, obj.nq ] );

            % The ineria tensor along the principal axis
            obj.M_Mat = repmat( diag( [ m, m, m, I, I, I ] ) , [ 1, 1, obj.nq ]  );

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
            obj.AxisOrigins = [ 0,  0, 0;
                                0, -l, 0]';
                                
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
            obj.H_init( 2, 4, 2 ) = -1 * l;
            obj.H_init( 2, 4, 3 ) = -2 * l;
                       
            % Get initial transformations of point on COM of each link
            % The COM of each link is located at the geometric center,
            % i.e., the half of the distance.
            % which is (l1/2, l1 + l2/2, ..., l1 + ... + l(n-1) + ln/2
            obj.H_COM_init  = repmat( eye( 4 ), [ 1, 1, obj.nq ] ); 
            obj.H_COM_init( 2, 4, 1 ) = -0.5 * l;
            obj.H_COM_init( 2, 4, 2 ) = -1.5 * l;             
            
            % ======================================================= %
            % =========== GRAPHIC PROPERTIES OF THE ROBOT =========== %
            % ======================================================= %
            % Specifying the joint marker size
            obj.gMarkerSize = [ 12, 12 ]; 
            
            % For all robots, we need to specify 
            % (1) the base graphic design
            % (2) the end-effector graphic design 
            % For this robot, there is no base object
            obj.gBase = [ ];
            
            % The end-effector will be a marker with same size of others
            % Collect all the details as a cell
            obj.gEE   = { @plot, obj.H_init( 1, 4, 3 ), obj.H_init( 2, 4, 3 ), ...
                          'o', 'MarkerFaceColor', 'k', ...
                          'MarkerEdgeColor', 'k', 'MarkerSize', 12  };
            
        end

    end
    
end

