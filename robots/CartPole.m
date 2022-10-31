classdef CartPole < RobotPrimitive & handle
    
    properties

        % ========================================== %
        % ======= Properties for CartPole ========== %
        % ========================================== %
        % For the 2DOF cartpole system, 
        % We simply save the masses of cart/pole, and the length of the pole
        
        mc      % Mass of the cart
        mp      % Mass of the Pole (Point-mass)
        lp      % Length of the Pole 
       

    end
    
    methods
        function obj = CartPole( ID, mc, lp, mp )

            % ======================================================= %
            % ============ BASIC PROPERTIES OF THE ROBOT ============ %
            % ======================================================= %
            obj.ID     = ID;
            obj.Name   = 'CartPole';
            obj.nq     = 2;

            obj.ParentID = [ 0, 1 ];
            
            % Robot simulation in 2D
            obj.Dimension = 2;
            
            % ======================================================= %
            % ====== GEOMETRIC/INERTIA PROPERTIES OF THE ROBOT ====== %
            % ======================================================= %
            % The mass, length properties of the robot
            obj.mc = mc;
            obj.mp = mp;
            obj.lp = lp;
            
            % The mass array of the robot
            obj.Masses = [ mc, mp ];
            
            % The generalized mass matrix
            obj.M_Mat = zeros( 6, 6, 2 );
            obj.M_Mat( :, :, 1 ) = diag( [ mc, mc, mc, 0, 0, 0 ] );
            obj.M_Mat( :, :, 2 ) = diag( [ mp, mp, mp, 0, 0, 0 ] );

            % ======================================================= %
            % ============ JOINT PROPERTIES OF THE ROBOT ============ %
            % ======================================================= %          
            % The first one is a prismatic joint 
            % The second one is a revolute joint
            obj.JointTypes = [ 2, 1 ];

            % The initial position of the joints 
            % For the cart/pole, it is at the same intial location
            obj.AxisOrigins = zeros( 3, 2 );
                                
            % Cart is along the +x axis 
            % Pole is along the +z axis
            obj.AxisDirections = [ 1, 0, 0;
                                   0, 0, 1]';


            % ======================================================= %
            % =========== INITIAL H MATRICES OF THE ROBOT =========== %
            % ======================================================= %    
            % Calculating the initial H array, including end-effector (EE)
            % Since we have included the EE, size is 3 (nq + 1)
            % Construct a (4 x 4 x 3) matrix
            obj.H_init = repmat( eye( 4 ), [ 1, 1, 3 ] ); 

            % The tip of the pole is at -lp, reside along the y axis
            % i.e., the (2,4)th coordinate value
            obj.H_init( 2, 4, 3 ) = -lp;
                       
            % Get initial transformations of point on COM of each link
            % The COM is concentrated at the tip of the pole
            obj.H_COM_init = repmat( eye( 4 ), [ 1, 1, 2 ] ); 
            obj.H_COM_init( 2, 4, 2 ) = -lp;
            
            % ======================================================= %
            % =========== GRAPHIC PROPERTIES OF THE ROBOT =========== %
            % ======================================================= %         
            % The joint is depicted as a marker 
            % Saving the size of the marker
            % The size of the cart and the revolute joint 
            % Attached to it
            obj.gMarkerSize = [ 24, 12 ];
            
            % Specifying the line width of the robot
            obj.gLineWidth  = 3;            
            
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

