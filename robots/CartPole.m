classdef CartPole < RobotPrimitive & handle
    % Constructs a planar 2-DOF CartPole robot.
    % 
    % Parameters
    % ----------
    %     mc : float
    %          The mass (kg) of the cart.
    %     mp : float
    %          The mass (kg) of the point mass suspended at the pole.
    %     lp : float
    %          The length (m) of the pole.
    % 
    % Error
    % -----
    %    `mc`, `mp`, `lp` must be positive values

    properties
        
        mc      
        mp      
        lp      
       
    end
    
    methods
        function obj = CartPole( mc, mp, lp )

            % ======================================================= %
            % ============ BASIC PROPERTIES OF THE ROBOT ============ %
            % ======================================================= %
            obj.Name   = 'CartPole';   
            obj.nq     = 2;
            obj.ParentID  = 0 : 1 : obj.nq;
            obj.Dimension = 2;
            
            assert( mc > 0 && mp > 0 && lp > 0 )
            obj.mc = mc;
            obj.mp = mp;
            obj.lp = lp;

            % ======================================================= %
            % ====== GEOMETRIC/INERTIA PROPERTIES OF THE ROBOT ====== %
            % ======================================================= %    
                       
            % The masses of the cart-pole robot
            obj.Masses = [ mc, mp ];   
            
            % Order is Ixx, Iyy, Izz, Ixy, Ixz, Iyz
            obj.Inertias = zeros( 6, 2 );

            % ======================================================= %
            % ============ JOINT PROPERTIES OF THE ROBOT ============ %
            % ======================================================= %          
            % Revolute (1) and Prismatic (2)
            obj.JointTypes = [ 2, 1 ];

            % The position of the point of the joint axis. 
            % Expressed in {S} frame.
            obj.AxisOrigins = zeros( 3, 2 );
                                
            % Prismatic joint is along the +x axis 
            % Revolute  joint is along the +z axis
            obj.AxisDirections = [ 1, 0, 0;
                                   0, 0, 1]';


            % ======================================================= %
            % =========== INITIAL H MATRICES OF THE ROBOT =========== %
            % ======================================================= %    
            % Calculating the initial H array, including end-effector (EE)
            % Since we have included the EE, size is nq + 1
            % Construct a (4 x 4 x 3) matrix
            obj.H_init = repmat( eye( 4 ), [ 1, 1, obj.nq+1 ] ); 

            % The tip of the pole is at -lp, reside along the y axis
            % i.e., the (2,4)th coordinate value
            obj.H_init( 2, 4, 3 ) = -lp;
                       
            % Get initial transformations of point on COM of each link
            % The COM is concentrated at the tip of the pole
            obj.H_COM_init = repmat( eye( 4 ), [ 1, 1, obj.nq ] ); 
            obj.H_COM_init( 2, 4, 2 ) = -lp;
            
            % ======================================================= %
            % =========== GRAPHIC PROPERTIES OF THE ROBOT =========== %
            % ======================================================= %         
            % The joint is depicted as a marker 
            % Saving the size of the marker
            % The size of the cart and the revolute joint 
            % Attached to it
            obj.gMarkerSize = [ 24, 12 ];
      
            
            % For all robots, we need to specify 
            % (1) the base graphic design
            % (2) the end-effector   design 
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

