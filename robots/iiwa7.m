classdef iiwa7 < RobotPrimitive & handle
    
    properties

        % ========================================== %
        % ===== Properties for KUKA LBR iiwa7 ====== %
        % ========================================== %


    end
    
    methods
        function obj = iiwa7( )

            % ======================================================= %
            % ============ BASIC PROPERTIES OF THE ROBOT ============ %
            % ======================================================= %            
            obj.Name   = 'iiwa7';
            obj.nq     = 7;

            % All of the joints are revolute joints
            obj.JointTypes = ones( obj.nq, 1 );            

            % Robot simulation in 3D
            obj.Dimension = 3;

            % Parents of the object
            obj.Parents = 0:obj.nq;            

            % Qmax, Qmin of the robot 
            obj.QMax  =  func_deg2rad( [ 170; 120; 170; 120; 170; 120; 175 ] );
            obj.QMin  = -obj.QMax;

            % dQMax, dQMin of the robot
            obj.dQMax =  func_deg2rad( [ 100; 110; 100; 130; 130; 180; 180 ] );
            obj.dQMin = -obj.dQMax;
    
            % ddQMax, ddQMin of the robot            
            obj.ddQMax = func_deg2rad( 300) * ones( obj.nq, 1 );
            obj.ddQMin = -obj.ddQMax;

            % The starting configuration of the robot 
            obj.QStart = [ 0 , 45 , 0 , -70 , 0, 70, 10 ]';        

            % Color Flag of the robot
            obj.ColorFlag = 1;

            % Mass of the robot 1xnq
            obj.Mass = [ 2.7426, 4.9464, 2.5451, 4.6376, 1.7140, 2.4272, 0.4219 ];

            % COM  of the robot nqx3 
            obj.COM  = [       0, -18.7e-3, 101.6e-3;
                        -0.21e-3,  25.0e-3,  82.5e-3;
                        -0.20e-3,  19.5e-3,  98.4e-3;
                        -0.21e-3, -20.1e-3,    86e-3;
                        -0.04e-3, -13.5e-3,    66e-3;
                        -0.35e-3,  51.4e-3,  17.1e-3;
                        -0.01e-3,   0.1e-3,    11e-3]';

            % The inertia matrix of the robot, nqx3
            obj.Inertia = [  0.24,  0.024, 0.0128; 
                           0.0468, 0.0282, 0.0101;
                             0.02,   0.02, 0.0600;
                             0.04,  0.027, 0.0100;
                            0.019,  0.016, 0.0120;
                            0.007,  0.006, 0.0050;
                           0.0003, 0.0003, 0.0005 ]';

            % The axis origin of the robot at initial configuration 
            obj.AxisOrigin = [ 0,      0, 152.5e-3;
                               0, -11e-3, 187.5e-3; 
                               0,  11e-3, 212.5e-3;
                               0,  11e-3, 187.5e-3;
                               0, -11e-3, 212.5e-3;
                               0, -62e-3, 187.5e-3;
                               0,  62e-3,  79.6e-3]';
            
            % We conduct a cumsum to get the Axis Origin
            obj.AxisOrigin = cumsum( obj.AxisOrigin, 2 );

            % Axis Direction
            obj.AxisDirection = [ 0,  0, 1;
                                  0,  1, 0; 
                                  0,  0, 1;
                                  0, -1, 0;
                                  0,  0, 1;
                                  0,  1, 0;
                                  0,  0, 1]';                                 

            % End-effector origin
            obj.AxisOriginFlange = [ 0, 0 , 31.4e-3]; 

            for i = 1:obj.nq
                obj.M_Mat(   :,   :, i ) = zeros( 6, 6 );
                obj.M_Mat( 1:3, 1:3, i ) = obj.Mass( i  )   * eye( 3 );
                obj.M_Mat( 4:6, 4:6, i ) = obj.Inertia( i ) * eye( 3 );
            end

        end
        
        function init( obj )
    
            nq = obj.nq;

            % Setting up the joint-twist (eta) of the robot
            % This will fill-in the JointTwist Property
            obj.setJointTwist( )
                            
            % get initial transformation of point on each joint
            obj.H_init = repmat( eye( 4 ), [ 1, 1, nq + 1 ] );
            obj.H_init( 1:3, 4, 1:nq ) = obj.AxisOrigin;

            % The end-effector of the robot
            obj.H_init( 1:3, 4, end ) = obj.AxisOrigin( :, end ) + obj.AxisOriginFlange';
            
            % get initial transformations of point on COM of each link
            obj.H_COM_init = repmat( eye( 4 ), [ 1, 1, nq ] );
            obj.H_COM_init( 1:3, 4, : ) = obj.AxisOrigin + obj.COM;
           
            
        end

    end
end