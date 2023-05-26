classdef RobotPrimitive < handle
    % A primitive robot parent class
    % This class is a "template" to construct a robot.
    %
    % ================================ %
    % ====== Naming Conventions ====== %
    % ================================ %
    % For Class Attributes with basic full names, use "CamelCase" Naming convention
    % e.g., Name, ParentiD, Dimension
    %
    % For Class Attributes with mathematical notations, use "snake-case" Naming convention
    % This is necessary to disambiguate the name.
    % For instance, if we want to define the initial values of the
    % Homogenous Matrix H, making the name as "HInit" is dislikeable.
    % Rather than "CamelCase", we use "H_init".
    %
    % ================================ %
    % ======= Array Conventions ====== %
    % ================================ %
    % For all arrays, the values are stacked horizontally, 
    % i.e., stacked columnwise.
    % Consider m arrays with n size, then, the resulting matrix is m x n
    % For vector, both column/row work.
    
    
    % For Class Methods, use "CamelCase" Naming convention

    properties
        % ================================ %
        % ======= Basic Properties ======= %
        % ================================ %
        Name
        ParentID

        % ================================ %
        % ======= Joint Properties ======= %
        % ================================ %

        nq                      % The number of the robot joints
        JointTypes              % 1 is revolute, 2 is prismatic

        AxisOrigins             % The 3D position of on joint axis
        AxisDirections          % The 3D direction of joint axis
                                % w for revolute joint, v for prismatic joint
        JointTwists             % (6 x nq) Joint twist array at initial configuration
        % Joint Twist Array consists of
        % [1] [-w x p, w] for Revolute  Joint
        % [2] [     v, w] for Prismatic Joint
        % p is the point on the joint axis

        q                       % The current q arrays
        q_max                   % Upper limit of   q (position) values
        q_min                   % Lower limit of   q (position) values
        dq_max                  % Upper limit of  dq (velocity) values
        dq_min                  % Lower limit of  dq (velocity) values
        ddq_max                 % Upper limit of ddq (accelertion) values
        ddq_min                 % Lower limit of ddq (accelertion) values

        % =================================== %
        % ======== The H Matrix (SE3) ======= %
        % =================================== %

        H_init                  % The (4 x 4 x nq+1) H matrix of at initial config.
                                % (nq+1), the final one contains the end-effector
        H_COM_init              % The (4 x 4 x nq) H matrix of COMs at initial config.
        H_ij                    % The (4 x 4 x nq) H matrix between consecutive segments
        H_base = eye( 4 )       % The initial base H matrix, usually an eye( 4 ) matrix


        % ================================ %
        % ===== Inertial Properties ====== %
        % ================================ %

        Masses                  % (1 x nq) array
        Inertias                % (6 x nq) array, Ixx, Iyy, Izz, Ixy, Ixz, Iyz
        M_Mat                   % The (6 x 6) generalized inertia matrix.
        M_Mat2                  % The (6nq x 6nq) matrix, which is simply
                                % the diagonal collection of M_Mat
                                % Eq. 8.61 of Modern Robotics

        % ================================ %
        % ===== Additional Matrices ====== %
        % ================================ %
        A_Mat1                  % (6   x nq) array, stacking it horizontally.
        A_Mat2                  % (6nq x nq) array, Eq. 8.60 of Modern Robotics.
                                % A full-array respresntation of A_Mat1
                    

        % =================================== %
        % ====== Animation Properties ======= %
        % =================================== %
       
        
        gBase = {};
        gEE   = {};
        gObjs = {};
        
        % Marker size of the joint
        gMarkerSize

        % Dimension of the animation
        Dimension

        % Quality of the animation
        % This property is only used for 3D robots
        Quality

        % =================================== %
        % ======== Other Properties ========= %
        % =================================== %
        % Gravitational Acceleration
        Grav = 9.81             
    end

    methods

        function args = parseArgument( obj, varargin )
            % An internal method for parsing the argument
            % The arguments that we want to parse is as follows
            % [1] bodyID (integer)
            %     The index of body which is ranged from 1 ~ obj.nq
            %
            % [2] position ( (1 x 3) or (3 x 1) array ) or ("COM")
            %     The position of the point w.r.t. the body's joint frame
            %     "COM" is a special character to get the position of the
            %     "COM" with respect to the body frame that is
            %     attached to the bodyID-th joint.

            % Structure initialization
            args = struct( 'bodyID', [ ], 'position', [ ] );

            % Parsing the bodyID
            idx = find( strcmp( 'bodyID', varargin ) );

            % If bodyID is passed, take its value pair
            if ~isempty( idx )

                % The value pair of bodyID.
                bodyID = varargin{ idx + 1 };

                % bodyID must be ranged between 1 <= bodyID <= nq
                assert( 1 <= bodyID && bodyID <= obj.nq )

                % If set correctly, then save the value.
                args.bodyID = bodyID;
            end

            % Parsing the position
            idx = find( strcmp( 'position', varargin ) );

            % If position is passed, take its value pair
            if ~isempty( idx )
                pos = varargin{ idx + 1 };

                % position must be either "COM" or 3D vector
                assert( strcmp( 'COM', pos ) || isequal( size( pos ), [ 3, 1 ] ) || isequal( size( pos ), [ 1, 3 ] ) );

                % If the position is a 3D row vector, then change it to a column vector.
                if( isequal( size( pos ), [ 1,3 ] ) )
                    pos = pos.';
                end
                
                % If all correct, save the position
                args.position = pos;
            end

            % Assert that either "position" or "bodyID" argument should NOT be empty.
            assert( ~isempty( args.bodyID ) || ~isempty( args.position ) );

        end

        function init( obj )
            % Initialization of the robot
            % A separate init function is required, since the joint-twists
            % and generalized mass matrix must be calculated AFTER
            % the subclass object is constructed.
            obj.setJointTwists( )
            obj.setGeneralizedMassMatrix( )

        end

        function setGeneralizedMassMatrix( obj )
            % Set the generalized Mass Matrix
            % The size of Masses and Inertias must be as follows:
            assert( isequal( size( obj.Masses    ), [ 1, obj.nq ] ) || isequal( size( obj.Masses ), [ obj.nq, 1 ] ) );
            assert( isequal( size( obj.Inertias  ), [ 6, obj.nq ] ) );

            % The generalized mass matrix, two types:
            obj.M_Mat  = zeros( 6, 6, obj.nq );
            obj.M_Mat2 = zeros( 6 * obj.nq, 6 * obj.nq );

            % Set the Generalized Mass Matrix
            for i = 1 : obj.nq
                m = obj.Masses( i );

                Ixx = obj.Inertias( 1, i );
                Iyy = obj.Inertias( 2, i );
                Izz = obj.Inertias( 3, i );
                Ixy = obj.Inertias( 4, i );
                Ixz = obj.Inertias( 5, i );
                Iyz = obj.Inertias( 6, i );                

                I_mat = [ Ixx, Ixy, Ixz; ...
                          Ixy, Iyy, Iyz; ...
                          Ixz, Iyz, Izz];

                % Positioning M_Mat, as a 3D array
                obj.M_Mat( :, :, i ) = [ diag( m * ones( 1, 3 ) ), zeros( 3 ); ...
                                                       zeros( 3 ),    I_mat ];
                
                % Positioning M_Mat diagonally.
                obj.M_Mat2( 6*(i-1)+1:6*i, 6*(i-1)+1:6*i ) = obj.M_Mat( :, :, i );

            end


        end

        function setJointTwists( obj )
            % Setting up the Joint Twists of the robot
            % In the main document, the eta arrays are created
            % with this method.

            % Before running this method,
            % AxisOrigins, AxisDirections should not be empty 
            % And nq (DOF of the robot) should be positive
            assert( ~isempty( obj.AxisOrigins    ) )
            assert( ~isempty( obj.AxisDirections ) )
            assert( ~isempty( obj.nq ) && obj.nq >= 1 )

            % Initialize the (6 x nq) joint twist array
            obj.JointTwists = zeros( 6, obj.nq );

            % Iterate through the joint types and generate the joint twists
            for i = 1 : obj.nq

                % We need to check whether the joint is a rev (1) or a
                % prism. (2) by calling the "JointTypes" attribute.
                idx = obj.JointTypes( i );

                % If neither 1 nor 2 is given for the joint types, halt.
                assert( idx == 1 || idx == 2 )

                % If the joint is rev. (i.e., idx == 1)
                if idx == 1
                    w_i = obj.AxisDirections( :, i );
                    obj.JointTwists( 1:3, i ) = -vec_to_so3( w_i ) * obj.AxisOrigins( :, i );
                    obj.JointTwists( 4:6, i ) = w_i;

                % If the joint is prism. (i.e., idx == 2)
                else
                    obj.JointTwists( 1:3, i ) = obj.AxisDirections( :, i );
                    obj.JointTwists( 4:6, i ) = [ 0, 0, 0 ];
                end

            end

            % Once the joint twists are defined, define the A matrix
            % Each A matrix is Ai, which is defined by Section 8.3.1.
            % in Modern Robotics (2017), Frank Park and Kevin Lynch.
            % This is a constant matrix, hence saving it separately
            obj.A_Mat1 = zeros(        6, obj.nq );            
            obj.A_Mat2 = zeros( 6*obj.nq, obj.nq );            

            for i = 1:obj.nq
               Ai = H_to_invAdj( obj.H_COM_init( :, :, i ) ) * obj.JointTwists( :, i );
               obj.A_Mat1( :, i ) = Ai;
               obj.A_Mat2( 6*(i-1)+1:6*i, i ) = Ai;
            end

        end


        function H_FK = getForwardKinematics( obj, q_arr, varargin )
            % Return the forward kinematics (FK) of the robot with the q array
            % By default, it returns the end-effector (EE) FK
            % We can specify other points than the EE for the FK
            % via "varargin"
            % The units of the q_arr is 
            % [1] radian for revolute  joint ( i.e., joint_type == 1 )
            % [2] meters for prismatic joint ( i.e., joint_type == 2 )

            assert( obj.nq >= 1 )

            % Check if varargin is empty or not
            % If empty, then calculate the FK of EE

            if isempty( varargin )
                n = obj.nq;

                % The final matrix of H_init is the EE's H Matrix
                H = obj.H_init( :, :, end );

            else
                % Parse the argument cell array passed
                args = obj.parseArgument( varargin{ : } );

                % Get the bodyID
                n = args.bodyID;

                % If the position is empty, then set H as the joint
                % of the bodyID-th joint position
                if isempty( args.position )
                    H = obj.H_init( :, :, n );

                % If not empty
                else
                    % If special character "COM" is passed, then
                    % Set the H Matrix as the COM of the bodyID-th segment.
                    if strcmp( args.position, "COM" )
                        H = obj.H_COM_init( :, :, n );

                    else
                        % Add the position array
                        H = obj.H_init( :, :, n );
                        H( 1:3, 4 ) = H( 1:3, 4 ) + args.position;
                    end
                end

            end

            % Conduct mutliplication of T01 T12 T23 ... T(n-1)n
            % Initialization of the exponential product matrix   
            exp_T_arr = zeros( 4, 4, n );
            
            % Wrapper to use symbolic form.
            if isa( q_arr , 'sym' )
                exp_T_arr = sym( exp_T_arr );
            end

            for i = 1:n
                exp_T_arr( : , :, i ) = expSE3( obj.JointTwists( :, i ), q_arr( i ) );
            end

            % Multiply the exponential product with the initial H matrix
            H_FK = getExpProd( exp_T_arr ) * H;

        end


        function JS = getSpatialJacobian( obj, q_arr, varargin )
            % Return the Spatial Jacobian (JS) of the robot with the q_deg array
            % By default, it returns the end-effector (EE) FK
            % We can specify other points than the EE for the FK
            % via "varargin" [IN DEVELOPMENT] [2022.08.30]

            assert( obj.nq >= 1 )

            % Check if varargin is empty or not
            % If empty, then calculate the Spatial Jacobian of EE
            if isempty( varargin )
                n = obj.nq;

            else
                % If there is varargin passed, get the bodyID
                % and ignore other variables passed
                args = obj.parseArgument( varargin{ : } );
                n = args.bodyID;
            end

            % Initialization of the Spatial Jacobian Matrix
            % [Advanced Users]
            % In case if the q_arr is symbolic, then 
            % Covering the matrix with             
            JS = zeros( 6, obj.nq );
            exp_T_arr = repmat( eye( 4 ), [ 1, 1, obj.nq ] );
            
            % If symbolic, wrap the variable as symbolic.
            if isa( q_arr , 'sym' )
                JS        = sym( JS );
                exp_T_arr = sym( exp_T_arr );
            end    

            for i = 2:n
               exp_T_arr( : , :, i ) = expSE3( obj.JointTwists( :, i - 1 ), q_arr( i - 1 ) );
            end
            
            for i = 1:n
                % Set the Spatial Jacobian and save the exponential product
                exp_prod = getExpProd( exp_T_arr( :, :, 1 : i ) ); 
                JS( :, i ) = H_to_Adj( exp_prod ) * obj.JointTwists( :, i );
            end


        end

        function JH = getHybridJacobian( obj, q_arr, varargin )
            % Calculating the Hybrid Jacobian (JH) of the end-effector (EE)
            % By default, it returns the end-effector (EE) JH
            % We can specify other points than the EE for the JH calculation
            % via "varargin" [IN DEVELOPMENT] [2022.08.30]

            % Get the Spatial Jacobian
            % varargin{ ":" } is necessary, not varargin itself
            JS = obj.getSpatialJacobian( q_arr, varargin{ : } );

            % Get the Forward Kinematics of the point in interest
            H = obj.getForwardKinematics( q_arr, varargin{ : } );

            % Refer to the tutorial paper
            A = eye( 6 );

            % If q_arr is sym
            if isa( q_arr, 'sym')
                A = sym( A );
            end

            p_tilde = vec_to_so3( H( 1:3, 4 ) );
            A( 1:3,4:6 ) = -p_tilde;

            JH = A * JS;

        end

        function JB = getBodyJacobian( obj, q_arr, bodyID, varargin )
            % Get the Body Jacobian (JB) of the frame with H_init
            % Not only we need a q input, but we also need an H matrix
            % of the frame to calculate its Body Jacobian.
            % For that, we need to pass the bodyID integer and position of
            % the point w.r.t. to it's joint H matrix

            assert( bodyID <= obj.nq )

            if isempty( varargin )
                % Set the point in interest as the joint location of bodyID
                H = obj.getForwardKinematics( q_arr, 'bodyID', bodyID );

            elseif ( length( varargin ) == 1 && strcmp( varargin{ 1 }, "COM" ) )
                % Set the point of interest as the COM of bodyID
                H = obj.getForwardKinematics( q_arr, 'bodyID', bodyID, 'position', "COM" );

            else
                % [TODO] For More Advanced Users
                H = obj.getForwardKinematics( q_arr, 'bodyID', bodyID, varargin{ : } );
            end

            JS = obj.getSpatialJacobian( q_arr, 'bodyID', bodyID );
            JB = H_to_invAdj( H ) * JS;

        end

        
        function M = getMassMatrix( obj, q_arr )

           % Get the L_matrix
           [ L_Mat, ~ ] = LW_Mat( obj.A_Mat1, obj.H_COM_init, q_arr );
           M = obj.A_Mat2.' * L_Mat.' * obj.M_Mat2 * L_Mat * obj.A_Mat2;
            
        end        

        
        function G = getGravityVector( obj, q_arr )

           % Get the L_matrix
           [L_Mat, ~] = LW_Mat( obj.A_Mat1, obj.H_COM_init, q_arr );

           % Defining dVbase, from Section 8.3.2 of Modern Robotics
           dVbase = zeros( 6 * obj.nq );
           A1     = obj.A_Mat1( 1:6, 1 );               
           tmp    = H_to_Adj( expSE3( -A1, q_arr( 1 ) ) * ( obj.H_COM_init( :, :, 1 ) )^-1 );
           
           % For 2D case, gravity is along y direction
           if obj.Dimension == 2
              tmp_grav = [ 0; obj.Grav; 0; 0; 0; 0 ];
           else
              tmp_grav = [ 0; 0; obj.Grav; 0; 0; 0 ];
           end
           
           dVbase( 1: 6 ) = tmp * tmp_grav;
           G = obj.A_Mat2.' * L_Mat.' * obj.M_Mat2 * L_Mat * dVbase;
           G = G( :, 1 );

        end        
       
        
        function C_mat = getCoriolisMatrix( obj, q_arr, dq_arr )
            
           % Get the L and W matrices
           [L_Mat, W_Mat ] = LW_Mat( obj.A_Mat1, obj.H_COM_init, q_arr );

           Aa_mat  = zeros( 6 * obj.nq, 6 * obj.nq );
           Aaa_mat = zeros( 6 * obj.nq, 6 * obj.nq );
           
           % If q_arr or dq_arr are symbolic arguments, wrap the function as sym
           if ( isa( q_arr, 'sym' ) || isa( dq_arr, 'sym' ) )
               Aa_mat  = sym( Aa_mat  );
               Aaa_mat = sym( Aaa_mat );
           end
           
           for i = 1 : obj.nq
               
               % Equation 8.63
               Ai = obj.A_Mat1( :, i );
               Aa_mat( 6*(i-1)+1:6*i,6*(i-1)+1:6*i ) = vec_to_adj( Ai * dq_arr( i ) );
               
               % Equation 8.62
               Vi = obj.getBodyJacobian( q_arr, i, "COM" ) * dq_arr;            
               Aaa_mat( 6*(i-1)+1:6*i,6*(i-1)+1:6*i ) = vec_to_adj( Vi );               
           end
           
           C_mat = - obj.A_Mat2.' * L_Mat.' * ( obj.M_Mat2* L_Mat * Aa_mat * W_Mat + Aaa_mat.' * obj.M_Mat2 ) * L_Mat * obj.A_Mat2;
          
        end        
        
        function updateKinematics( obj, q_arr )
            % Updating the kinematics of the robot, with the q_deg array
            % The updates will calculate the Hij matrices,
            % which are the homogeneous transformation between
            % consecutive links

            % Initialize H_ij matrices
            obj.H_ij = repmat( eye( 4 ), [ 1, 1, obj.nq ]);

            for i = 1:obj.nq

                % Get the 6D joint twist axis
                joint_twist = obj.JointTwists( :, i );

                % Get the rotation matrix
                H_ini = eye( 4 );
                H_ini( 1:3, 1:3 ) = obj.H_init( 1:3, 1:3, i );

                % The two consecutive transformation
                obj.H_ij( :, :, i ) = expSE3( joint_twist, q_arr( i ) ) * H_ini;
            end

            % Save the current q array to robot object
            % Note that only the "updateKinematics" method
            % Save the q_arr internally.
            obj.q = q_arr;

        end

        function switchJoint( obj, bodyID, idx_type, axis_direction )

            % The bodyID should be between 1 to nq
            assert( bodyID >= 1 && bodyID <= obj.nq )

            % Joint type should either 1 (rev.) or 2 (prism.)
            assert( idx_type == 1 || idx_type == 2 )

            % The axis direction should be a 3D array
            assert( isequal( size( axis_direction ), [ 3, 1 ] ) || isequal( size( axis_direction ), [ 1, 3 ] ) )

            
            % Normalize the axis_direction array
            % Before that, we need assert that the axis_direction array
            % is not a zero vector
            norm = sqrt( sum( axis_direction.^2 ) );
            assert( norm ~= 0 )

            axis_direction = axis_direction / norm;

            % Re-setting the values
            obj.JointTypes( bodyID ) = idx_type;
            obj.AxisDirections( :, bodyID ) = axis_direction;

            % Robot re-initialization
            obj.init( )
        end

        function robot_new = addKinematics( obj, robot2add, varargin )
            
            % Before Adding, the robot dimension should match each other
            assert( obj.Dimension == robot2add.Dimension );
            
            
            % Extend the robot!
            % For the extension, we create a new robot
            robot_new = RobotPrimitive( );
            robot_new.Dimension = obj.Dimension;
            
            % ================================ %
            % ======= Basic Properties ======= %
            % ================================ %
            robot_new.nq = obj.nq + robot2add.nq;

            % Reproducing the parent ID
            robot_new.ParentID = 0 : robot_new.nq;

            % ================================ %
            % ======= Joint Properties ======= %
            % ================================ %
            % Defining the joint properties of the new robot
            % Extending the joint types
            % Make sure that the JointTypes are row-vector            
            assert( size( obj.JointTypes, 1 ) == 1 && size( robot2add.JointTypes, 1 ) == 1  );
            robot_new.JointTypes = horzcat( obj.JointTypes , robot2add.JointTypes  );

            % Extending the axis origins and directions
            % For that, we get the end-effector position
            pEE = obj.H_init( 1 : 3 , 4, end );
            
            % The row-length should match before horzcat
            assert( size( obj.AxisOrigins   , 1 ) == 3 && size( robot2add.AxisOrigins   , 1 ) == 3  );
            assert( size( obj.AxisDirections, 1 ) == 3 && size( robot2add.AxisDirections, 1 ) == 3  );
            
            robot_new.AxisOrigins    = horzcat( obj.AxisOrigins   , pEE + robot2add.AxisOrigins    );
            robot_new.AxisDirections = horzcat( obj.AxisDirections,       robot2add.AxisDirections );
            
            % =================================== %
            % ======== The H Matrix (SE3) ======= %
            % =================================== %            
            % Setting the H_init of the COM and the robot's joints
            % ----------------------------------------Excluding EE -----
            robot_new.H_init     = cat( 3, obj.H_init( :, :, 1:end - 1 ), robot2add.H_init     );
            robot_new.H_COM_init = cat( 3, obj.H_COM_init               , robot2add.H_COM_init );

            % Adding pEE for robot2's H_init
            robot_new.H_init(     1 : 3, 4, obj.nq + 1: end ) = pEE + robot_new.H_init(     1 : 3, 4, obj.nq + 1: end );
            robot_new.H_COM_init( 1 : 3, 4, obj.nq + 1: end ) = pEE + robot_new.H_COM_init( 1 : 3, 4, obj.nq + 1: end );
                
            % ================================ %
            % ===== Inertial Properties ====== %
            % ================================ %
            assert( size( obj.Masses  , 1 ) == 1 && size( robot2add.Masses  , 1 ) == 1 );
            assert( size( obj.Inertias, 1 ) == 6 && size( robot2add.Inertias, 1 ) == 6 );
            
            robot_new.Masses   = horzcat(  obj.Masses,   robot2add.Masses   );
            robot_new.Inertias = horzcat(  obj.Inertias, robot2add.Inertias );
            robot_new.M_Mat    =     cat( 3, obj.M_Mat, robot2add.M_Mat    );
            
            % =================================== %
            % ====== Animation Properties ======= %
            % =================================== %
            
            % Extending the MarkerSize
            robot_new.gMarkerSize = horzcat( obj.gMarkerSize, robot2add.gMarkerSize );

            % Rewriting the end-effector as the provided one
            % Reset the x y positions of the end effector
            gEE_new = robot2add.gEE;

            % Change the x (2) and y (3) location 
            if robot_new.Dimension == 2 
                gEE_new{ 2 } = robot_new.H_init( 1, 4, end );
                gEE_new{ 3 } = robot_new.H_init( 2, 4, end );
            else
                % 3D Case is not yet implemented
            end
            
            robot_new.gEE = gEE_new;
            
            % Give a new name
            if isempty( varargin )
                robot_new.Name = 'myNewRobot';

            else
                % Parsing the robot name
                idx = find( strcmp( 'name', varargin ) );

                % If there is a value, then set it
                if ~isempty( idx )
                    name = varargin{ idx + 1 };
                end
                robot_new.Name = name;
            end


        end

    end

end

