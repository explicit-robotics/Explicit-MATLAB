classdef RobotPrimitive < handle
    % A primitive robot parent class
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
        Inertias                % (3 x nq) array, along principal axis.
        M_Mat                   % The (6 x 6) generalized inertia matrix.

        % =================================== %
        % ====== Animation Properties ======= %
        % =================================== %
       
        % Graphic Objects for the Animation
        % prefix "g" is used to refer "g"raphic objects
        gBase = {};
        gEE   = {};
        gObjs = {};
        
        % Marker size of the joint
        gMarkerSize

        % Dimension of the animation
        Dimension

        % Quality of the animation
        Quality

        % Color flag to assign FaceColor by hand
        Color

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

            % If there is a value, then take its value pair
            if ~isempty( idx )

                % Getting its pair.
                bodyID = varargin{ idx + 1 };

                % BodyID must be ranged between 1 <= bodyID <= nq
                assert( 1 <= bodyID && bodyID <= obj.nq )

                % If set correctly, then save the value.
                args.bodyID = bodyID;
            end

            % Parsing the position
            idx = find( strcmp( 'position', varargin ) );


            if ~isempty( idx )
                pos = varargin{ idx + 1 };

                % position must be either "COM" or 3D vector
                assert( strcmp( 'COM', pos ) || isequal( size( pos ), [ 3, 1 ] ) );

                % If all correct, save the position
                args.position = pos;
            end

            % Assert that either argument should NOT be empty.
            assert( ~isempty( args.bodyID ) || ~isempty( args.position ) );

        end


        function init( obj )
            % Initialization of the robot
            % [TODO] [2022.09.02] [Moses C. Nah]
            % Currently, we only have setJointTwists in this
            % init( ) function, but for the near future we will include
            % other functions.
            obj.setJointTwists( )

        end

        function setJointTwists( obj )
            % Setting up the Joint Twists of the robot
            % In the main document, the eta arrays are created
            % with this method.

            % Before running this method,
            % AxisOrigins, AxisDirections should not be empty 
            % And nq (DOF of the robot) should be positive
            assert( ~isempty( obj.AxisOrigins ) )
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
                    obj.JointTwists( 1:3, i ) = -func_skewSym( w_i ) * obj.AxisOrigins( :, i );
                    obj.JointTwists( 4:6, i ) = w_i;

                % If the joint is prism. (i.e., idx == 2)
                else
                    obj.JointTwists( 1:3, i ) = obj.AxisDirections( :, i );
                    obj.JointTwists( 4:6, i ) = [ 0, 0, 0 ];
                end

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
                    % Set the H Matrix as the COM of the bodyID-th
                    % segment.
                    if strcmp( args.position, "COM" )
                        H = obj.H_COM_init( :, :, n );

                    else
                        % If not, add the position from the joint's H matrix
                        % Position must be a (3 x 1) array
                        assert( isequal( size( args.position ), [ 3, 1 ] ) )

                        % Add the position array
                        H = obj.H_init( :, :, n );
                        H( 1:3, 4 ) = H( 1:3, 4 ) + args.position;
                    end
                end

            end

            % Conduct mutliplication of T01 T12 T23 ... T(n-1)n
            % Initialization of the exponential product matrix    
            exp_T_arr = zeros( 4, 4, n );

            if isa( q_arr , 'sym' )
                exp_T_arr = sym( exp_T_arr );
            end

            for i = 1:n
                exp_T_arr( : , :, i ) = func_getExponential_T( obj.JointTwists( :, i ), q_arr( i ) );
            end

            % Multiply the exponential product with the initial H matrix
            H_FK = func_getExponentialProduct( exp_T_arr ) * H;

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
            JS = zeros( 6, obj.nq ) ;
            exp_T_arr = repmat( eye( 4 ), [ 1, 1, obj.nq ] );
            
            if isa( q_arr , 'sym' )
                JS = sym( JS );
                exp_T_arr = sym( exp_T_arr );
            end    

            for i = 2:n
               exp_T_arr( : , :, i ) = func_getExponential_T( obj.JointTwists( :, i - 1 ), q_arr( i - 1 ) );
            end
            
            for i = 1:n
                % Set the Spatial Jacobian and save the exponential product
                exp_prod = func_getExponentialProduct( exp_T_arr( :, :, 1 : i ) ); 
                JS( :, i ) = func_getAdjointMatrix( exp_prod ) * obj.JointTwists( :, i );
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

            p_tilde = func_skewSym( H( 1:3, 4 ) );
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
                H = obj.getForwardKinematics( q_arr, 'bodyID', bodyID, varargin{ : });
            end

            JS = obj.getSpatialJacobian( q_arr, 'bodyID', bodyID );
            JB = func_getInvAdjointMatrix( H ) * JS;

        end


        function M = getMassMatrix( obj, q_arr )
            % generalized inertia tensor
            % Eq. 4.19 from
            % [REF] A mathematical introduction to robotic manipulation. (2017)

            M = zeros( obj.nq, obj.nq );

            % Conduct JT M J
            % Iterating over each segment
            for i = 1:obj.nq

                JB = obj.getBodyJacobian( q_arr, i, "COM" );

                % Summing over the mass matrix
                M = M + JB.' *  obj.M_Mat( :, :, i ) * JB;
            end

        end

        function G = getGravityVector( obj, q_arr )
            % Gravity Co-vector
            % Simply getting it from tau = J^T (F)
            % where F = mg
            g = obj.Grav;
            G = zeros( obj.nq, 1 );

            if isa( q_arr, 'sym' )
                G = sym( G );
            end

            % Iterating through the COMs of the robot
            for i = 1 : obj.nq

                % The force acting upon the COM
                % If object dimension 2D, then force vector is along y axis
                if obj.Dimension == 2
                    F = [ 0; -obj.Masses( i ) * g; 0; ];
                else
                    F = [ 0; 0; -obj.Masses( i ) * g ];
                end

                % Getting the Hybrid Jacobian of the robot
                JH = obj.getHybridJacobian( q_arr, 'bodyID', i, 'position', 'COM' );
                G = G - JH( 1:3, : ).' * F;
            end

        end
        
        function C_mat = getCoriolisMatrix( obj, q_arr, dq_arr )
            % Calculating the Coriolis-Centrifugal Term of the Robot
            % Based on equation 4.30 of REF
            % [REF] A mathematical introduction to robotic manipulation. (2017)
            % Calculating all of the \partial Mij/\partial thetak values
            % Ordered in i, j, k order
            
            C_mat = zeros( obj.nq, obj.nq );
            B_mat = zeros( obj.nq, obj.nq, obj.nq );
            
            % To speed up the calculation, we need to define the 
            % A matrices before hand
            % A matrix is a 2D( (4xnq)x(4xnq) matrix)
            % First, define the A matrix 
            A_mat = kron( eye( 6 ), eye( obj.nq ) );
            
            % Next, we define a 3D matrix to save the exponential array vals
            % We don't need the first one hence neglecting it.
            exp_T_arr = repmat( eye( 4 ), [ 1, 1, obj.nq - 1 ] );            
            
            if isa( q_arr , 'sym' )
                B_mat = sym( B_mat );
                A_mat = sym( A_mat );
                exp_T_arr = sym( exp_T_arr );
            end
            
            % Conduct mutliplication of T01 T12 T23 ... T(n-1)n
            % Initialization of the exponential product matrix
            for i = 1 : obj.nq-1
                exp_T_arr( : , :, i ) = func_getExponential_T( obj.JointTwists( :, i + 1 ), q_arr( i + 1 ) );
            end
            
            % Define the A Matrices 
            for i = 1 : obj.nq - 1
                for j = 1 : obj.nq - i
                    idx1 = (6 * i) + 6 * (j - 1);
                    idx2 = idx1 - 6 * ( i ); 
                    H = func_getExponentialProduct( exp_T_arr( :, :, j : (i + j - 1) ) );
                    A_mat( idx1 + 1: idx1 + 6, idx2 + 1 : idx2 + 6 ) = func_getInvAdjointMatrix( H );
                end
            end

            for i = 1: obj.nq
                for j = 1 : obj.nq
                    for k = 1 : obj.nq 
                        
                        % The joint twists
                        t_i = obj.JointTwists( :, i );
                        t_j = obj.JointTwists( :, j );
                        t_k = obj.JointTwists( :, k );
                        
                        % Initializing the value to sum over
                        val = 0;

                        for l = max( i , j ) : obj.nq

                            % Get the mass matrix comma (Ml) (Eq. 4.28) (Ibid)
                            tmp = func_getInvAdjointMatrix( obj.H_COM_init( :, :, l ) );
                            Ml  = tmp' * obj.M_Mat( :, :, l ) * tmp;

                            % Calculating the necessary A matrices (Eq. 4.27) (Ibid)
                            % To do, getA method was separately implemented,
                            % due to the potential usage of getA in the
                            % near future.
                            Aki = A_mat( 6*( k - 1 ) + 1 : 6*k, 6*( i - 1 ) + 1 : 6*i );
                            Akj = A_mat( 6*( k - 1 ) + 1 : 6*k, 6*( j - 1 ) + 1 : 6*j );
                            Alk = A_mat( 6*( l - 1 ) + 1 : 6*l, 6*( k - 1 ) + 1 : 6*k );
                            Alj = A_mat( 6*( l - 1 ) + 1 : 6*l, 6*( j - 1 ) + 1 : 6*j );                            
                            Ali = A_mat( 6*( l - 1 ) + 1 : 6*l, 6*( i - 1 ) + 1 : 6*i );                                                        

                            % Calculation (Eq. 4.28) (Ibid)
                            % func_LieBracket was implemented
                            val = val + func_LieBracket( Aki * t_i, t_k ).' * Alk.' * Ml * Alj * t_j + ...
                                        t_i.' * Ali.' * Ml * Alk * func_LieBracket( Akj * t_j, t_k );
                           
                        end
                        
                        % Savethe \partial Mij/\partial thetak value
                        B_mat( i, j, k ) = val;
                    end
                    
                end
            end
            
            gamma = 1/2 * ( B_mat + permute( B_mat, [ 1, 3, 2 ] ) - permute( B_mat, [ 3, 2, 1 ] ) );
            
            % [REF] https://www.mathworks.com/matlabcentral/answers/609866-multiply-an-array-of-scalars-by-a-3d-matrix
            for i = 1 : obj.nq
                C_mat = C_mat + gamma( :, :, i ) * dq_arr( i );
            end
          
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
                obj.H_ij( :, :, i ) = func_getExponential_T( joint_twist, q_arr( i ) ) * H_ini;
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

            % The axis direction should be (3 x 1) array
            assert( isequal( size( axis_direction ), [ 3, 1 ] ) )

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
            % Extend the robot!
            % For the extension, we create a new robot
            robot_new = RobotPrimitive( );

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
            robot_new.JointTypes = horzcat( obj.JointTypes , robot2add.JointTypes  );

            % Extending the axis origins and directions
            % For that, we get the end-effector position
            pEE = obj.H_init( 1 : 3 , 4, end );
            
            robot_new.AxisOrigins    = horzcat( obj.AxisOrigins   , pEE + robot2add.AxisOrigins    );
            robot_new.AxisDirections = horzcat( obj.AxisDirections,       robot2add.AxisDirections );
            
            % =================================== %
            % ======== The H Matrix (SE3) ======= %
            % =================================== %            
            % Setting the H_init of the COM and the robot's joints
            % ----------------------------------------Excluding EE -----
            robot_new.H_init     = cat( 3, obj.H_init( :, :, 1:end - 1 ), robot2add.H_init );
            robot_new.H_COM_init = cat( 3, obj.H_COM_init               , robot2add.H_init );

            % Adding pEE for robot2's H_init
            robot_new.H_init(     1 : 3, 4, obj.nq + 1: end ) = pEE + robot_new.H_init(     1 : 3, 4, obj.nq + 1: end );
            robot_new.H_COM_init( 1 : 3, 4, obj.nq + 1: end ) = pEE + robot_new.H_COM_init( 1 : 3, 4, obj.nq + 1: end );
                
            % ================================ %
            % ===== Inertial Properties ====== %
            % ================================ %
            robot_new.Masses   = horzcat(  obj.Masses, robot2add.Masses   );
            robot_new.Inertias = cat( 3, obj.Inertias, robot2add.Inertias );
            robot_new.M_Mat    = cat( 3, obj.M_Mat   , robot2add.M_Mat    );
            
            % =================================== %
            % ====== Animation Properties ======= %
            % =================================== %
            
            % Extending the MarkerSize
            robot_new.gMarkerSize = horzcat( obj.gMarkerSize, robot2add.gMarkerSize );

            % Rewriting the end-effector as the provided one
            % Reset the x y positions of the end effector
            gEE_new = robot2add.gEE;

            % Change the x (2) and y (3) location 
            gEE_new{ 2 } = robot_new.H_init( 1, 4, end );
            gEE_new{ 3 } = robot_new.H_init( 2, 4, end );
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

