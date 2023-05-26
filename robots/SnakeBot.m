classdef SnakeBot < RobotPrimitive & handle
    % Constructs a planar n-DOF Robot.
    % 
    % The links are assumed to have identical masses and lengths.
    % Moreover, uniform mass density is assumed for each link.
    % 
    % Parameters
    % ----------
    %     n : int
    %         The number of linkages for the snakebot.
    %     m_arr : float array
    %         The mass (kg) array of the n linkages, size of 1 x n
    %     l_arr : float array
    %         The length (m) array of the n linkages, size of 1 x n
    % 
    % Error
    % -----
    %    `n` must be an integer. `m_arr` and `l_arr` must all have positive values.

    properties

        LinkLengths
        
    end
    
    methods
        function obj = SnakeBot( nq, m_arr, l_arr )

            % ======================================================= %
            % ============ BASIC PROPERTIES OF THE ROBOT ============ %
            % ======================================================= %
            obj.Name      = 'SnakeBot';
            obj.nq        = nq;
            obj.ParentID  = 0 : 1 : nq;
            obj.Dimension = 2;

            assert( all( m_arr > 0 ) && all( l_arr > 0 ) )
            assert( all( size( m_arr ) == [ 1, obj.nq ] ) && ...
                    all( size( l_arr ) == [ 1, obj.nq ] ) )

            obj.LinkLengths = l_arr;

            % ======================================================= %
            % ====== GEOMETRIC/INERTIA PROPERTIES OF THE ROBOT ====== %
            % ======================================================= %

            % The geometrical and inertia property of the Robots
            obj.Masses   = m_arr;
            obj.Inertias = zeros( 6, obj.nq );
            obj.Inertias( 3, : ) = 1/12 * obj.Masses .* obj.LinkLengths.^2;

            % The ineria tensor along the principal axis

            % ======================================================= %
            % ============ JOINT PROPERTIES OF THE ROBOT ============ %
            % ======================================================= %          
            % All of the joints are revolute joints
            obj.JointTypes = ones( 1, nq );

            % The initial position of the joints (q_i_arr) are 
            % x = [ 0, l1, l1 + l2, ... , l1 + l2 + ... + ln ]
            % All other values are zero 
            % We define q_i_arr which is the initial position
            % q_i_arr is sized as 3xnq
            % First, calculating the x positions of the matrix
            obj.AxisOrigins = zeros( 3, nq );
            obj.AxisOrigins( 1, 2: end ) = cumsum( obj.LinkLengths( 1 : end-1 ) );

            % For the snake robot, the axes are along +z, (0,0,1)
            obj.AxisDirections = repmat( [ 0; 0; 1 ], 1, nq );


            % ======================================================= %
            % =========== INITIAL H MATRICES OF THE ROBOT =========== %
            % ======================================================= %    
            % Calculating the initial H array, including end-effector (EE)
            % Since we have included the EE, size is nq + 1
            % Construct a (4 x 4 x (nq+1)) matrix
            obj.H_init = repmat( eye( 4 ), [ 1, 1, nq + 1 ] ); 

            % Fill-in the H_init matrix with 
            % x locations of the joint (including EE) is located at:
            % [ 0, l1, l1+l2, .... , l1+l2+...+ln ]
            obj.H_init( 1, 4, : ) = [ 0, cumsum( obj.LinkLengths ) ];
                       
            % Get initial transformations of point on COM of each link
            % The COM of each link is located at the geometric center,
            % i.e., the half of the distance.
            % which is (l1/2, l1 + l2/2, ..., l1 + ... + l(n-1) + ln/2
            obj.H_COM_init  = repmat( eye( 4 ), [ 1, 1, nq ] ); 
            obj.H_COM_init( 1, 4, : ) = [ 0, cumsum( obj.LinkLengths( 1: end-1 ) ) ] + ...
                                        0.5 * obj.LinkLengths;
                        
            % ======================================================= %
            % ============= GRAPHIC OBJECTS OF THE ROBOT ============ %
            % ======================================================= %                                    
            % Specifying the joint marker size
            obj.gMarkerSize = 18 * ones( 1, obj.nq );
            
            
            % For all robots, we need to specify 
            % (1) the base graphic design
            % (2) the end-effector graphic design 
            % For this robot, there is no base object
            tmpl = 1/3;
            x = [ 0,  1.5*tmpl, -1.5*tmpl 0];
            y = [ 0, -1.5*tmpl  -1.5*tmpl 0];            
            obj.gBase = { @plot, x, y, 'k-', 'linewidth', 3 };
            
            % The end-effector will be a marker with same size of others
            % Collect all the details as a cell
            obj.gEE   = { @scatter, obj.H_init( 1, 4, obj.nq + 1 ), obj.H_init( 2, 4, obj.nq + 1 ), 180, ...
                          'o', 'MarkerFaceColor', 'k', ...
                          'MarkerEdgeColor', 'k' };                                    
        end


    end
    
end

