classdef Animation < handle
    % An Animation Class
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

        % The X, Y, Z axis limit
        xLim
        yLim
        zLim

        % Title/SubTitle of the animation
        Title
        SubTitle

        % Dimension of the robot, either 2D or 3D
        Dimension

        % The time of the current simulation
        t = 0

        % =================================== %
        % ======== VIDEO Properties ========= %
        % =================================== %

        % Variables for saving the animation
        isSaveVideo
        Video
        VideoSpeed
        FrameUpdateTime

        % The number of current frame
        % This will be useful for video saving functionality
        nFrame = 0

        % Handles
        hFig
        hAxes

        % The Argument Parser
        ArgParse

        % The robot object in the animation
        % There can be multiple Robots
        Robots = {};
        nRobots = 0;

        % =================================== %
        % ======== GRAPHIC Properties ======= %
        % =================================== %
        gLinks   = {};
        gObjs    = {};        % Graphics from the object file.
        gPatches = {};
    end

    methods

        function obj = Animation( varargin )

            % Define the Argument Parser to parse the varargin
            obj.ArgParse = inputParser;

            % Set the case sensitive to be "true"
            % [TODO] [Moses C. Nah] [2022.09.02]
            % The current style is too strict to the user for passing the
            % variables' names. Hence this strictness should be loosen
            % up later
            obj.ArgParse.CaseSensitive = true;

            % The Title of the Animation
            addParameter( obj.ArgParse, 'Title', 'Robot Simulator' )

            % Add the dimension of the argument parser
            % The third variable is the default value.
            addParameter( obj.ArgParse, 'Dimension', 2, @(x) ( x == 2 || x == 3 ) )

            % Add the x/y/zLim of the argument parser
            % The third variable is the default value.
            check_input = @(x) ( isequal( size( x ), [ 1, 2 ] ) );
            addParameter( obj.ArgParse, 'xLim', [ -1, 1 ], check_input  )
            addParameter( obj.ArgParse, 'yLim', [ -1, 1 ], check_input  )
            addParameter( obj.ArgParse, 'zLim', [ -1, 1 ], check_input  )

            % Check whether the video should be saved or not
            check_input = @(x) ( x == false || x == true );
            addParameter( obj.ArgParse, 'isSaveVideo', false, check_input )

            % Check whether the video should be saved or not
            check_input = @(x) ( x > 0 );
            addParameter( obj.ArgParse, 'VideoSpeed', 1.0, check_input )

            % Parse the varargin
            parse( obj.ArgParse , varargin{ : } );

            % Set the Properties of the class as ArgParse.
            arg_names = [ "Dimension", "xLim", "yLim", "zLim", 'Title', 'isSaveVideo', 'VideoSpeed' ];

            % Cleaning-up the passed arguments
            for i = 1 : length( arg_names )
                name = arg_names{ i };
                obj.( name ) = obj.ArgParse.Results.( name );
            end

        end

        function init( obj )

            % Saving time step
            obj.FrameUpdateTime = 1.0 / 30 * obj.VideoSpeed;

            % If Save video is on.
            if obj.isSaveVideo
                obj.Video = VideoWriter( obj.Title, 'MPEG-4' );

                % The default frame rate of the video is 30
                obj.Video.FrameRate = 30;

                % Open the video file
                open( obj.Video )
            end

            % Setting up the figure of the animation
            obj.hFig = figure( );
            set( obj.hFig, 'Color',[ 1, 1, 1 ], 'Name', obj.Title, 'NumberTitle', 'off')

            % Setting up the Axes of the animation
            obj.hAxes = axes( 'parent', obj.hFig );

            axis equal
            axis manual
            hold( obj.hAxes, 'on' )
            grid( obj.hAxes, 'on' )

            set( obj.hAxes, 'xlim', obj.xLim, 'ylim', obj.yLim, 'zlim', obj.zLim )
            xlabel( obj.hAxes, 'x (m)' )
            ylabel( obj.hAxes, 'y (m)' )

            % Title: simulation time
            obj.SubTitle = title( sprintf( 'Time: %2.1f sec', obj.t ) );
            set( obj.SubTitle, 'FontSize' , 15);

            if obj.Dimension == 3
                zlabel( obj.hAxes, 'z (m)' )
                lighting gouraud;
                camlight('headlight');
                light("Style","local","Position",[0.9, 0, 0.5]);
                light("Style","local","Position",[0, 0.4, 0.5]);
                view( 135, 10 );
            end
        end

        function attachRobot( obj, robot, varargin )
            % Attaching robot to the animation.

            % For attaching the robot, we create the graphic objects
            % Based on the joint type and size of the joint markers,
            % Specified as gMarkerSize

            % Add the robot to the cell
            obj.nRobots = obj.nRobots + 1;

            % For notational abbreviation, set n as the current number
            % of the robots
            n = obj.nRobots;
            obj.Robots{ n } = robot;

            % The initial settings of the object
            prefix = 'robot';

            % Creating the hggroup for the graphs
            % prefix "g" is added to stand for "{g}raphical objects
            % The first element is for the base
            obj.gLinks{ n } = cell( 1, robot.nq + 1 );

            % Create empty hggroup and save it on the list
            for i = 1: ( robot.nq + 1 )
                obj.gLinks{ n }{ i } = hggroup( );
            end


            % ========================================= %
            % ====== In case gObjects are given ======= %
            % ========================================= %
            if ~isempty( robot.gObjs )

                for i = 1 : robot.nq+1

                    % Check if the faces are 3 values rather than 6 values
                    if size( robot.gObjs.data{ i }.f, 2 ) == 6
                        obj.gPatches{ i } = patch( obj.hAxes, 'faces', robot.gObjs.data{ i }.f( :, 1:3 ),'vertices', robot.gObjs.data{ i }.v, ...
                                        'EdgeColor', 'none', 'FaceVertexCData', robot.gObjs.data{ i }.f( :, 4:6 ), 'facecolor', 'flat' );
                    else
                        obj.gPatches{ i } = patch( obj.hAxes, 'faces', robot.gObjs.data{ i }.f,'vertices', robot.gObjs.data{ i }.v, ...
                                        'EdgeColor', 'none' );
                    end

                    set( obj.gPatches{ i }, 'Parent', obj.gLinks{ n }{ i } );
                end

                % =================================================== %
                % In case the graphics should be created from scratch %
                % =================================================== %
            else

                % Creating the base, which can be conducted with the
                % robot gBase's cell
                % First check whether the base is empty or not
                if ~isempty( robot.gBase )
                    gBase = robot.gBase{ 1 }( obj.hAxes, robot.gBase{ 2 : end } );
                    set( gBase, 'Parent', obj.gLinks{ n }{ 1 } );
                end

                % Iterating through the objects
                for i = 1:robot.nq

                    % Creating the graphics
                    % The only information required is the
                    % length of the consecutive limbs.
                    % This can be calculated via H_init matrix

                    % First, the joint of the SnakeBot
                    % If the joint is a rev. joint
                    if robot.JointTypes( i ) == 1
                        markerstyle = 'o';
                        markerfacecolor = [ 0, 0.4470, 0.7410 ];

                        % Else if the joint is a prism. joint.
                    else
                        markerstyle = 's';
                        markerfacecolor = [ 0.8500, 0.3250, 0.0980 ];
                    end

                    % Plot the Joints as a marker
                    % Note that the H_init( 1:3, 4, i ) is the initial pos.
                    % of the i-th joint.
                    gJoint = scatter( robot.H_init( 1, 4, i ), robot.H_init( 2, 4, i ), 10*robot.gMarkerSize( i ), ...
                                    'Marker', markerstyle,...
                                    'MarkerFaceColor', markerfacecolor, ...
                                    'MarkerEdgeColor', 'k' );

                    set( gJoint, 'PickableParts', 'none' );

                    % Plot the Link as a line
                    gLink = plot( squeeze( robot.H_init( 1, 4, i:i+1 ) ), ...
                        squeeze( robot.H_init( 2, 4, i:i+1 ) ), ...
                        'LineWidth', 3, ...
                        'Color', 'k' );

                    % Attach the graphs
                    set(  gLink, 'Parent' , obj.gLinks{ n }{ i + 1 } );
                    set( gJoint, 'Parent' , obj.gLinks{ n }{ i + 1 } );

                end

                % Plotting the end-effector
                if ~isempty( robot.gEE )

                    gEndEffector = robot.gEE{ 1 }( obj.hAxes, robot.gEE{ 2 : end } );
                    set( gEndEffector, 'Parent', obj.gLinks{ n }{ robot.nq + 1 } ) ;

                end
            end

            % Iterating
            for i = 0:robot.nq

                if i == 0
                    parent = obj.hAxes;
                    matrix = robot.H_base;
                else
                    tag_parent = [ prefix, num2str( obj.hFig.Number), '_', num2str( obj.nRobots ), '_', num2str( robot.ParentID( i ) ) ];
                    parent = findobj( 'Tag', tag_parent );
                    matrix = eye( 4 );
                end

                tag = [ prefix, num2str( obj.hFig.Number), '_', num2str( obj.nRobots ) , '_' , num2str( i ) ];
                % hgtf for base
                gt = hgtransform( 'Parent', parent, 'Tag', tag, 'Matrix', matrix );
                set( obj.gLinks{ n }{ i+1 } , 'parent', gt );

            end

            disp( ['Patches of ', robot.Name,' initialized.'] );

        end

        function update( obj, t )

            % Update the time of the simulation
            obj.t = t;

            % The robot should NOT be empty before running update.
            assert( ~isempty( obj.Robots ) )

            prefix = 'robot';

            for i = 1 : obj.nRobots
                % Get the transformation matrices for every segment
                nq = obj.Robots{ i }.nq;
                H_ij = obj.Robots{ i }.H_ij;

                % Find the hgtransform handler t by its tag.
                for j = 1:nq

                    % Tag name is usually
                    %       robot1_1, robot1_2
                    Tag = [ prefix,  num2str( obj.hFig.Number), '_',  num2str( i ), '_', num2str( j ) ];
                    
                    gt = findobj( obj.hAxes, 'Tag', Tag );

                    if isempty( gt )
                        error('Figure closed.');
                    else
                        set( gt, 'Matrix', H_ij( :, :, j ) ) ;
                    end

                end
            end

            % Saving the figure if record is on
            % You don't need to update the animation if larger than frame rate.
            if round( obj.t / obj.FrameUpdateTime ) >= obj.nFrame

                obj.nFrame = obj.nFrame + 1;
                drawnow;

                if obj.isSaveVideo
                    writeVideo( obj.Video , getframe( obj.hFig ) )
                end

                set( obj.SubTitle, 'String', sprintf( 'Time: %2.1f sec', obj.t ) );
                obj.t
                               
            end

        end

        function close( obj )

            if obj.isSaveVideo
                close( obj.Video );
            end

        end


    end
end