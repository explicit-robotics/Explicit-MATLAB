function C_mat = coriolis_py( q, dq )
     % Set figure size and attach robot to simulation
    robot = iiwa14_2nd( );
    robot.init( );

    C_mat = robot.getCoriolisMatrix( q, dq' );

end