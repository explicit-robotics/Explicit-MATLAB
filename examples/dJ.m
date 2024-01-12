function C_mat = dJ( q, dq )
     % Set figure size and attach robot to simulation
    robot = iiwa14_2nd( );
    robot.init( );
    
    C_mat = robot.getHybridJacobian( q );

end