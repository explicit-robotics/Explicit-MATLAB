function arr = se3_to_R6( mat )

assert( is_skewsym( mat( 1:3, 1:3 ) ) );

arr = zeros( 1, 6 );
arr( 1 ) = mat( 1, 4 );
arr( 2 ) = mat( 2, 4 );
arr( 3 ) = mat( 3, 4 );

arr( 4 ) = -mat( 2, 3 );
arr( 5 ) =  mat( 1, 3 );
arr( 6 ) = -mat( 1, 2 );


end