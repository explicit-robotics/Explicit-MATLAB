function exp_T = getExpProd( exp_T_arr )
% ========================================================================
% getExpProd: 
%   - An utility function that calculates the product of exponential 
%     formulas
%   exp_T = getExpProd( exp_T_arr )
%
% Authors                         Email
%   Johannes Lachner              jlachner@mit.edu
%   Moses C. Nah                  mosesnah@mit.edu
% 
% Input 
%   [1] exp_T_arr - A (4 x 4 x n) 3D array.
%                   Each (4 x 4) matrix is the exponential matrix
% 
% Output 
%   [1] exp_T - A (4 x 4) matrix
%             - We calculate the exp_T1 x exp_T2 x ... x exp_Tn
%             - where last number is the index of the i-th 
%             - (4 x 4) matrix of the exp_T_arr 3D array
% ========================================================================

% Take a look at the following post, which shows why 
% the unrolling method is way faster than the simple
% exponential product method (shown by Andrei Bobrov)

% Assert that the row (r) and column (c) is (4 x 4)
[r, c, d] = size( exp_T_arr );

assert( r == 4 && c == 4 )

% Unroll the matrices for faster calculation. 
% Although the code is ugly, way more faster than the simple 
% multiplication method
% Renaming the 3D matrix
M = exp_T_arr;

m11 = M(  1 );
m21 = M(  2 );
m31 = M(  3 );
m41 = M(  4 );

m12 = M(  5 );
m22 = M(  6 );
m32 = M(  7 );
m42 = M(  8 );

m13 = M(  9 );
m23 = M( 10 );
m33 = M( 11 );
m43 = M( 12 );

m14 = M( 13 );
m24 = M( 14 );
m34 = M( 15 );
m44 = M( 16 );

c = 1;

if d >= 2
    for i = 2 : d
        c = c + 16;
    
        t11 = m11 * M( c ) + m12 * M( c + 1 ) + m13 * M( c + 2 ) +  m14 * M( c + 3 );
        t21 = m21 * M( c ) + m22 * M( c + 1 ) + m23 * M( c + 2 ) +  m24 * M( c + 3 );
        t31 = m31 * M( c ) + m32 * M( c + 1 ) + m33 * M( c + 2 ) +  m34 * M( c + 3 );
        t41 = m41 * M( c ) + m42 * M( c + 1 ) + m43 * M( c + 2 ) +  m44 * M( c + 3 );
    
        t12 = m11 * M( c + 4 ) + m12 * M( c + 5 ) + m13 * M( c + 6 ) +  m14 * M( c + 7 );
        t22 = m21 * M( c + 4 ) + m22 * M( c + 5 ) + m23 * M( c + 6 ) +  m24 * M( c + 7 );
        t32 = m31 * M( c + 4 ) + m32 * M( c + 5 ) + m33 * M( c + 6 ) +  m34 * M( c + 7 );
        t42 = m41 * M( c + 4 ) + m42 * M( c + 5 ) + m43 * M( c + 6 ) +  m44 * M( c + 7 );    
    
        t13 = m11 * M( c + 8 ) + m12 * M( c + 9 ) + m13 * M( c + 10 ) +  m14 * M( c + 11 );
        t23 = m21 * M( c + 8 ) + m22 * M( c + 9 ) + m23 * M( c + 10 ) +  m24 * M( c + 11 );
        t33 = m31 * M( c + 8 ) + m32 * M( c + 9 ) + m33 * M( c + 10 ) +  m34 * M( c + 11 );
        t43 = m41 * M( c + 8 ) + m42 * M( c + 9 ) + m43 * M( c + 10 ) +  m44 * M( c + 11 );        
    
        m14 = m11 * M( c + 12 ) + m12 * M( c + 13 ) + m13 * M( c + 14 ) +  m14 * M( c + 15 );
        m24 = m21 * M( c + 12 ) + m22 * M( c + 13 ) + m23 * M( c + 14 ) +  m24 * M( c + 15 );
        m34 = m31 * M( c + 12 ) + m32 * M( c + 13 ) + m33 * M( c + 14 ) +  m34 * M( c + 15 );
        m44 = m41 * M( c + 12 ) + m42 * M( c + 13 ) + m43 * M( c + 14 ) +  m44 * M( c + 15 );        
    
        m11 = t11;
        m21 = t21;
        m31 = t31;
        m41 = t41;
    
        m12 = t12;
        m22 = t22;
        m32 = t32;
        m42 = t42;    
    
        m13 = t13;
        m23 = t23;
        m33 = t33;
        m43 = t43;        
    
    
    end
end

exp_T = [ m11, m12, m13, m14; 
          m21, m22, m23, m24; 
          m31, m32, m33, m34; 
          m41, m42, m43, m44 ];
