function [ q1, dq1 ] = func_symplecticEuler( q0, dq0, ddq0 , dt )
% ===========================================================================
% func_symplecticEuler - Semi-implicit Euler method for forward simulation
% [ q1, q1Dot ] = func_symplecticEuler( q0, dq0, ddq0, dt )
%
% Authors                       Email                   Created 
%   [1] Johannes Lachner        jlachner@mit.edu        2019
%   [2] Moses C. Nah            mosesnah@mit.edu
%
% Description
%    - To get the dq and q term, the following integration is conducted:
%           dq{ n+1 } = dq{ n } +  ddq{ n } * dt
%            q{ n+1 } =  q{ n } + dq{ n+1 } * dt
%
% [REF 1] https://mujoco.readthedocs.io/en/latest/computation.html#numerical-integration
% [REF 2] https://en.wikipedia.org/wiki/Semi-implicit_Euler_method
% [REF 3] Claude Lacoursière, 2007, Ghosts and Machines: Regularized Variational Methods for Interactive Simulations of Multibodies with Dry Frictional Contacts
%
% Input 
%   [1] q0   - The  q array of the current time step (0) 
%
%   [2] dq0  - The dq array of the current time step (0) 
%
%   [3] ddq0 - The ddq array of the current time step (0) 
%
%   [4] dt   - The time-step for the forward integration
%
% Output 
%   [1] q1  - The  q array of the next time step (1) after integration
%
%   [2] dq1 - The dq array of the next time step (1) after integration
%
%  ===========================================================================

% Check that the size of the arr are the same


% Calculate the dq term from the rhs_of_ddq0 term
dq1 = dq0 + ddq0 * dt;

% Calculate the q term from the rhs_of_ddq0 term
q1  = q0  + dq1 * dt;