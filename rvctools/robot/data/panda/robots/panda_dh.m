%% Panda

L1=0.333;

L2=-0.316;
L3=0.0825;
L4=-0.0825; % -L3
L5=0.384;
L6=0.088;
L7=0.107;

s = 'Tz(L1) Rz(q1) Rz(-90) Rz(q2) Ty(L2) Rz(90) Rz(q3) Tx(L3) Rz(90) Rz(q4) Tx(L4) Ty(L5) Rz(-90) Rz(q5) Rz(90) Rz(q6) Tx(L6) Rz(90) Rz(q7) Tz(L7)'

r = DHFactor(s)

%%
L1= 0.6718
L2=0.4318
L3=-0.0203
L4=0.1501
L5=0.4331
L6=0.0558
s= 'Tz(L1) Rx(90) Ry(q1) Rz(q2) Tx(L2) Ty(L3) Tz(L4) Rz(-90) Rz(q3) Rz(90) Rx(90) Rz(q4) Tz(L5) Rx(90) Rz(q5) Ty(L6) Rx(-90) Rz(q6)'
r = DHFactor(s)

%%
L1=0.664
L2=0.4318
L3=0.1291
L4=-0.0203
L5=0.4331

s = 'Rz(q1) Tz(L1) Rx(-90) Rz(q2) Tx(L2) Tz(L3) Rz(q3) Tx(L4) Rx(90) Rz(q4) Tz(L5) Rx(-90) Rz(q5) Rx(90) Rz(q6)'
r = DHFactor(s)