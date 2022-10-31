arb = Arbotix('port', '/dev/tty.usbserial-A800JDPN', 'nservos', 5)


t = [];

profile clear
profile on
for i=1:100
    tic
    q = arb.getpos(1);
    t(i) = toc;
end

profile off
t
mean(t)
std(t)

for j=1:5
    s(j).model = arb.readdata2(j, 0);
    s(j).firmware = arb.readdata1(j, 2);
    s(j).id = arb.readdata1(j, 3);
    s(j).retdelay = arb.readdata1(j, 5);
    s(j).torqen = arb.readdata1(j, 24);
    s(j).compmargin_cw = arb.readdata1(j, 26);
    s(j).compmargin_ccw = arb.readdata1(j, 27);
    s(j).compslope_cw = arb.readdata1(j, 28);
    s(j).compslope_ccw = arb.readdata1(j, 29);
end

arb.disconnect()

profile report

% arb.command(3, arb.INS_WRITE_DATA, q(3) )
% pause(1)
% arb.command(4, arb.INS_WRITE_DATA, q(4) )