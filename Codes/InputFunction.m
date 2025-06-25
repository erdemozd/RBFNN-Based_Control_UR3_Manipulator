function [sys, x0, str, ts] = InputFunction(t,x,u,flag)
switch flag
    case 0
        [sys,x0,str,ts] = mdlInitializeSizes;
    case 1
        sys = mdlDerivatives(t,x,u);
    case 3
        sys = mdlOutputs(t,x,u);
    case {2,4,9}
        sys = [];
    otherwise
        error(['Unhandledflag = ',num2str(flag)])
end


function [sys,x0,str,ts] = mdlInitializeSizes
sizes = simsizes;
sizes.NumContStates = 0;
sizes.NumDiscStates = 0;
sizes.NumOutputs = 18;
sizes.NumInputs = 0;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;
sys = simsizes(sizes);
x0 = [];
str = [];
ts = [0 0];
end

function sys = mdlOutputs(t,x,u)
qd1 = 0.1*sin(2*t);
dqd1 = 0.1*cos(2*t);
ddqd1 = -0.1*sin(2*t);

qd2 = 0.1*sin(2*t);
dqd2 = 0.1*cos(2*t);
ddqd2 = -0.1*sin(2*t);

qd3 = 0.1*sin(2*t);
dqd3 = 0.1*cos(2*t);
ddqd3 = -0.1*sin(2*t);

qd4 = 0.1*sin(2*t);
dqd4 = 0.1*cos(2*t);
ddqd4 = -0.1*sin(2*t);

qd5 = 0.1*sin(2*t);
dqd5 = 0.1*cos(2*t);
ddqd5 = -0.1*sin(2*t);

qd6 = 0.1*sin(2*t);
dqd6 = 0.1*cos(2*t);
ddqd6 = -0.1*sin(2*t);

sys(1) = qd1;
sys(2) = dqd1;
sys(3) = ddqd1;
sys(4) = qd2;
sys(5) = dqd2;
sys(6) = ddqd2;
sys(7) = qd3;
sys(8) = dqd3;
sys(9) = ddqd3;
sys(10) = qd4;
sys(11) = dqd4;
sys(12) = ddqd4;
sys(13) = qd5;
sys(14) = dqd5;
sys(15) = ddqd5;
sys(16) = qd6;
sys(17) = dqd6;
sys(18) = ddqd6;
end
end