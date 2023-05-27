function [index,theta,dtheta,ddtheta] = traj_poly3(init_q,final_q,t_span,division)
% both init_q and final_q should be in masurement of 'rad'
t_f=t_span;
theta_i=init_q;
theta_f=final_q;

a1=[0 0 0 0];
a1(1)=theta_i;
a1(2)=0;
a1(3)=3/t_f^2*(theta_f-theta_i);
a1(4)=-2/t_f^3*(theta_f-theta_i);

t=linspace(0,t_f,division);
theta=a1(1)+a1(2)*t+a1(3)*t.^2+a1(4)*t.^3;
dtheta=a1(2)+a1(3)*t*2+3*a1(4)*t.^2;
ddtheta=a1(3)*2+6*a1(4)*t;

index=1:length(t)
end