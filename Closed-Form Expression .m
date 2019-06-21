%input theta1.2...6
clc;
clear all;
DH_alpha=[pi/2 0 0];
DH_a=[0 0.105 0.113 ];
DH_d=[0 0 0];
syms theta1;
syms theta2;
syms theta3;
theta=[theta1 theta2 theta3];

for i=1
    T_0_1=[cos(theta(i)),-cos(DH_alpha(i))*sin(theta(i)), sin(DH_alpha(i))*sin(theta(i)),DH_a(i)*cos(theta(i));
       sin(theta(i)),cos(DH_alpha(i))*cos(theta(i)),-sin(DH_alpha(i))*cos(theta(i)),DH_a(i)*sin(theta(i));
       0, sin(DH_alpha(i)), cos(DH_alpha(i)), DH_d(i);
       0, 0, 0, 1];
end

for i=2
    T_1_2=[cos(theta(i)),-cos(DH_alpha(i))*sin(theta(i)), sin(DH_alpha(i))*sin(theta(i)),DH_a(i)*cos(theta(i));
       sin(theta(i)),cos(DH_alpha(i))*cos(theta(i)),-sin(DH_alpha(i))*cos(theta(i)),DH_a(i)*sin(theta(i));
       0, sin(DH_alpha(i)), cos(DH_alpha(i)), DH_d(i);
       0, 0, 0, 1];
end

for i=3
    T_2_3=[cos(theta(i)),-cos(DH_alpha(i))*sin(theta(i)), sin(DH_alpha(i))*sin(theta(i)),DH_a(i)*cos(theta(i));
       sin(theta(i)),cos(DH_alpha(i))*cos(theta(i)),-sin(DH_alpha(i))*cos(theta(i)),DH_a(i)*sin(theta(i));
       0, sin(DH_alpha(i)), cos(DH_alpha(i)), DH_d(i);
       0, 0, 0, 1];
end

%syms n_x s_x a_x p_x n_y s_y a_y p_y n_z s_z a_z p_z;
global n_x s_x a_x p_x n_y s_y a_y p_y n_z s_z a_z p_z;
n_x=1; s_x=1; a_x=1; p_x=1; n_y=1; s_y=1; a_y=1; p_y=1; n_z=1; s_z=1; a_z=1; p_z=1;
T_0_3=[n_x s_x a_x p_x
       n_y s_y a_y p_y
       n_z s_z a_z p_z
       0 0 0 1];

ans1=inv(T_0_1)*T_0_3;
ans2=T_1_2*T_2_3;
A=(ans1-ans2);
B=A(:);
options=optimset('display','off');
fsolve(@(theta) myfun_express(theta),[1;0.5;3],options)

   
    