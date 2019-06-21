%input theta1.2...6
clc;
clear all;
theta=[1 2 3 4 5 6 ];
DH_alpha=[1 2 3 4 5 6 ];
DH_a=[1 2 3 4 5 6 ];
DH_d=[1 2 3 4 5 6 ];

T_0{1}=ones(4:4);
for i=2:1:6
    T_0{i}=T_0{i-1}*[cos(theta(i)),-cos(DH_alpha(i)).*sin(theta(i)), sin(DH_alpha(i)).*sin(theta(i)),DH_a(i).*cos(theta(i));
                       sin(theta(i)),cos(DH_alpha(i)).*cos(theta(i)),-sin(DH_alpha(i)).*cos(theta(i)),DH_a(i).*sin(theta(i));
                       0, sin(DH_alpha(i)), cos(DH_alpha(i)), DH_d(i);
                       0, 0, 0, 1];
end

n_x=T_0{6}(1,1);
n_y=T_0{6}(2,1);
n_z=T_0{6}(3,1);
s_x=T_0{6}(1,2);
s_y=T_0{6}(2,2);
s_z=T_0{6}(2,3);
a_x=T_0{6}(1,3);
a_y=T_0{6}(2,3);
a_z=T_0{6}(3,3);

phi=atan2(n_y,n_x);
thetaa=atan2(-n_z,cos(phi)*n_z+sin(phi)*n_y);
gamma=atan2(sin(phi)*a_x-cos(phi)*a_y, -sin(phi)*s_x+cos(phi)*s_y);

final_position=[T_0{6}(1,4), T_0{6}(2,4), T_0{6}(3,4), phi, thetaa, gamma]