clear;
clc;
a2 = 105;
a3 = 113.7;

b2 = a2 * 0.7;
b3 = a3 * 0.6;

N=10;
for i=1:N
    x(i)=b2+b3*sqrt(1/2);
    z(i)=b3*sqrt(1/2);
    y(i)=b2/N*i;
    x(N+i)=b2+b3*sqrt(1/2);
    y(N+i)=b2;
    z(N+i)=b3*sqrt(1/2)-b3*sqrt(1/2)/N*i;
    x(2*N+i)=b2+b3*sqrt(1/2);
    z(2*N+i)=0;
    y(2*N+i)=b2-b2/N*i;
    x(3*N+i)=b2+b3*sqrt(1/2);
    y(3*N+i)=0;
    z(3*N+i)=b3*sqrt(1/2)/N*i;
end
plot3(x,y,z);
t = -x;

d = sqrt(x.^2 + y.^2 + z.^2);
theta = zeros(3,length(t));
theta(3,:) = acos((d.^2 - a2^2 - a3^2) / (2 * a2 * a3)) - pi;
global k1 k2 zt;
t0 = 0;
for i = 1:length(t)
    k1 = a2 + a3 * cos(theta(3,i));
    k2 = a3 * sin(theta(3,i));
    zt = z(i);
    theta(2,i) = fmincon('fun',t0,[],[],[],[],0,pi/2);
end
theta(1,:) = -acos(y ./ (a2 * cos(theta(2,:)) + a3 * (cos(theta(2,:) + theta(3,:))))) + pi/2;

theta(2,:) = -theta(2,:);
theta(3,:) = -theta(3,:);
n = 1:length(t);
%plot(n,theta(1,:),'r',n,theta(2,:),'g',n,theta(3,:),'b');
fid1=fopen('C:\Users\apple.com.cn\Desktop\robotics\theta1.txt','w');
fprintf(fid1,'%6.2f,',theta(1,:));
fclose(fid1);
fid2=fopen('C:\Users\apple.com.cn\Desktop\robotics\theta2.txt','w');
fprintf(fid1,'%6.2f,',-theta(2,:));
fclose(fid2);
fid3=fopen('C:\Users\apple.com.cn\Desktop\robotics\theta3.txt','w');
fprintf(fid3,'%6.2f,',theta(3,:));
fclose(fid3);
for i=1:40
    time(i)=0.4*i;
end
fid4=fopen('C:\Users\apple.com.cn\Desktop\robotics\time.txt','w');
fprintf(fid4,'%6.2f,',time(1,:));
fclose(fid4);
