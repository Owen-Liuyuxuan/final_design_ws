m = 1600; lf = 1.2;
lr = 1.6; iz = 2500; v = 10; steering_ratio = 14.8;

c = [30000, 1e5, 1e6, 1e7];

t = 0:0.001:1.5;
y = zeros(max(size(t)), 4);
for i = 1:4
    cr = c(i); cf = 1.3 * c(i);
    A = [-(cf + cr)/(m * v), -(1 + (cf*lf - cr * lr)/(m * v^2));...
    -(cf*lf-cr*lr)/(iz), -(cf * lf^2 + cr * lr^2)/(iz * v)];
    B = [cf/m/v;...
        cf*lf/iz];
    B = B./steering_ratio;
    C = [0 1];
    D = 0;
    ss1 = ss(A,B,C,D);
    y(:,i) = step(ss1, t);
end
plot(t, y(:,1), 'b-', t, y(:,2), 'r-', t,y(:,3), 'g-', t,y(:,4), 'c-')
title('step response for vehicle models with different tire stiffness')
xlabel('time /s')
ylabel('omega /(rad/s)')
legend('C=30000', 'C=1e5', 'C=1e6', 'C=1e7', 'location', 'best')
