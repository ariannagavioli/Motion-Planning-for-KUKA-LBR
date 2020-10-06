function cart_ee = directKinematics(q)
x_ee = (21*sin(q(6))*(cos(q(5))*(cos(q(4))*(sin(q(1))*sin(q(3)) - cos(q(1))*cos(q(2))*cos(q(3))) - cos(q(1))*sin(q(2))*sin(q(4))) + sin(q(5))*(cos(q(3))*sin(q(1)) + cos(q(1))*cos(q(2))*sin(q(3)))))/100 - (21*cos(q(6))*(sin(q(4))*(sin(q(1))*sin(q(3)) - cos(q(1))*cos(q(2))*cos(q(3))) + cos(q(1))*cos(q(4))*sin(q(2))))/100 - (21*cos(q(1))*sin(q(2)))/50 - (2*sin(q(4))*(sin(q(1))*sin(q(3)) - cos(q(1))*cos(q(2))*cos(q(3))))/5 - (2*cos(q(1))*cos(q(4))*sin(q(2)))/5;
y_ee = (21*cos(q(6))*(sin(q(4))*(cos(q(1))*sin(q(3)) + cos(q(2))*cos(q(3))*sin(q(1))) - cos(q(4))*sin(q(1))*sin(q(2))))/100 - (21*sin(q(1))*sin(q(2)))/50 - (21*sin(q(6))*(cos(q(5))*(cos(q(4))*(cos(q(1))*sin(q(3)) + cos(q(2))*cos(q(3))*sin(q(1))) + sin(q(1))*sin(q(2))*sin(q(4))) + sin(q(5))*(cos(q(1))*cos(q(3)) - cos(q(2))*sin(q(1))*sin(q(3)))))/100 + (2*sin(q(4))*(cos(q(1))*sin(q(3)) + cos(q(2))*cos(q(3))*sin(q(1))))/5 - (2*cos(q(4))*sin(q(1))*sin(q(2)))/5;
z_ee = (21*cos(q(2)))/50 + (2*cos(q(2))*cos(q(4)))/5 + (21*sin(q(6))*(cos(q(5))*(cos(q(2))*sin(q(4)) - cos(q(3))*cos(q(4))*sin(q(2))) + sin(q(2))*sin(q(3))*sin(q(5))))/100 + (21*cos(q(6))*(cos(q(2))*cos(q(4)) + cos(q(3))*sin(q(2))*sin(q(4))))/100 + (2*cos(q(3))*sin(q(2))*sin(q(4)))/5 + 9/25;

cart_ee = [x_ee; y_ee; z_ee];
end

