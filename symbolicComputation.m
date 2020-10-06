%http://www.diag.uniroma1.it/~deluca/rob1_en/09_Exercise_DH_KukaLWR4.pdf
clear;
clc;
DH_table;
joint_hom_mat = sym(zeros(4, 4*num_joint));
for i = 1:num_joint
    param_i = dh_table(i,:);
    joint_hom_mat(:,(i-1)*4+1:(i-1)*4+4) = [cos(param_i(4)),    -sin(param_i(4))*cos(param_i(1)),  sin(param_i(4))*sin(param_i(1)), param_i(3)*cos(param_i(4));
                                            sin(param_i(4)),     cos(param_i(4))*cos(param_i(1)), -cos(param_i(4))*sin(param_i(1)), param_i(3)*sin(param_i(4));
                                                          0,                     sin(param_i(1)),                  cos(param_i(1)),                 param_i(2);
                                                          0,                                   0,                                0,                         1];

end
direct_kin = eye(4,4);
for i = 1:num_joint
    direct_kin = direct_kin * joint_hom_mat(:,(i-1)*4+1:(i-1)*4+4);
end

direct_kin = simplify(direct_kin * [0, 0, 0, 1]');

ana_jac = simplify(jacobian(direct_kin(1:3), [q1, q2, q3, q4, q5, q6, q7]));

%direct kin for obstacle avoidance points
%in homo coordinate
point_0_0 = [0;0;0.36;1];
point_1_2 = simplify(joint_hom_mat(:,1:4)*joint_hom_mat(:,5:8)*[0;0;0.2;1]);
point_2_3 = simplify(joint_hom_mat(:,1:4)*joint_hom_mat(:,5:8)*joint_hom_mat(:,9:12)*[0;0;0;1]);
point_3_4 = simplify(joint_hom_mat(:,1:4)*joint_hom_mat(:,5:8)*joint_hom_mat(:,9:12)*joint_hom_mat(:,13:16)*[0;0;0.2;1]);
point_4_5 = simplify(joint_hom_mat(:,1:4)*joint_hom_mat(:,5:8)*joint_hom_mat(:,9:12)*joint_hom_mat(:,13:16)*joint_hom_mat(:,17:20)*[0;0;0;1]);
point_5_7 = direct_kin;

%grad = gradient(((points(index,:)' - obs)'*(points(index,:)' - obs)),[q1;q2;q3;q4;q5;q6;q7]);