function J = get_analytic_jacobian(q)
    
    q1 = q(1);
    q2 = q(2);
    q3 = q(3);
    q4 = q(4);
    q5 = q(5);
    q6 = q(6);
    q7 = q(7);

    % syms q1 q2 q3 q4 q5 q6 q7 d1 d2
    % 
    % T01r = [cos(q1), -sin(q1), 0;
    %        sin(q1) , cos(q1), 0;
    %         0, 0, 1] * [1, 0, 0;
    %                     0, 0, -1;
    %                     0, 1, 0];
    % 
    % T12r = [cos(q2), -sin(q2), 0;
    %        sin(q2) , cos(q2), 0;
    %         0, 0, 1] * [1, 0, 0;
    %                     0, 0, 1;
    %                     0, -1, 0];
    % 
    % T23r = [cos(q3), -sin(q3), 0;
    %        sin(q3) , cos(q3), 0;
    %         0, 0, 1] * [1, 0, 0;
    %                     0, 0, 1;
    %                     0, -1, 0]; 
    % 
    % T23t = [0 0 d1]';
    % 
    % T34r = [cos(q4), -sin(q4), 0;
    %        sin(q4) , cos(q4), 0;
    %         0, 0, 1] * [1, 0, 0;
    %                     0, 0, -1;
    %                     0, 1, 0]; 
    % 
    % T45r = [cos(q5), -sin(q5), 0;
    %        sin(q5) , cos(q5), 0;
    %         0, 0, 1] * [1, 0, 0;
    %                     0, 0, -1;
    %                     0, 1, 0];
    % 
    % T45t = [0 0 d2]';
    % 
    % T56r = [cos(q6), -sin(q6), 0;
    %        sin(q6) , cos(q6), 0;
    %         0, 0, 1] * [1, 0, 0;
    %                     0, 0, 1;
    %                     0, -1, 0];
    % 
    % T67r = [cos(q7), -sin(q7), 0;
    %        sin(q7) , cos(q7), 0;
    %         0, 0, 1];
    % 
    % 
    % T01 = [T01r, [0;0;0]; [0,0,0], 1];
    % T12 = [T12r, [0;0;0]; [0,0,0], 1];
    % T23 = [T23r, T23t; [0,0,0], 1];
    % T34 = [T34r, [0;0;0]; [0,0,0], 1];
    % T45 = [T45r, T45t; [0,0,0], 1];
    % T56 = [T56r, [0;0;0]; [0,0,0], 1];
    % T67 = [T67r, [0;0;0]; [0,0,0], 1];
    % 
    % % T01*T12*T23*T34
    % % T45*T56*T67
    % 
    % T = simplify(T01*T12*T23*T34*T45*T56*T67)
    % 
    % 
    % d = T(1:3,4);
    % 
    % J = simplify(jacobian(d,[q1;q2;q3;q4;q5;q6;q7]))
    % old = [q1,q2,q3,q4,d1,d2]';
    % new = [q(1),q(2),q(3),q(4),0.42,0.40]';
    % J = subs(J,old,new)


    % J = [(2*cos(q1 + q2)*cos(q4))/5 - (21*cos(q1 + q2))/50 - (2*sin(q1 + q2)*cos(q3)*sin(q4))/5, (2*cos(q1 + q2)*cos(q4))/5 - (21*cos(q1 + q2))/50 - (2*sin(q1 + q2)*cos(q3)*sin(q4))/5, -(2*cos(q1 + q2)*sin(q3)*sin(q4))/5, (2*cos(q1 + q2)*cos(q3)*cos(q4))/5 - (2*sin(q1 + q2)*sin(q4))/5, 0, 0, 0;
    %     (2*sin(q1 + q2)*cos(q4))/5 - (21*sin(q1 + q2))/50 + (2*cos(q1 + q2)*cos(q3)*sin(q4))/5, (2*sin(q1 + q2)*cos(q4))/5 - (21*sin(q1 + q2))/50 + (2*cos(q1 + q2)*cos(q3)*sin(q4))/5, -(2*sin(q1 + q2)*sin(q3)*sin(q4))/5, (2*cos(q1 + q2)*sin(q4))/5 + (2*sin(q1 + q2)*cos(q3)*cos(q4))/5, 0, 0, 0;
    %                                                                                          0,                                                                                      0,              -(2*cos(q3)*sin(q4))/5,                                          -(2*cos(q4)*sin(q3))/5, 0, 0, 0]

    J = [(21*sin(q1)*sin(q2))/50 - (21*cos(q6)*(sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - cos(q4)*sin(q1)*sin(q2)))/100 + (21*sin(q6)*(cos(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + sin(q1)*sin(q2)*sin(q4)) + sin(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3))))/100 - (2*sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)))/5 + (2*cos(q4)*sin(q1)*sin(q2))/5, -(cos(q1)*(42*cos(q2) + 40*cos(q2)*cos(q4) + 21*cos(q2)*cos(q4)*cos(q6) + 40*cos(q3)*sin(q2)*sin(q4) + 21*cos(q3)*cos(q6)*sin(q2)*sin(q4) + 21*cos(q2)*cos(q5)*sin(q4)*sin(q6) + 21*sin(q2)*sin(q3)*sin(q5)*sin(q6) - 21*cos(q3)*cos(q4)*cos(q5)*sin(q2)*sin(q6)))/100, - (2*sin(q4)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3)))/5 - (21*sin(q6)*(sin(q5)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q4)*cos(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3))))/100 - (21*cos(q6)*sin(q4)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3)))/100, (2*cos(q1)*sin(q2)*sin(q4))/5 - (2*cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)))/5 - (21*cos(q5)*sin(q6)*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*cos(q4)*sin(q2)))/100 - (21*cos(q6)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*sin(q2)*sin(q4)))/100, -(21*sin(q6)*(sin(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*sin(q2)*sin(q4)) - cos(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3))))/100,   (21*sin(q6)*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*cos(q4)*sin(q2)))/100 + (21*cos(q6)*(cos(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*sin(q2)*sin(q4)) + sin(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3))))/100, 0;
        (21*sin(q6)*(cos(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*sin(q2)*sin(q4)) + sin(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3))))/100 - (21*cos(q6)*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*cos(q4)*sin(q2)))/100 - (21*cos(q1)*sin(q2))/50 - (2*sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)))/5 - (2*cos(q1)*cos(q4)*sin(q2))/5, -(sin(q1)*(42*cos(q2) + 40*cos(q2)*cos(q4) + 21*cos(q2)*cos(q4)*cos(q6) + 40*cos(q3)*sin(q2)*sin(q4) + 21*cos(q3)*cos(q6)*sin(q2)*sin(q4) + 21*cos(q2)*cos(q5)*sin(q4)*sin(q6) + 21*sin(q2)*sin(q3)*sin(q5)*sin(q6) - 21*cos(q3)*cos(q4)*cos(q5)*sin(q2)*sin(q6)))/100,   (2*sin(q4)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3)))/5 + (21*sin(q6)*(sin(q5)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - cos(q4)*cos(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3))))/100 + (21*cos(q6)*sin(q4)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3)))/100, (21*cos(q6)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + sin(q1)*sin(q2)*sin(q4)))/100 + (2*cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)))/5 + (2*sin(q1)*sin(q2)*sin(q4))/5 + (21*cos(q5)*sin(q6)*(sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - cos(q4)*sin(q1)*sin(q2)))/100,  (21*sin(q6)*(sin(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + sin(q1)*sin(q2)*sin(q4)) - cos(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3))))/100, - (21*sin(q6)*(sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - cos(q4)*sin(q1)*sin(q2)))/100 - (21*cos(q6)*(cos(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + sin(q1)*sin(q2)*sin(q4)) + sin(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3))))/100, 0;
                                                                                                                                                                                                                                                                                                                                                                                        0,                              (2*cos(q2)*cos(q3)*sin(q4))/5 - (2*cos(q4)*sin(q2))/5 - (21*sin(q6)*(cos(q5)*(sin(q2)*sin(q4) + cos(q2)*cos(q3)*cos(q4)) - cos(q2)*sin(q3)*sin(q5)))/100 - (21*cos(q6)*(cos(q4)*sin(q2) - cos(q2)*cos(q3)*sin(q4)))/100 - (21*sin(q2))/50,                                                                                                                                        -(sin(q2)*(40*sin(q3)*sin(q4) + 21*cos(q6)*sin(q3)*sin(q4) - 21*cos(q3)*sin(q5)*sin(q6) - 21*cos(q4)*cos(q5)*sin(q3)*sin(q6)))/100,                                                                                                             (21*cos(q5)*sin(q6)*(cos(q2)*cos(q4) + cos(q3)*sin(q2)*sin(q4)))/100 - (21*cos(q6)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)))/100 - (2*cos(q2)*sin(q4))/5 + (2*cos(q3)*cos(q4)*sin(q2))/5,                                                                 -(21*sin(q6)*(sin(q5)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) - cos(q5)*sin(q2)*sin(q3)))/100,                                                                                                       (21*cos(q6)*(cos(q5)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) + sin(q2)*sin(q3)*sin(q5)))/100 - (21*sin(q6)*(cos(q2)*cos(q4) + cos(q3)*sin(q2)*sin(q4)))/100, 0];
 
end

