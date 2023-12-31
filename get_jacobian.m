function J = get_jacobian(q)

q1 = q(1);
q2 = q(2);
q3 = q(3);
q4 = q(4);
q5 = q(5);
t2 = sin(q5);
t3 = q2+q3+q4;
t4 = cos(q5);
t5 = sin(q1);
t6 = cos(t3);
t7 = cos(q1);
t8 = sin(t3);
t9 = t8.*1.6e1;
t10 = q2+q3;
t11 = cos(t10);
t12 = t11.*7.3e1;
t13 = cos(q2);
t14 = t13.*6.3e1;
t15 = t9+t12+t14+5.0;
t16 = t2.*t5;
t17 = t4.*t6.*t7;
t18 = t16+t17;
t19 = t4.*t5;
t29 = t2.*t6.*t7;
t20 = t19-t29;
t21 = q3+q4;
t22 = sin(t21);
t23 = t22.*6.3e1;
t24 = sin(q4);
t25 = t24.*7.3e1;
t26 = t23+t25+1.6e1;
t27 = cos(q4);
t28 = t25+1.6e1;
t30 = cos(t21);
t31 = t30.*(6.3e1./2.0e2);
t32 = t27.*(7.3e1./2.0e2);
t33 = t31+t32;
t34 = t2.*t7;
t39 = t4.*t5.*t6;
t35 = t34-t39;
t36 = t4.*t7;
t37 = t2.*t5.*t6;
t38 = t36+t37;
t40 = t4.^2;
t41 = t2.^2;
t42 = t4.*t20;
t43 = t2.*t18;
t44 = t42+t43;
t46 = t4.*t38;
t47 = t2.*t35;
t45 = -t46-t47;
t48 = t8.^2;
J = reshape([t2.*t15.*t18.*(-1.0./2.0e2)-t4.*t15.*t20.*(1.0./2.0e2),t2.*t15.*t35.*(1.0./2.0e2)+t4.*t15.*t38.*(1.0./2.0e2),0.0,-t6.*t7.*t8-t2.*t8.*t20+t4.*t8.*t18,-t5.*t6.*t8-t4.*t8.*t35+t2.*t8.*t38,t40.*t48+t41.*t48+t6.^2,t2.*t20.*t26.*(-1.0./2.0e2)+t4.*t18.*t26.*(1.0./2.0e2)-t7.*t8.*t33,-t5.*t8.*t33-t4.*t26.*t35.*(1.0./2.0e2)+t2.*t26.*t38.*(1.0./2.0e2),t6.*t33+t8.*t26.*t40.*(1.0./2.0e2)+t8.*t26.*t41.*(1.0./2.0e2),t44,t45,0.0,t7.*t8.*t27.*(-7.3e1./2.0e2)-t2.*t20.*t28.*(1.0./2.0e2)+t4.*t18.*t28.*(1.0./2.0e2),t5.*t8.*t27.*(-7.3e1./2.0e2)-t4.*t28.*t35.*(1.0./2.0e2)+t2.*t28.*t38.*(1.0./2.0e2),t6.*t27.*(7.3e1./2.0e2)+t8.*t28.*t40.*(1.0./2.0e2)+t8.*t28.*t41.*(1.0./2.0e2),t44,t45,0.0,t2.*t20.*(-2.0./2.5e1)+t4.*t18.*(2.0./2.5e1),t4.*t35.*(-2.0./2.5e1)+t2.*t38.*(2.0./2.5e1),t8.*t40.*(2.0./2.5e1)+t8.*t41.*(2.0./2.5e1),t44,t45,0.0,0.0,0.0,0.0,t7.*t8,t5.*t8,-t6],[6, 5]);

end
