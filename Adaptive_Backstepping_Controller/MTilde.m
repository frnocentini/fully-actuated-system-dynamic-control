function M_sym = MTilde(in1)
%MTilde
%    M_sym = MTilde(IN1)

%    This function was generated by the Symbolic Math Toolbox version 23.2.
%    17-Dec-2023 18:07:44

m4 = in1(:,4);
m5 = in1(:,5);
q2 = in1(:,1);
q3 = in1(:,2);
q4 = in1(:,3);
t2 = cos(q2);
t3 = cos(q3);
t4 = cos(q4);
t5 = sin(q2);
t6 = sin(q3);
t7 = sin(q4);
t11 = m5.*2.133333333333333e-3;
t14 = m4.*1.33225e-1;
t26 = m5.*1.353583333333333e-1;
t8 = t2.^2;
t9 = t3.^2;
t10 = t4.^2;
t12 = m5.*t7.*2.92e-2;
t13 = m5.*t7.*1.46e-2;
t15 = m5.*t3.*t7.*1.26e-2;
t16 = m5.*t4.*t6.*1.26e-2;
t17 = t3.*5.74875e-2;
t18 = (m5.*t2.*t3.*t4)./5.0e+3;
t19 = (m5.*t2.*t6.*t7)./5.0e+3;
t20 = (m5.*t3.*t5.*t7)./5.0e+3;
t21 = (m5.*t4.*t5.*t6)./5.0e+3;
t22 = m4.*t3.*1.14975e-1;
t23 = m5.*t3.*1.14975e-1;
t24 = -t18;
t25 = t11+t13;
t29 = t12+t14+t15+t16+t17+t22+t23+t26+4.440833333333333e-2;
t27 = t15+t16+t25;
t28 = t19+t20+t21+t24;
et1 = m4.*1.3385e-1+m5.*1.359833333333333e-1+t2.*2.3625e-2+t8.*8.789166666666667e-2-t9.*4.420833333333333e-2+t12+m4.*t2.*1.575e-2+m5.*t2.*1.575e-2-m4.*t8.*(1.7e+1./5.0e+2)-m4.*t9.*1.33225e-1-m5.*t8.*3.593333333333333e-2-m5.*t9.*1.351583333333333e-1-m5.*t10.*1.933333333333333e-3+t2.*t3.*9.125e-3+t3.*t8.*1.14975e-1-t5.*t6.*9.125e-3+t8.*t9.*8.841666666666667e-2;
et2 = m4.*t2.*t3.*1.825e-2+m5.*t2.*t3.*1.825e-2+m4.*t3.*t8.*2.2995e-1-m4.*t5.*t6.*1.825e-2+m5.*t3.*t8.*2.2995e-1-m5.*t5.*t6.*1.825e-2-m5.*t7.*t8.*2.92e-2+m4.*t8.*t9.*2.6645e-1-m5.*t7.*t9.*2.92e-2+m5.*t8.*t9.*2.703166666666667e-1+m5.*t8.*t10.*3.866666666666667e-3+m5.*t9.*t10.*3.866666666666667e-3-t2.*t5.*t6.*1.14975e-1-m5.*t2.*t4.*t5.*2.92e-2-m4.*t2.*t5.*t6.*2.2995e-1;
et3 = (m5.*t2.*t3.*t7)./5.0e+2+(m5.*t2.*t4.*t6)./5.0e+2+(m5.*t3.*t4.*t5)./5.0e+2-m5.*t2.*t5.*t6.*2.2995e-1-m5.*t3.*t4.*t6.*2.92e-2+m5.*t3.*t7.*t8.*2.52e-2+m5.*t4.*t6.*t8.*2.52e-2-(m5.*t5.*t6.*t7)./5.0e+2+m5.*t7.*t8.*t9.*5.84e-2-m5.*t8.*t9.*t10.*7.733333333333333e-3-t2.*t3.*t5.*t6.*8.841666666666667e-2+m5.*t2.*t3.*t4.*t5.*2.52e-2-m4.*t2.*t3.*t5.*t6.*2.6645e-1-m5.*t2.*t3.*t5.*t6.*2.703166666666667e-1-m5.*t2.*t4.*t5.*t7.*3.866666666666667e-3;
et4 = m5.*t2.*t4.*t5.*t9.*5.84e-2-m5.*t2.*t5.*t6.*t7.*2.52e-2-m5.*t3.*t4.*t6.*t7.*3.866666666666667e-3+m5.*t3.*t4.*t6.*t8.*5.84e-2-m5.*t2.*t3.*t5.*t6.*t7.*5.84e-2+m5.*t2.*t3.*t5.*t6.*t10.*7.733333333333333e-3+m5.*t2.*t4.*t5.*t7.*t9.*7.733333333333333e-3+m5.*t3.*t4.*t6.*t7.*t8.*7.733333333333333e-3+4.605833333333334e-2;
M_sym = reshape([et1+et2+et3+et4,0.0,0.0,0.0,t28,0.0,m4.*2.3245e-1+m5.*2.345833333333333e-1+t3.*1.14975e-1+t12+m4.*t3.*2.2995e-1+m5.*t3.*2.2995e-1+m5.*t3.*t7.*2.52e-2+m5.*t4.*t6.*2.52e-2+1.767083333333333e-1,t29,t27,0.0,0.0,t29,t12+t14+t26+4.440833333333333e-2,t25,0.0,0.0,t27,t25,t11,0.0,t28,0.0,0.0,0.0,m5./5.0e+3],[5,5]);
end
