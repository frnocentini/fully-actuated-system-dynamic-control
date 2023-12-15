function Irow = inertia_row_7(rob,in2)
%% INERTIA_ROW_7 - Computation of the robot specific inertia matrix row for corresponding to joint 7 of 7. 
% ========================================================================= 
%    
%    Irow = inertia_row_7(rob,q) 
%    Irow = rob.inertia_row_7(q) 
%    
%  Description:: 
%    Given a full set of joint variables this function computes the 
%    inertia matrix row number 7 of 7 for PANDA. 
%    
%  Input:: 
%    rob: robot object of PANDA specific class 
%    q:  7-element vector of generalized 
%         coordinates 
%    Angles have to be given in radians! 
%    
%  Output:: 
%    Irow:  [1x7] row of the robot inertia matrix 
%    
%  Example:: 
%    --- 
%    
%  Known Bugs:: 
%    --- 
%    
%  TODO:: 
%    --- 
%    
%  References:: 
%    1) Robot Modeling and Control - Spong, Hutchinson, Vidyasagar 
%    2) Modelling and Control of Robot Manipulators - Sciavicco, Siciliano 
%    3) Introduction to Robotics, Mechanics and Control - Craig 
%    4) Modeling, Identification & Control of Robots - Khalil & Dombre 
%    
%  Authors:: 
%    This is an autogenerated function! 
%    Code generator written by: 
%    Joern Malzahn 
%    2012 RST, Technische Universitaet Dortmund, Germany 
%    http://www.rst.e-technik.tu-dortmund.de 
%    
%  See also coriolis.
%    
    
% Copyright (C) 1993-2023, by Peter I. Corke 
% Copyright (C) 2012-2023, by Joern Malzahn 
% 
% This file has been automatically generated with The Robotics Toolbox for Matlab (RTB). 
% 
% RTB and code generated with RTB is free software: you can redistribute it and/or modify 
% it under the terms of the GNU Lesser General Public License as published by 
% the Free Software Foundation, either version 3 of the License, or 
% (at your option) any later version. 
%  
% RTB is distributed in the hope that it will be useful, 
% but WITHOUT ANY WARRANTY; without even the implied warranty of 
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the 
% GNU Lesser General Public License for more details. 
%  
% You should have received a copy of the GNU Leser General Public License 
% along with RTB.  If not, see <http://www.gnu.org/licenses/>. 
% 
% http://www.petercorke.com 
% 
% The code generation module emerged during the work on a project funded by 
% the German Research Foundation (DFG, BE1569/7-1). The authors gratefully  
% acknowledge the financial support. 

%% Bugfix
%  In some versions the symbolic toolbox writes the constant $pi$ in
%  capital letters. This way autogenerated functions might not work properly.
%  To fix this issue a local variable is introduced:
PI = pi;
   




%    This function was generated by the Symbolic Math Toolbox version 23.2.
%    14-Dec-2023 10:10:21
sympref('FloatingPointOutput',true)
syms m6 m7
syms r6x r6y r6z
syms r7x r7y r7z

q2 = in2(:,2);
q3 = in2(:,3);
q4 = in2(:,4);
q5 = in2(:,5);
q6 = in2(:,6);
q7 = in2(:,7);

t2 = cos(q3);
t3 = cos(q4);
t4 = cos(q5);
t5 = cos(q6);
t6 = cos(q7);
t7 = sin(q3);
t8 = sin(q4);
t9 = sin(q5);
t10 = sin(q6);
t11 = sin(q7);
t12 = r7x.^2;
t13 = r7y.^2;
t14 = m7.*r7x.*t6;
t15 = m7.*r7y.*t6;
t16 = m7.*r7x.*t11;
t17 = m7.*r7y.*t11;
t18 = m7.*t12;
t19 = m7.*t13;
t20 = r7z.*t14;
t21 = r7z.*t15;
t22 = r7z.*t16;
t23 = r7z.*t17;
t24 = -t17;
t26 = t14.*(1.1e+1./1.25e+2);
t27 = t17.*(1.1e+1./1.25e+2);
t28 = t15+t16;
t25 = -t23;
t29 = -t27;
t30 = t21+t22;
t31 = t14+t24;
t33 = t4.*t5.*t28;
t35 = t5.*t9.*t28;
t37 = t8.*t10.*t28;
t45 = t10.*t28.*(3.3e+1./4.0e+2);
t32 = t20+t25;
t34 = t4.*t30;
t36 = t9.*t30;
t38 = t4.*t31;
t39 = t9.*t31;
t46 = -t45;
t47 = t33.*(4.8e+1./1.25e+2);
t48 = t35.*(4.8e+1./1.25e+2);
t49 = t3.*t45;
t50 = t35.*(3.3e+1./4.0e+2);
t63 = t18+t19+t26+t29+5.281249999999999e-4;
t41 = t5.*t32;
t42 = -t36;
t43 = t10.*t32;
t51 = t38.*(4.8e+1./1.25e+2);
t52 = t38.*(3.3e+1./4.0e+2);
t53 = t39.*(4.8e+1./1.25e+2);
t54 = -t47;
t55 = -t50;
t57 = t35+t38;
t59 = -t3.*(t33-t39);
t61 = t8.*(t33-t39).*(3.3e+1./4.0e+2);
t64 = t5.*t63;
t65 = t10.*t63;
t56 = -t52;
t62 = t37+t59;
t67 = -t4.*(t41-t65);
t68 = -t9.*(t41-t65);
t69 = t43+t55+t56+t64;
t72 = t42+t48+t51+t67;
t75 = t34+t46+t49+t53+t54+t61+t68;
t70 = t3.*t69;
t71 = t8.*t69;
t73 = t3.*t72;
t74 = t8.*t72;
t76 = t71+t73;
Irow = [-sin(q2).*(t2.*t57.*(7.9e+1./2.5e+2)-t7.*t62.*(7.9e+1./2.5e+2)+t2.*t76-t7.*t75)-cos(q2).*(t50+t52+t70-t74),t2.*t62.*(7.9e+1./2.5e+2)+t7.*t57.*(7.9e+1./2.5e+2)+t2.*t75+t7.*t76,t55+t56-t70+t74,-t34+t45+t47-t53+t9.*(t41-t65),-t43-t64,-t30,t18+t19+5.281249999999999e-4];
end
