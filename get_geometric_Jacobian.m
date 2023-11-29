function J = get_geometric_Jacobian(q)
robotModel = loadrobot("kukaIiwa14", DataFormat = 'column', Gravity=[0; 0; -9.81]);
J = geometricJacobian(robotModel,q,"iiwa_link_ee");
end

