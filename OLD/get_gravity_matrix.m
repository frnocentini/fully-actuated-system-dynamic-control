function G = get_gravity_matrix(q)
    robotModel = loadrobot("kukaIiwa14", DataFormat = 'column', Gravity=[0; 0; -9.81]);
    G = gravityTorque(robotModel,q);
end

