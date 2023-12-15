function C = C_Matrix_Sym(M,q,dq)
    sympref('FloatingPointOutput',true);

    C = sym('C',[7,7]);
    Gamma = sym('Gamma',[7,7,7]); % index order: j,k,i

    num_joints = 7;

    for i=1:num_joints
        for j=1:num_joints
            for k=1:num_joints
                Gamma(j,k,i) = 1/2 * (jacobian(M(i,j),q(k)) + jacobian(M(i,k),q(j)) - jacobian(M(j,k),q(i)));
                
            end
        end

        C(i,:) = dq*Gamma(:,:,i);
    end



