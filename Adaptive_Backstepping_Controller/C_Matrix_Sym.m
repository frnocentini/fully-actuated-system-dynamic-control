function C = C_Matrix_Sym(M,q,dq)
    sympref('FloatingPointOutput',true);

    % C = sym('C',[5,5]);
    C = zeros(5,5);
    C = symmatrix(C);

    Gamma = sym('Gamma',[5,5,5]); % index order: i,j,k

    num_joints = 5;

    for k=1:num_joints
        for j=1:num_joints
            for i=1:num_joints
                Gamma(i,j,k) = 1/2 * (jacobian(M(i,j),q(k)) + jacobian(M(i,k),q(j)) - jacobian(M(j,k),q(i)));
                C(i,j) = C(i,j) + Gamma(i,j,k)*dq(k);
            end
        end

        % C(i,:) = dq*Gamma(:,:,i);
    end

    