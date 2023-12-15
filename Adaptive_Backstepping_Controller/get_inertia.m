function M = get_inertia(robot, q)
    if numcols(q) ~= robot.n
        error('q must have %d columns', robot.n);
    end

    if numrows(q) > 1
        M = [];
        for i=1:numrows(q)
            M = cat(3, M, robot.inertia(q(i,:)));
        end
        return
    end

	n = robot.n;

	if numel(q) == robot.n
		q = q(:).';
	end

	M = zeros(n,n,0);
	for Q = q.'
		m = rne(robot, ones(n,1)*Q.', zeros(n,n), eye(n), 'gravity', [0 0 0]);
		M = cat(3, M, m);
    end

