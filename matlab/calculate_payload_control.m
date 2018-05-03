function [] = calculate_payload_control()

    %% Payload control

    %Parallel components of force integral
    Fd_integral_sum = zeros(3, 1);
    for ii = 1:n
        Fd_integral_sum = Fd_integral_sum + q(:,ii)*q(:,ii)'*Deltax(:,ii);
    end

    %Desired force
    Fd = m0 * (-kx0*ex0 - kx0_dot*ex0_dot + x0d_ddot - g*[0 0 1]') ...
        - Deltax0 - Fd_integral_sum;

    Md_integral_sum = zeros(3,1);
    for ii = 1:n
        Md_integral_sum = Md_integral_sum + ...
            hat(rho(:,ii))*R0*q(:,ii)*q(:,ii)'*Deltax(:,ii);
    end

    %Desired Moment
    Md = -kR0*eR0 - kOmega0*eOmega0 + hat(R0'*R0d*Omega0d) * J0*R0'*R0d*Omega0d + ...
        J0*R0'*R0d*Omega0d_dot - DeltaR0 - Md_integral_sum;

    %% Individual components from control

    %Diagonal matrix of R0
    diag = kron(R0, eye(n));
    psuedo_P =  P' * inv(P*P');

    %desired virtual controls, 3nx1
    mud = diag * psuedo_P * [R0'*Fd; Md];

    %desired cable directions, 3nx1
    qd = sym('qd', [3, n]);

    %virtual control input
    mu = sym('mu', [3,n]);
    for ii = 1:n
        mu(:,ii) = q(:,ii)*q(:,ii)'*(mud(3*(ii-1)+(1:3)));
        qd(:,ii) = -(mud(3*(ii-1)+(1:3))) / norm((mud(3*(ii-1)+(1:3))));
    end

end

