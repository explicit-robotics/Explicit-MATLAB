function p = slcamera(cam, u)
    if all(u == 0)
        % Simulink is probing for the dimensions
        np = (length(u)-16)/3;
        p = zeros(2, np);
    else
        P = reshape(u(17:end), 3, []);
        Tcam = reshape(u(1:16), 4, 4);
        p = cam.plot(P, 'pose', SE3(Tcam), 'drawnow');
    end