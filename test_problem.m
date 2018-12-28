function arg_struct = test_problem( N, nx, nu, ny, nc )

rng(1)
arg_struct = struct;
arg_struct.A = rand(nx,nx);
arg_struct.B = rand(nx,nu);
arg_struct.C = rand(ny,nx);
arg_struct.D = rand(ny,nu);
arg_struct.Q = eye(nx);
arg_struct.R = eye(nu);
arg_struct.x0 = ones(nx,1);

arg_struct.lb_x = -10*ones(nx,1);
arg_struct.ub_x = 10*ones(nx,1);

arg_struct.lb_u = -2*ones(nu,1);
arg_struct.ub_u = 2*ones(nu,1);

arg_struct.H_x = rand(nc,nx);
arg_struct.H_u = rand(nc,nu);

arg_struct.lb_h = -10*ones(nc,1);
arg_struct.ub_h = 10*ones(nc,1);


arg_struct.x_initial = zeros(N*nu+(N+1)*nx,1);
arg_struct.lam_x_initial = zeros(N*nu+(N+1)*nx,1);
arg_struct.lam_g_initial = zeros(N*nx+nc*N,1);

end

