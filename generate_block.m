function [MPC_step] = generate_block( N, nx, nu, ny, nc , max_iter, constr_viol_tol, dual_inf_tol, verbose)

import casadi.*

%%%
% Declare parameters
%%%

% System dynamics
A = MX.sym('A',nx,nx);
B = MX.sym('B',nx,nu);
C = MX.sym('C',ny,nx);
D = MX.sym('D',ny,nu);

% Weights
Q = MX.sym('Q',nx,nx);
R = MX.sym('R',nu,nu);

% Intial state constraint
x0 = MX.sym('x0',nx);

%Simple bounds
lb_x = MX.sym('lb_x',nx);
ub_x = MX.sym('ub_x',nx);
lb_u = MX.sym('lb_u',nu);
ub_u = MX.sym('ub_u',nu);


% Linear constraints
H_x = MX.sym('H_x',nc,nx);
H_u = MX.sym('H_u',nc,nu);
lb_h = MX.sym('lb_h',nc);
ub_h = MX.sym('ub_h',nc);


p = {A,B,C,D,Q,R,x0,lb_x,ub_x,lb_u,ub_u,H_x,H_u,lb_h,ub_h};

%%%
% Build objective and constraints expression
%
% min  f(x,p)
%  x
%
% s.t.   lbg <= g(x,p) <= ubg
%        lbx <= x <= ubx
%%%

% State for k=0
x = MX.sym('x',nx);

% Objective (will be added to)
f = x'*Q*x;

% Decision variables
w = {x};

% Bounds on decision variables
lbx = {x0};
ubx = {x0};

% Constraints
g = {};
% Bounds on constraints
lbg = [];
ubg = [];


for k=1:N
  u = MX.sym('u',nu); % Control for k
  w{end+1} = u;
  lbx{end+1} = lb_u;
  ubx{end+1} = ub_u;
  f = f + u'*R*u;

  x_plus = MX.sym('x',nx); % State for k+1
  w{end+1} = x_plus;
  lbx{end+1} = lb_x;
  ubx{end+1} = ub_x;
  f = f + x'*Q*x;

  % Dynamic constraints
  g{end+1} = A*x+B*u-x_plus;
  lbg{end+1} = zeros(nx,1);
  ubg{end+1} = zeros(nx,1);

  % path constraints
  g{end+1} = H_x*x+H_u*u;
  lbg{end+1} = lb_h;
  ubg{end+1} = ub_h;
end

% Make tall flattened vectors
g = vertcat(g{:});
lbg = vertcat(lbg{:});
ubg = vertcat(ubg{:});
w = vertcat(w{:});
lbx = vertcat(lbx{:});
ubx = vertcat(ubx{:});


%%%
% Construct QP solver
%%%

qp_struct = struct;
qp_struct.f = f;
qp_struct.g = g;
qp_struct.x = w;
qp_struct.p = veccat(p{:});
options = struct;
options.max_iter = max_iter;
options.constr_viol_tol = constr_viol_tol;
options.dual_inf_tol = dual_inf_tol;
options.print_iter = verbose;
options.print_time = verbose;
options.print_info = verbose;
options.print_header = verbose;
solver = qpsol('solver','qrqp',qp_struct,options);

%%%
% Call QP solver symbolically
%%%

x0 = MX.sym('x_initial',size(w,1),1);
lam_x0 = MX.sym('lam_x_initial',size(w,1),1);
lam_g0 = MX.sym('lam_g_initial',size(g,1),1);

arg_struct = struct;
arg_struct.x0 = x0;
arg_struct.lam_x0 = lam_x0;
arg_struct.lam_g0 = lam_g0;
arg_struct.lbx = lbx;
arg_struct.ubx = ubx;
arg_struct.lbg = lbg;
arg_struct.ubg = ubg;
arg_struct.p = qp_struct.p;
res_struct = solver.call(arg_struct);


xsol = res_struct.x;
out = {xsol(nx+1:nx+nu),res_struct.f,res_struct.x,res_struct.lam_x,res_struct.lam_g};
out_names = {'u_next','f','x','lam_x','lam_g'};


p = {p{:} x0 lam_x0 lam_g0};

p_names = cell(numel(p),1);
for i=1:numel(p)
  p_names{i} = name(p{i});
end

%%%
% Create a Function out of it
%%%


MPC_step = Function('MPC_step',p,out,p_names,out_names);

%%%
% Generate code
%%%

cg_options = struct;
cg_options.casadi_real = 'real_T';
cg_options.casadi_int = 'int_T';
cg_options.with_header = true;
cg_options.real_min = num2str(realmin('double'));
if verbose
    cg_options.verbose_runtime = true;
end
cg = CodeGenerator('MPC_step.c',cg_options);
cg.add_include('simstruc.h');
cg.add(MPC_step);

cg.generate();

mex -DCASADI_PRINTF=ssPrintf LMPC.c MPC_step.c

end

