% Requires https://github.com/casadi/binaries/releases/tag/commit-fe4be0b
addpath('/home/jgillis/programs/casadi/matlab_install/casadi');

open_system('demo')

N = 5; % Horizon
nx = 5; % Number of states
nu = 2; % Number of controls
ny = 1; % Number of outputs
nc = 1; % Number of path constraints

max_iter = 1000;
constr_viol_tol = 1e-8;
dual_inf_tol = 1e-8;

verbose = true;

[MPC_step] = generate_block( N, nx, nu, ny, nc , max_iter, constr_viol_tol, dual_inf_tol, verbose);

% Create random test problem
prob = test_problem(N, nx, nu, ny, nc);

disp('Test random problem (QRQP without codegen)')
res_struct = MPC_step.call(prob);
result_casadi = full(res_struct.x);

disp('Run simulation with codegened QRQP')
simOut = sim('demo','SaveOutput','on');

result_simulink = simOut.get('simout').Data(:,1);

% Assert results are same
assert(norm(result_casadi-result_simulink,'inf')<1e-12);