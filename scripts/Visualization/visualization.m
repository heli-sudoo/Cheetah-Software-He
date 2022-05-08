clear
ctraj = load('../../build/state.txt');
mat_traj = importdata('/home/wensinglab/HL/Code/MHPC-project/MATLAB_untouched/Data/matalb_state.mat');
ctraj = ctraj';

G = Graphics();
G.addTrajectory(ctraj);
G.addTrajectory(mat_traj);
G.compareTrajectory([1,2]);
G.save_animation();