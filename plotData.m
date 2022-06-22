%% Initialization
clear ; close all; clc
fprintf('Plotting Data ...\n')
data = load('evolution_prey.dat');

X = data(:, 1); y = data(:, 2);
% Plot Data
figure;
plot(X, y);
xlabel('GENERATIONS');
ylabel('FITNESS');
title('PREY STATISTICS');