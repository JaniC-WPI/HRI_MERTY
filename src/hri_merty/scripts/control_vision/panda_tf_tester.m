rosshutdown
rosinit
clc; clear; close all

tftree = rostf;
pause(2);

syms q q0 q1 q2 q3 q4 q5 q6
q = [q0; q1; q2; q3; q4; q5; q6]; % arrange joints in a column vector

[phtm] = getPandaHTM(q,tftree);
