clc
clear all
close all
addpath('C:\Users\Hamidreza\git\artisynth_core\matlab');
setArtisynthClasspath (getenv ('ARTISYNTH_HOME'));

for i=1:5
     runArtisynthSim(-4)
end
