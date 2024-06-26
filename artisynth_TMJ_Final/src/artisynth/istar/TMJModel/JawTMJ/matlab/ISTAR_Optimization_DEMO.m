%import maspack.matrix.*

addpath('C:\Users\Hamidreza\git\artisynth_core\matlab');
setArtisynthClasspath (getenv ('ARTISYNTH_HOME'));

% parameters to optimize
zOffset = -4;

ah = artisynth('-model', 'artisynth.istar.reconstruction.MandibleRecon');
root=ah.root();
root.createFibulaOptimization(zOffset);

for i=1:100
    ah.step();
end

pause(5);

root.getPlatePanel.createScrews();
root.exportFiles();
root.exportFemPlate();

sourceDir = 'C:\Users\Hamidreza\git\artisynth_istar\src\artisynth\istar\reconstruction\optimizationResult';
destinationDir = 'C:\Users\Hamidreza\git\artisynth_tmj_final\artisynth_TMJ_Final\src\artisynth\istar\TMJModel\JawTMJ\geometry';

fileList = {'donor_opt.obj', 'plate_opt.art', 'plate_opt.obj', 'resected_mandible_l_opt.obj', 'resected_mandible_r_opt.obj', 'screw_opt.obj'};

for i = 1:length(fileList)
    sourceFile = fullfile(sourceDir, fileList{i});
    copyfile(sourceFile, destinationDir);
end

ah.quit()

pyrunfile('PymeshlabRemesh.py')

ah = artisynth('-model', 'artisynth.istar.TMJModel.JawTMJ.JawFemDemoOptimize');

for i=1:1000
    ah.step();
end

left_percent=ah.getOprobeData ('6');
right_percent=ah.getOprobeData ('7');

loss = - (mean(left_percent(:,2))+mean(right_percent(:,2)))/(mean(left_percent(:,2))-mean(right_percent(:,2))+0.0000001);

ah.quit()




