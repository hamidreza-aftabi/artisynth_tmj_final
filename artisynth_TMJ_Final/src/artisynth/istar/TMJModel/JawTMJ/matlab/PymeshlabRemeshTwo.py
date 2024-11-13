import os
import pymeshlab
from pathlib import Path


# Define source and destination directories
#sourceDir = 'C:\\Users\\Hamidreza\\git\\artisynth_istar\\src\\artisynth\\istar\\reconstruction\\optimizationResultTwo'
#destinationDir = 'C:\\Users\\Hamidreza\\git\\artisynth_tmj_final\\artisynth_TMJ_Final\\src\\artisynth\\istar\\TMJModel\\JawTMJ\\geometry'


# Get the current directory of the Python script
current_path = Path().resolve()

# Navigate back 8 directories
target_path = current_path

for _ in range(8):
    target_path = target_path.parent

# Navigate into the 'artisynth_istar/src/artisynth/istar/reconstruction/optimizationResult' directory
sourceDir = target_path / 'artisynth_istar' / 'src' / 'artisynth' / 'istar' / 'reconstruction' / 'optimizationResultTwo'

destinationDir = target_path / 'artisynth_tmj_final' / 'artisynth_TMJ_Final' / 'src' / 'artisynth' / 'istar' / 'TMJModel' / 'JawTMJ' / 'geometry'




# List of files to be remeshed with specific target lengths
file_list = {
    'donor_opt0.obj': .55,
    'donor_opt1.obj': .55,
    'resected_mandible_l_opt.obj': .55,
    'resected_mandible_r_opt.obj': .55,
    'screw_opt0.obj': .55,
    'screw_opt1.obj': .55
}

for file_name, target_length in file_list.items():
    input_mesh_path = os.path.join(sourceDir, file_name)
    output_mesh_name = os.path.splitext(file_name)[0] + '_remeshed' + os.path.splitext(file_name)[1]
    output_mesh_path = os.path.join(destinationDir, output_mesh_name)
    
    try:
        # Create a MeshSet object
        ms = pymeshlab.MeshSet()
        
        # Load the mesh
        ms.load_new_mesh(input_mesh_path)
        
        # Set the specific target length percentage for the current file
        target_length = pymeshlab.PureValue(target_length)

        # Remesh the current mesh
        ms.meshing_isotropic_explicit_remeshing(iterations = 40, targetlen = target_length, featuredeg = 20.000000, checksurfdist = True, maxsurfdist = target_length, splitflag = True, collapseflag = True, swapflag = True, smoothflag = True, reprojectflag = True)       
        ms.meshing_merge_close_vertices()
        ms.meshing_snap_mismatched_borders()
        ms.meshing_remove_duplicate_faces()
        ms.meshing_repair_non_manifold_edges()
        ms.meshing_repair_non_manifold_vertices()
        ms.meshing_close_holes(maxholesize = 30, newfaceselected = True, selfintersection = True, refinehole = True, refineholeedgelen = pymeshlab.PercentageValue(2.906027))

        # Save the remeshed mesh
        ms.save_current_mesh(output_mesh_path)
        print(f"Remeshed file has been saved to {output_mesh_path}")
    
    except Exception as e:
        print(f"An error occurred with file {file_name}: {e}")

    finally:
        # Ensure the MeshSet object is deleted to free resources
        del ms
        print(f"Cleaned up resources for {file_name}")
