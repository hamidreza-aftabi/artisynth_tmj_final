import os
import pymeshlab

# Define source and destination directories
sourceDir = 'C:\\Users\\Hamidreza\\git\\artisynth_istar\\src\\artisynth\\istar\\reconstruction\\optimizationResultTwo'
destinationDir = 'C:\\Users\\Hamidreza\\git\\artisynth_tmj_final\\artisynth_TMJ_Final\\src\\artisynth\\istar\\TMJModel\\JawTMJ\\geometry'

# List of files to be remeshed with specific target lengths
file_list = {
    'donor_opt0.obj': 1.452,
    'donor_opt1.obj': 1.452,
    'resected_mandible_l_opt.obj': 0.449,
    'resected_mandible_r_opt.obj': 0.687,
    'screw_opt0.obj': 5.479,
    'screw_opt1.obj': 5.479
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
        target_length = pymeshlab.PercentageValue(target_length)

        # Remesh the current mesh
        ms.meshing_isotropic_explicit_remeshing(adaptive = True, iterations=20, targetlen=target_length)
        ms.meshing_merge_close_vertices()
        ms.meshing_snap_mismatched_borders()
        
        # Save the remeshed mesh
        ms.save_current_mesh(output_mesh_path)
        print(f"Remeshed file has been saved to {output_mesh_path}")
    
    except Exception as e:
        print(f"An error occurred with file {file_name}: {e}")

    finally:
        # Ensure the MeshSet object is deleted to free resources
        del ms
        print(f"Cleaned up resources for {file_name}")