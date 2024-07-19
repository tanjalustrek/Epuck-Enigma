import os
import random
import xml.etree.ElementTree as ET

# Generates random .sdf files for triangular prisms
def generate_triangular_prism(num_files=1):
    # Define the height scale, side length scale range for the equilateral triangle and orientation range around z axis
    height = 0.125
    side_length_range = (0.03, 0.13)
    orientation_range = (-3.14, 3.14)

    for i in range(num_files):
        # Generate random side length value and orientation
        side_length = random.uniform(side_length_range[0], side_length_range[1])
        orientation_z = random.uniform(orientation_range[0], orientation_range[1])
        orientation = [0, 0, orientation_z]

        # Create the .sdf file
        root = ET.Element('sdf', version='1.6')
        model = ET.SubElement(root, 'model', name='model')
        link = ET.SubElement(model, 'link', name='link')
        ET.SubElement(link, 'pose').text = ' '.join(map(str, [0, 0, 0] + orientation))

        # Add gravity
        ET.SubElement(link, 'gravity').text = 'true'

        # Make the prism static
        ET.SubElement(link, 'static').text = 'true'

        inertial = ET.SubElement(link, 'inertial')
        ET.SubElement(inertial, 'mass').text = '1.0'
        inertia = ET.SubElement(inertial, 'inertia')
        ET.SubElement(inertia, 'ixx').text = '0.25'
        ET.SubElement(inertia, 'ixy').text = '0'
        ET.SubElement(inertia, 'ixz').text = '0'
        ET.SubElement(inertia, 'iyy').text = '0.25'
        ET.SubElement(inertia, 'iyz').text = '0'
        ET.SubElement(inertia, 'izz').text = '0.5'

        collision = ET.SubElement(link, 'collision', name='collision')
        geometry = ET.SubElement(collision, 'geometry')
        mesh = ET.SubElement(geometry, 'mesh')
        ET.SubElement(mesh, 'uri').text = 'model://triangular_prism/meshes/triangular_prism.stl'
        ET.SubElement(mesh, 'scale').text = f'{side_length} {side_length} {height}'

        visual = ET.SubElement(link, 'visual', name='visual')
        geometry = ET.SubElement(visual, 'geometry')
        mesh = ET.SubElement(geometry, 'mesh')
        ET.SubElement(mesh, 'uri').text = 'model://triangular_prism/meshes/triangular_prism.stl'
        ET.SubElement(mesh, 'scale').text = f'{side_length} {side_length} {height}'
        material = ET.SubElement(visual, 'material')
        script = ET.SubElement(material, 'script')
        ET.SubElement(script, 'name').text = 'Gazebo/Red'
        ET.SubElement(script, 'uri').text = 'file://media/materials/scripts/gazebo.material'

        # Save the .sdf file
        output_dir = '/home/romi-lab-2/project_ws/src/my_simulation/my_simulation/models/triangular_prism'
        os.makedirs(output_dir, exist_ok=True)
        output_file = os.path.join(output_dir, f'model_{i+1}.sdf')
        tree = ET.ElementTree(root)
        tree.write(output_file)
        print(f'SDF file saved as {output_file}')

# Gets the side length of the triangular prism
def get_side_length_scale(sdf_file_path):
    tree = ET.parse(sdf_file_path)
    root = tree.getroot()
    
    # Find the scale element within the mesh element
    scale_element = root.find(".//mesh/scale")
    if scale_element is None:
        raise ValueError("The SDF file does not contain a valid triangular prism definition.")
    
    # Extract the side length from the scale attribute
    scale_values = scale_element.text.split()
    if len(scale_values) < 3:
        raise ValueError("The scale attribute does not contain enough values.")
    
    side_scale = float(scale_values[0])
    return side_scale