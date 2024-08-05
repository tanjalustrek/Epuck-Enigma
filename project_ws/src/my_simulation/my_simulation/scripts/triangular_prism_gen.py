import os
import random
import xml.etree.ElementTree as ET

def generate_triangular_prism(num_files=1):
    '''Generates random .sdf files for triangular prisms'''
    # Define the height scale, side length scale range for the equilateral triangle and orientation range around z axis
    height = 0.025
    side_length_range = (0.02, 0.04)
    orientation_range = (-3.14, 3.14)

    # Get mesh path
    mesh_path = os.path.join(os.path.abspath('src'), 'my_simulation', 'my_simulation', 'meshes', 'triangular_prism', 'triangular_prism.stl')

    # Create the desired number of files
    for i in range(num_files):
        # Generate random side length scale value and orientation
        side_length = random.uniform(side_length_range[0], side_length_range[1])
        orientation_z = random.uniform(orientation_range[0], orientation_range[1])
        orientation = [0, 0, orientation_z]

        # Create the .sdf file
        root = ET.Element('sdf', version='1.6')
        model = ET.SubElement(root, 'model', name=f'model_{i+1}')

        # Set static to true
        ET.SubElement(model, 'static').text = 'true'

        # Add pose
        link = ET.SubElement(model, 'link', name='link')
        ET.SubElement(link, 'pose').text = ' '.join(map(str, [0, 0, 0] + orientation))

        # Add collision
        collision = ET.SubElement(link, 'collision', name='collision')
        geometry = ET.SubElement(collision, 'geometry')
        mesh = ET.SubElement(geometry, 'mesh')
        ET.SubElement(mesh, 'uri').text = mesh_path
        ET.SubElement(mesh, 'scale').text = f'{side_length} {side_length} {height}'

        # Add visual
        visual = ET.SubElement(link, 'visual', name='visual')
        geometry = ET.SubElement(visual, 'geometry')
        mesh = ET.SubElement(geometry, 'mesh')
        ET.SubElement(mesh, 'uri').text = mesh_path
        ET.SubElement(mesh, 'scale').text = f'{side_length} {side_length} {height}'
        material = ET.SubElement(visual, 'material')
        script = ET.SubElement(material, 'script')
        ET.SubElement(script, 'name').text = 'Gazebo/Red'
        ET.SubElement(script, 'uri').text = 'file://media/materials/scripts/gazebo.material'

        # Save the .sdf file
        output_dir = os.path.join(os.path.abspath('src'), 'my_simulation', 'my_simulation', 'models', 'triangular_prism')
        os.makedirs(output_dir, exist_ok=True)
        output_file = os.path.join(output_dir, f'model_{i+1}.sdf')
        tree = ET.ElementTree(root)
        tree.write(output_file)

def get_side_length(sdf_file_path):
    '''Gets the side length of the triangular prism'''
    tree = ET.parse(sdf_file_path)
    root = tree.getroot()
    scale_element = root.find(".//mesh/scale")
    scale_values = scale_element.text.split()
    side_length = float(scale_values[0]) * 4
    return side_length