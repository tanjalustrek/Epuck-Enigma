import os
import random
import xml.etree.ElementTree as ET

def generate_cuboid():
    # Define the size and orientation ranges
    size_range = [(0.1, 0.5), (0.1, 0.5), (0.1, 0.5)]
    orientation_range = (-3.14, 3.14)

    # Generate random size and random rotation around the z-axis
    size = [random.uniform(min_size, max_size) for min_size, max_size in size_range]
    orientation_z = random.uniform(orientation_range[0], orientation_range[1])
    orientation = [0, 0, orientation_z]

    # Create the .sdf file
    root = ET.Element('sdf', version='1.6')
    model = ET.SubElement(root, 'model', name='model')
    link = ET.SubElement(model, 'link', name='link')
    ET.SubElement(link, 'pose').text = ' '.join(map(str, [0, 0, 0] + orientation))

    # Add gravity
    ET.SubElement(link, 'gravity').text = 'true'

    # Make the cuboid static
    ET.SubElement(link, 'static').text = 'true'

    inertial = ET.SubElement(link, 'inertial')
    ET.SubElement(inertial, 'mass').text = '1.0'
    inertia = ET.SubElement(inertial, 'inertia')
    ET.SubElement(inertia, 'ixx').text = '0.16666'
    ET.SubElement(inertia, 'ixy').text = '0'
    ET.SubElement(inertia, 'ixz').text = '0'
    ET.SubElement(inertia, 'iyy').text = '0.16666'
    ET.SubElement(inertia, 'iyz').text = '0'
    ET.SubElement(inertia, 'izz').text = '0.16666'

    collision = ET.SubElement(link, 'collision', name='collision')
    geometry = ET.SubElement(collision, 'geometry')
    box = ET.SubElement(geometry, 'box')
    ET.SubElement(box, 'size').text = ' '.join(map(str, size))

    visual = ET.SubElement(link, 'visual', name='visual')
    geometry = ET.SubElement(visual, 'geometry')
    box = ET.SubElement(geometry, 'box')
    ET.SubElement(box, 'size').text = ' '.join(map(str, size))
    material = ET.SubElement(visual, 'material')
    script = ET.SubElement(material, 'script')
    ET.SubElement(script, 'name').text = 'Gazebo/Blue'
    ET.SubElement(script, 'uri').text = 'file://media/materials/scripts/gazebo.material'

    # Save the .sdf file
    output_dir = '/home/romi-lab-2/project_ws/src/my_simulation/my_simulation/models/cube'
    os.makedirs(output_dir, exist_ok=True)
    output_file = os.path.join(output_dir, 'model.sdf')
    tree = ET.ElementTree(root)
    tree.write(output_file)
    print(f'SDF file saved as {output_file}')