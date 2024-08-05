import os
import random
import xml.etree.ElementTree as ET

def generate_cuboid(num_files=1):
    '''Generates random .sdf files for cuboids'''
    # Define the size and orientation ranges and height
    size_range = [(0.05, 0.15), (0.05, 0.15)]
    orientation_range = (0, 3.14)
    height = 0.1

    # Create the desired number of files
    for i in range(num_files):
        # Generate random size and random rotation around the z-axis
        size = [random.uniform(min_size, max_size) for min_size, max_size in size_range]
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
        box = ET.SubElement(geometry, 'box')
        ET.SubElement(box, 'size').text = ' '.join(map(str, size + [height]))

        # Add visual
        visual = ET.SubElement(link, 'visual', name='visual')
        geometry = ET.SubElement(visual, 'geometry')
        box = ET.SubElement(geometry, 'box')
        ET.SubElement(box, 'size').text = ' '.join(map(str, size + [height]))
        material = ET.SubElement(visual, 'material')
        script = ET.SubElement(material, 'script')
        ET.SubElement(script, 'name').text = 'Gazebo/Blue'
        ET.SubElement(script, 'uri').text = 'file://media/materials/scripts/gazebo.material'

        # Save the .sdf file
        output_dir = os.path.join(os.path.abspath('src'), 'my_simulation', 'my_simulation', 'models', 'cuboid')
        os.makedirs(output_dir, exist_ok=True)
        output_file = os.path.join(output_dir, f'model_{i+1}.sdf')
        tree = ET.ElementTree(root)
        tree.write(output_file)

def get_edges(sdf_file_path):
    '''Gets the length and width of the generated cuboid'''
    tree = ET.parse(sdf_file_path)
    root = tree.getroot()
    size_element = root.find(".//box/size")
    size_values = [float(value) for value in size_element.text.split()][:2]
    return size_values