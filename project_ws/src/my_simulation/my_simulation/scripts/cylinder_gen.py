import os
import random
import xml.etree.ElementTree as ET

def generate_cylinder(num_files=1):
    '''Generates random .sdf files for cylinders'''
    # Define the radius range and length
    radius_range = (0.02, 0.06)
    length = 0.1

    # Create the desired number of files
    for i in range(num_files):
        # Generate random radius value
        radius = random.uniform(radius_range[0], radius_range[1])

        # Create the .sdf file
        root = ET.Element('sdf', version='1.6')
        model = ET.SubElement(root, 'model', name='model')

        # Set static to true
        ET.SubElement(model, 'static').text = 'true'

        # Add pose
        link = ET.SubElement(model, 'link', name='link')
        ET.SubElement(link, 'pose').text = '0 0 0 0 0 0'

        # Add collision
        collision = ET.SubElement(link, 'collision', name='collision')
        geometry = ET.SubElement(collision, 'geometry')
        cylinder = ET.SubElement(geometry, 'cylinder')
        ET.SubElement(cylinder, 'radius').text = str(radius)
        ET.SubElement(cylinder, 'length').text = str(length)

        # Add visual
        visual = ET.SubElement(link, 'visual', name='visual')
        geometry = ET.SubElement(visual, 'geometry')
        cylinder = ET.SubElement(geometry, 'cylinder')
        ET.SubElement(cylinder, 'radius').text = str(radius)
        ET.SubElement(cylinder, 'length').text = str(length)
        material = ET.SubElement(visual, 'material')
        script = ET.SubElement(material, 'script')
        ET.SubElement(script, 'name').text = 'Gazebo/Green'
        ET.SubElement(script, 'uri').text = 'file://media/materials/scripts/gazebo.material'

        # Save the .sdf file
        output_dir = os.path.join(os.path.abspath('src'), 'my_simulation', 'my_simulation', 'models', 'cylinder')
        os.makedirs(output_dir, exist_ok=True)
        output_file = os.path.join(output_dir, f'model_{i+1}.sdf')
        tree = ET.ElementTree(root)
        tree.write(output_file)

def get_radius(sdf_file_path):
    '''Gets the radius of the generated cylinder'''
    tree = ET.parse(sdf_file_path)
    root = tree.getroot()
    radius = float(root.find(".//cylinder/radius").text)
    return radius