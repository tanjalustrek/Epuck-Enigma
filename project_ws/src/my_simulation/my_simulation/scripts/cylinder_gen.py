import os
import random
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory

### Generates random .sdf files for cylinders
def generate_cylinder(num_files=1):
    # Define the radius range
    radius_range = (0.05, 0.2)
    length = 0.5

    for i in range(num_files):
        # Generate random radius value
        radius = random.uniform(radius_range[0], radius_range[1])

        # Create the .sdf file
        root = ET.Element('sdf', version='1.6')
        model = ET.SubElement(root, 'model', name='model')
        link = ET.SubElement(model, 'link', name='link')
        ET.SubElement(link, 'pose').text = '0 0 0 0 0 0'

        # Add gravity
        ET.SubElement(link, 'gravity').text = 'true'

        # Make the cylinder static
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
        cylinder = ET.SubElement(geometry, 'cylinder')
        ET.SubElement(cylinder, 'radius').text = str(radius)
        ET.SubElement(cylinder, 'length').text = str(length)

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
        print(f'SDF file saved as {output_file}')

### Gets the radius
def get_radius(sdf_file_path):
    tree = ET.parse(sdf_file_path)
    root = tree.getroot()
    radius = float(root.find(".//cylinder/radius").text)
    return radius