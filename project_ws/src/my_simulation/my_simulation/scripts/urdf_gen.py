import os
import xml.etree.ElementTree as ET

### Generates a bocbot_gen.urdf file with personalized paths
def generate_urdf():
    # Get the package directory
    package_dir = os.path.join(os.path.abspath('src'), 'my_simulation', 'my_simulation')

    # Construct the path to the meshes
    mesh_dir = os.path.join(package_dir, 'models', 'urdf', 'meshes')

    # Parse the existing URDF file
    urdf_path = os.path.join(package_dir, 'models', 'urdf', 'bocbot.urdf')
    tree = ET.parse(urdf_path)
    root = tree.getroot()

    # Update all mesh filenames
    for elem in root.iter('mesh'):
        filename = elem.get('filename')
        if filename:
            # Extract just the filename from the path
            base_filename = os.path.basename(filename)
            # Construct the new path
            new_filename = os.path.join(mesh_dir, base_filename)
            elem.set('filename', new_filename)

    # Write the updated URDF to a new file
    output_path = os.path.join(package_dir, 'models', 'urdf', 'bocbot_gen.urdf')
    tree.write(output_path, encoding='utf-8', xml_declaration=True)