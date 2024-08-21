import os

def generate_bezier(num_files=1):
    sdf_template = '''<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="bezier_{index}">
    <link name="link">
      <pose>0 0 0 0 0 0</pose>
      <gravity>true</gravity>
      <static>false</static>
      <inertial>
        <mass>0.05</mass>
        <inertia>
          <ixx>5</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>5</iyy>
          <iyz>0</iyz>
          <izz>0.1</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry>
          <mesh>
            <uri>file://{path}_{index}.stl</uri>
            <scale>0.15 0.15 0.1</scale>
          </mesh>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>0.13</mu>
              <mu2>0.13</mu2>
            </ode>
          </friction>
          <contact>
            <ode>
              <max_vel>0.001</max_vel>
              <min_depth>0.1</min_depth>
              <kp>1e8</kp>
              <kd>100</kd>
            </ode>
          </contact>
        </surface>
      </collision>
      <visual name="visual">
        <geometry>
          <mesh>
            <uri>file://{path}_{index}.stl</uri>
            <scale>0.15 0.15 0.1</scale>
          </mesh>
        </geometry>
        <material>
          <script>
            <name>Gazebo/Purple</name>
            <uri>file://media/materials/scripts/gazebo.material</uri>
          </script>
        </material>
      </visual>
    </link>
  </model>
</sdf>
'''

    # Create the directory if it doesn't exist
    output_dir = os.path.join(os.path.abspath('src'), 'my_simulation', 'my_simulation', 'models', 'bezier')
    os.makedirs(output_dir, exist_ok=True)
    # Get the mesh path
    mesh_path = os.path.join(os.path.abspath('src'), 'my_simulation', 'my_simulation', 'meshes', 'bezier', 'model')

    for i in range(1, num_files + 1):
        file_content = sdf_template.format(path=mesh_path, index=i)
        file_name = f'model_{i}.sdf'
        file_path = os.path.join(output_dir, file_name)
        
        with open(file_path, 'w') as f:
            f.write(file_content)