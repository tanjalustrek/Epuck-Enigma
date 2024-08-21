import os

def generate_fixed_triangular_prism():
    sdf_template = '''<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="triangular_prism">
    <link name="link">
      <pose>0 0 0 0 0 0</pose>
      <inertial>
        <mass>0.05</mass>
        <inertia>
          <ixx>0.000041667</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.000041667</iyy>
          <iyz>0</iyz>
          <izz>0.000041667</izz>
        </inertia>
      </inertial>

      <collision name="collision">
        <geometry>
          <mesh>
            <uri>file://{path}</uri>
            <scale>0.015 0.015 0.015</scale>
          </mesh>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>0.1</mu>
              <mu2>0.1</mu2>
            </ode>
          </friction>
          <contact>
            <ode>
              <kp>10000.0</kp>
              <kd>10.0</kd>
              <max_vel>0.01</max_vel>
              <min_depth>0.001</min_depth>
            </ode>
          </contact>
        </surface>
      </collision>

      <visual name="visual">
        <geometry>
          <mesh>
            <uri>file://{path}</uri>
            <scale>0.015 0.015 0.015</scale>
          </mesh>
        </geometry>
        <material>
          <script>
            <name>Gazebo/Green</name>
            <uri>file://media/materials/scripts/gazebo.material</uri>
          </script>
        </material>
      </visual>
    </link>
  </model>
</sdf>
'''
    
    # Get the mesh path
    mesh_path = os.path.join(os.path.abspath('src'), 'my_simulation', 'my_simulation', 'meshes', 'triangular_prism', 'triangular_prism.stl')

    file_content = sdf_template.format(path=mesh_path)
    file_path = os.path.join(os.path.abspath('src'), 'my_simulation', 'my_simulation', 'models', 'triangular_prism', 'triangular_prism.sdf')
    
    with open(file_path, 'w') as f:
        f.write(file_content)