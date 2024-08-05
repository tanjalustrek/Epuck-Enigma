import random
import xml.etree.ElementTree as ET

### Functions to randomize the maze
def row_4(world_file_path):
    '''Changes the fourth row of the maze'''
    # Set parameters
    link_names = ["Wall_17", "Wall_18", "Wall_16"]
    min_gap = 0.14
    max_gap = 0.35

    # Parse the XML file
    tree = ET.parse(world_file_path)
    root = tree.getroot()

    # Generate a random gap length
    r_5 = round(random.uniform(min_gap, max_gap), 6)
    # Calculate the wall length
    a = (1.4 - 2 * r_5) / 3

    # Update the links
    for link_name in link_names:
        link = root.find(f".//link[@name='{link_name}']")
        # Update the size elements within this link
        for size_elem in link.findall(".//size"):
            current_size = size_elem.text.split()
            current_size[0] = str(a)
            size_elem.text = " ".join(current_size)

        # Update the pose element
        pose_elem = link.find("pose")
        pose = pose_elem.text.split()
        if link_name == "Wall_18":
            pose[0] = str(- a - r_5)
        elif link_name == "Wall_16":
            pose[0] = str(a + r_5)
        pose_elem.text = " ".join(pose)

    # Save the modified XML
    tree.write(world_file_path, encoding="unicode", xml_declaration=True)

def row_3(world_file_path):
    '''Changes the third row of the maze'''
    # Set parameters
    link_names = ["Wall_12", "Wall_13", "Wall_14", "Wall_15"]
    min_gap = 0.14
    max_gap = 0.20
    min_length = 0.2
    max_length = 0.6

    # Parse the XML file
    tree = ET.parse(world_file_path)
    root = tree.getroot()

    # Generate a random gap and length
    r_3 = round(random.uniform(min_gap, max_gap), 6)
    new_length = round(random.uniform(min_length, max_length), 6)
    # Calculate the wall width
    a = (1.4 - 5 * r_3) / 4

    # Update the links
    for link_name in link_names:
        link = root.find(f".//link[@name='{link_name}']")
        # Update the size elements
        for size_elem in link.findall(".//size"):
            current_size = size_elem.text.split()
            current_size[0] = str(new_length)
            current_size[1] = str(a)
            size_elem.text = " ".join(current_size)

        # Update the pose element
        pose_elem = link.find("pose")
        pose = pose_elem.text.split()
        if link_name == "Wall_14":
            pose[0] = str(- a/2 - r_3/2)
        elif link_name == "Wall_15":
            pose[0] = str(- 3 * a/2 - 3 * r_3/2)
        elif link_name == "Wall_13":
            pose[0] = str(a/2 + r_3/2)
        elif link_name == "Wall_12":
            pose[0] = str(3 * a/2 + 3 * r_3/2)
        pose_elem.text = " ".join(pose)

    # Save the modified XML
    tree.write(world_file_path, encoding="unicode", xml_declaration=True)

def row_2(world_file_path):
    '''Changes the second row of the maze'''
    # Set parameters
    link_names_ver = ["Wall_9", "Wall_11"]
    link_names_hor = ["Wall_8", "Wall_10"]
    min_gap = 0.14
    max_gap = 0.3
    min_length = 0.2
    max_length = 0.5

    # Parse the XML file
    tree = ET.parse(world_file_path)
    root = tree.getroot()

    # Generate a random gap and length of horizontal walls
    r_3 = round(random.uniform(min_gap, max_gap), 6)
    new_length = round(random.uniform(min_length, max_length), 6)
    # Calculate the vertical wall length
    a = (1.4 - 3 * r_3) / 2

    # Update the vertical links
    for link_name in link_names_ver:
        link = root.find(f".//link[@name='{link_name}']")
        # Update the size elements
        for size_elem in link.findall(".//size"):
            current_size = size_elem.text.split()
            current_size[0] = str(a)
            size_elem.text = " ".join(current_size)
            if link_name == "Wall_11":
                b_11 = current_size[1]
            elif link_name == "Wall_9":
                b_9 = current_size[1]

        # Update the pose element
        pose_elem = link.find("pose")
        pose = pose_elem.text.split()
        if link_name == "Wall_11":
            pose[0] = str(- a/2 - r_3/2)
            y_11 = pose[1]
        elif link_name == "Wall_9":
            pose[0] = str(a/2 + r_3/2)
            y_9 = pose[1]
        pose_elem.text = " ".join(pose)

    # Update the horizontal links
    for link_name in link_names_hor:
        link = root.find(f".//link[@name='{link_name}']")
        # Update the size elements
        for size_elem in link.findall(".//size"):
            current_size = size_elem.text.split()
            current_size[0] = str(new_length)
            size_elem.text = " ".join(current_size)

        # Update the pose element
        pose_elem = link.find("pose")
        pose = pose_elem.text.split()
        if link_name == "Wall_10":
            pose[0] = str(- a/2 - r_3/2)
            pose[1] = str(float(y_11) + float(b_11)/2 + new_length/2)
        elif link_name == "Wall_8":
            pose[0] = str(a/2 + r_3/2)
            pose[1] = str(float(y_9) + float(b_9)/2 + new_length/2)
        pose_elem.text = " ".join(pose)

    # Save the modified XML
    tree.write(world_file_path, encoding="unicode", xml_declaration=True)

def row_1(world_file_path):
    '''Changes the first row of the maze'''
    # Set parameters
    link_names = ["Wall_5", "Wall_6", "Wall_7"]
    min_gap_1 = 0.2
    max_gap_1 = 0.25
    min_gap_2 = 0.14
    max_gap_2 = 0.2

    # Parse the XML file
    tree = ET.parse(world_file_path)
    root = tree.getroot()

    # Generate random gaps
    r_1 = round(random.uniform(min_gap_1, max_gap_1), 6)
    r_2 = round(random.uniform(min_gap_2, max_gap_2), 6)
    # Calculate the wall width
    a = (1.4 - 2 * (r_1 + r_2)) / 3

    # Get the x coordinate of position and the width of Wall_2 (north wall)
    link = root.find(".//link[@name='Wall_2']")
    pose_elem = link.find("pose")
    pose = pose_elem.text.split()
    p_2 = float(pose[0])
    size_elem = link.find(".//size")
    size = size_elem.text.split()
    s_2 = float(size[1])

    # Update the links
    for link_name in link_names:
        link = root.find(f".//link[@name='{link_name}']")
        # Update the size elements
        for size_elem in link.findall(".//size"):
            current_size = size_elem.text.split()
            current_size[0] = str(a)
            size_elem.text = " ".join(current_size)

        # Update the pose element
        pose_elem = link.find("pose")
        pose = pose_elem.text.split()
        if link_name == "Wall_5":
            pose[0] = str(p_2 - s_2/2 - a/2 - r_1)
        elif link_name == "Wall_6":
            pose[0] = str(p_2 - s_2/2 - 3 * a/2 - 2 * r_1)
        elif link_name == "Wall_7":
            pose[0] = str(p_2 - s_2/2 - 5 * a/2 - 2 * r_1 - r_2)
        pose_elem.text = " ".join(pose)

    # Save the modified XML
    tree.write(world_file_path, encoding="unicode", xml_declaration=True)