import numpy as np
import trimesh
import shape
from scipy.spatial import Delaunay
import random
import pickle
import os

def points_in_polygon(pts, polygon):
    """
    Returns if the points are inside the given polygon.
    Implemented with angle accumulation.
    """
    polygon = np.vstack((polygon, polygon[0, :]))  # Close the polygon
    sum_angles = np.zeros([len(pts), ])
    for i in range(len(polygon) - 1):
        v1 = polygon[i, :] - pts
        norm_v1 = np.linalg.norm(v1, axis=1, keepdims=True)
        norm_v1[norm_v1 == 0.0] = 1.0  # Prevent divide-by-zero nans
        v1 = v1 / norm_v1
        v2 = polygon[i + 1, :] - pts
        norm_v2 = np.linalg.norm(v2, axis=1, keepdims=True)
        norm_v2[norm_v2 == 0.0] = 1.0  # Prevent divide-by-zero nans
        v2 = v2 / norm_v2
        dot_prods = np.sum(v1 * v2, axis=1)
        cross_prods = np.cross(v1, v2)
        angs = np.arccos(np.clip(dot_prods, -1, 1))
        angs = np.sign(cross_prods) * angs
        sum_angles += angs

    sum_degrees = np.rad2deg(sum_angles)
    return abs(sum_degrees) > 90.0

def generate_meshes(num_meshes=1):
    """ Create multiple meshes from bezier curve points """
    longest_distances = []

    for i in range(num_meshes):
        # Set parameters
        edgy = random.uniform(0.0, 0.1)
        contour = shape.get_bezier_curve(edgy=edgy)
        x, y, z_bottom, z_top = contour[0], contour[1], contour[2], contour[3]

        # Save output
        file_path = os.path.join(os.path.abspath('src'), 'my_simulation', 'my_simulation', 'scripts', 'bezier', f'bezier_curve_{i+1}.pkl')
        with open(file_path, 'wb') as f:
            pickle.dump(contour, f)

        # Create vertices for the bottom and top surfaces
        vertices = np.vstack((np.column_stack((x, y, z_bottom)), np.column_stack((x, y, z_top))))
        num_points = len(x)

        # Create faces
        faces = []

        for j in range(num_points - 1):
            # Side face
            faces.append([j, j + 1, num_points + j + 1])
            faces.append([j, num_points + j + 1, num_points + j])

        # Closing the side face
        faces.append([num_points - 1, 0, num_points])
        faces.append([num_points - 1, num_points, num_points + (num_points - 1)])

        # Top face
        top_vertices = np.column_stack((x, y, z_top))
        top = Delaunay(top_vertices[:, :2])
        top_faces = top.simplices  # Remove the offset

        # Bottom face
        bottom_vertices = np.column_stack((x, y, z_bottom))
        bottom = Delaunay(bottom_vertices[:, :2])
        bottom_faces = bottom.simplices

        # Filter top and bottom faces
        polygon = np.column_stack((x, y))
        
        # Filter top faces
        top_centroids = np.mean(top_vertices[top_faces], axis=1)[:, :2]
        top_mask = points_in_polygon(top_centroids, polygon)
        top_faces = top_faces[top_mask]
        top_faces += num_points  # Add the offset here

        # Filter bottom faces
        bottom_centroids = np.mean(bottom_vertices[bottom_faces], axis=1)[:, :2]
        bottom_mask = points_in_polygon(bottom_centroids, polygon)
        bottom_faces = bottom_faces[bottom_mask]

        # Combine faces
        faces = np.array(faces + top_faces.tolist() + bottom_faces.tolist())

        # Create and export the mesh
        mesh = trimesh.Trimesh(vertices=vertices, faces=faces)

        # Translate the mesh to center it at the origin
        centroid = mesh.centroid
        mesh.apply_transform(trimesh.transformations.translation_matrix(-centroid))

        # Calculate the longest horizontal distance from the center
        vertices = mesh.vertices
        horizontal_distances = np.sqrt(vertices[:, 0]**2 + vertices[:, 1]**2)
        longest_horizontal_distance = np.max(horizontal_distances) * 0.15 # The scale for x and y axis

        # Save the mesh with an incremental name and the longest distance
        mesh.export(os.path.join(os.path.abspath('src'), 'my_simulation', 'my_simulation', 'meshes', 'bezier', f'model_{i+1}.stl'))
        longest_distances.append(longest_horizontal_distance)

    return longest_distances