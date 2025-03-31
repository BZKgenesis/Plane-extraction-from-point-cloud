import open3d as o3d
import numpy as np
import time
import math

def rotation_matrix(axis, theta):
    """
    Return the rotation matrix associated with counterclockwise rotation about
    the given axis by theta radians.
    """
    axis = np.asarray(axis)
    axis = axis / math.sqrt(np.dot(axis, axis))
    a = math.cos(theta / 2.0)
    b, c, d = -axis * math.sin(theta / 2.0)
    aa, bb, cc, dd = a * a, b * b, c * c, d * d
    bc, ad, ac, ab, bd, cd = b * c, a * d, a * c, a * b, b * d, c * d
    return np.array([[aa + bb - cc - dd, 2 * (bc + ad), 2 * (bd - ac)],
                     [2 * (bc - ad), aa + cc - bb - dd, 2 * (cd + ab)],
                     [2 * (bd + ac), 2 * (cd - ab), aa + dd - bb - cc]])


# Fonction pour simuler un flux de nuages de points en direct
def simulate_point_cloud_stream():
    rotation = 0
    while True:
        # Générer un nuage de points aléatoire (remplacez ceci par votre source de données réelle)
        
        np.random.seed(0)
        axis = np.random.rand(1,3)[0]*2 - 1
        rotation += np.random.rand()*0.1
        rot_mat = rotation_matrix(axis, rotation)
        np.random.seed()
        plane_points = np.random.rand(100, 3) * 100 - 50
        plane_points[:, 2] = 0  # All points lie on the XY plane
        noise = np.random.rand(100, 3) * 0.1
        points = plane_points + noise
        points = (rot_mat @ points.T).T


        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        yield pcd
        time.sleep(0.1)  # Simuler un délai entre les nuages de points

# Fonction pour extraire un plan d'un nuage de points en utilisant RANSAC
def extract_plane(pcd):
    plane_model, inliers = pcd.segment_plane(distance_threshold=0.01,
                                             ransac_n=3,
                                             num_iterations=1000)
    inlier_cloud = pcd.select_by_index(inliers)
    outlier_cloud = pcd.select_by_index(inliers, invert=True)
    return inlier_cloud, outlier_cloud, plane_model

# Fonction pour visualiser les nuages de points et les plans extraits
def visualize_point_cloud_and_plane(pcd, inlier_cloud, outlier_cloud, plane_model):
    # Colorier les points inliers en rouge et les points outliers en bleu
    inlier_cloud.paint_uniform_color([1.0, 0.0, 0.0])
    outlier_cloud.paint_uniform_color([0.0, 0.0, 1.0])

    # Créer un maillage de plan à partir du modèle de plan
    plane_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(inlier_cloud, alpha=0.05)
    plane_mesh.compute_vertex_normals()
    plane_mesh.paint_uniform_color([0.0, 1.0, 0.0])

    # Visualiser les nuages de points et le plan
    o3d.visualization.draw_geometries([pcd, plane_mesh])


def create_plane_from_points(points):
    """
    Create a plane from four points.

    Args:
        points (numpy.ndarray): A 4x3 array of points where each row is a point [x, y, z].

    Returns:
        o3d.geometry.TriangleMesh: The plane mesh.
    """
    # Create a triangle mesh from the points
    vertices = o3d.utility.Vector3dVector(points)
    triangles = o3d.utility.Vector3iVector([[0, 1, 2], [0, 2, 3]])
    plane = o3d.geometry.TriangleMesh(vertices, triangles)

    # Compute vertex normals for better visualization
    plane.compute_vertex_normals()

    return plane

# Fonction principale pour démarrer le flux et la visualisation
def main():
    o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Debug)

    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.get_render_option().mesh_show_back_face = True
    # Démarrer le flux de nuages de points en direct
    point_cloud_stream = simulate_point_cloud_stream()
    points = o3d.geometry.PointCloud()
    points.points = next(point_cloud_stream).points
    
    vertices = o3d.utility.Vector3dVector([[0,0,0],[0,0,0],[0,0,0],[0,0,0]])
    triangles = o3d.utility.Vector3iVector([[0, 1, 2], [0, 2, 3]])
    plane = o3d.geometry.TriangleMesh(vertices, triangles)

    mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=10.0, origin=[0, 0, 0])

    vis.add_geometry(mesh_frame)
    vis.add_geometry(points)
    vis.add_geometry(plane)
    threshold = 0.05
    icp_iteration = 100
    save_image = False 

    # Boucle principale pour traiter et visualiser les nuages de points en temps réel
    for pcd in point_cloud_stream:
        # Extraire le plan du nuage de points
        inlier_cloud, outlier_cloud, plane_model = extract_plane(pcd)
        
        PLANE_SIZE = 20
        vect1 = np.array([ PLANE_SIZE,  PLANE_SIZE,( PLANE_SIZE*plane_model[0]+PLANE_SIZE*plane_model[1]+plane_model[3])/(-plane_model[2])])
        vect2 = np.array([-PLANE_SIZE,  PLANE_SIZE,(-PLANE_SIZE*plane_model[0]+PLANE_SIZE*plane_model[1]+plane_model[3])/(-plane_model[2])])
        vect1 /= np.sqrt(vect1[0]**2 + vect1[1]**2 + vect1[2]**2)
        vect2 /= np.sqrt(vect2[0]**2 + vect2[1]**2 + vect2[2]**2)


        plane.vertices = o3d.utility.Vector3dVector([vect1*PLANE_SIZE,
                                        vect2*PLANE_SIZE,
                                        -vect1*PLANE_SIZE,
                                        -vect2*PLANE_SIZE])

        plane.compute_vertex_normals()
        vis.update_geometry(plane)

        points.points = pcd.points
        vis.update_geometry(points)
        if not vis.poll_events():
            break
        vis.update_renderer()

    vis.destroy_window()

    o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Info)

if __name__ == "__main__":
    main()
