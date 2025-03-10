import numpy as np
import open3d as o3d
import math

def fit_plane(points):
    """
    Fit a plane to a set of points using least squares.
    Returns the normal vector and the offset of the plane.
    """
    centroid = points.mean(axis=0)
    _, _, v = np.linalg.svd(points - centroid)
    normal = v[-1]
    offset = -centroid.dot(normal)
    return normal, offset

def ransac_plane(points, num_iterations=100, threshold=0.01):
    """
    Detect a plane in a point cloud using RANSAC.
    """
    num_points = points.shape[0]
    best_normal = None
    best_offset = None
    max_inliers = 0

    for _ in range(num_iterations):
        # Step 1: Select three random points
        indices = np.random.choice(num_points, 3, replace=False)
        sample_points = points[indices]

        # Step 2: Fit a plane to the selected points
        normal, offset = fit_plane(sample_points)

        # Step 3: Count inliers
        distances = np.abs(points.dot(normal) + offset)
        inliers = points[distances < threshold]
        num_inliers = inliers.shape[0]

        # Step 4: Update best model if current model has more inliers
        if num_inliers > max_inliers:
            max_inliers = num_inliers
            best_normal = normal
            best_offset = offset

    # Step 5: Refine the model using all inliers
    if max_inliers > 0:
        distances = np.abs(points.dot(best_normal) + best_offset)
        inliers = points[distances < threshold]
        best_normal, best_offset = fit_plane(inliers)

    return best_normal, best_offset, max_inliers

def convert_to_pointcloud(points):
    """
    Convert a list of points to an Open3D PointCloud object.

    Args:
        points (numpy.ndarray): A Nx3 array of points where N is the number of points.

    Returns:
        open3d.geometry.PointCloud: The PointCloud object.
    """
    # Create a PointCloud object
    pcd = o3d.geometry.PointCloud()

    # Convert the numpy array to an Open3D Vector3dVector
    pcd.points = o3d.utility.Vector3dVector(points)

    return pcd

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

def visualize_plane(points, normal, offset):
    """
    Visualize the point cloud and the detected plane.
    """
    # Create a plane mesh
    mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0, origin=[0, 0, 0])

    # Create a point cloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    plane = create_plane_from_points([[ 1,  1,( normal[0]+normal[1]+offset)/(-normal[2])],
                                      [-1,  1,(-normal[0]+normal[1]+offset)/(-normal[2])],
                                      [-1, -1,(-normal[0]-normal[1]+offset)/(-normal[2])],
                                      [ 1, -1,( normal[0]-normal[1]+offset)/(-normal[2])]])
    plane.paint_uniform_color([1.0, 0.0, 0.0])
    # Visualize
    o3d.visualization.draw_geometries([pcd, plane, mesh_frame])


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

v = [3, 5, 0]
axis = [4, 4, 1]
theta = 1.2 

# Example usage
if __name__ == "__main__":
    # Generate a sample point cloud with a plane
    np.random.seed(0)
    axis = np.random.rand(1,3)[0]*2 - 1
    rot_mat = rotation_matrix(axis, (np.random.rand()*2 - 1)*3.1415)
    plane_points = np.random.rand(100, 3) * 2 - 1
    plane_points[:, 2] = 0  # All points lie on the XY plane
    noise = np.random.rand(100, 3) * 0.1
    points = plane_points + noise
    points = (rot_mat @ points.T).T

    # Detect the plane using RANSAC
    normal, offset, num_inliers = ransac_plane(points)
    print(f"Detected plane: normal={normal}, offset={offset}, inliers={num_inliers}")

    # Visualize the result
    visualize_plane(points, normal, offset)