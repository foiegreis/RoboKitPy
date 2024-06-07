import xml.etree.ElementTree as ET
from robokitpy.core.dynamics import *

# Load and parse the URDF file
def load_urdf(urdf_file):
    tree = ET.parse(urdf_file)
    root = tree.getroot()
    return root

# Extract mass matrix from a link
def get_mass_matrix(inertial):
    mass = float(inertial.find('mass').attrib['value'])
    return np.diag([mass, mass, mass])

# Extract spatial inertia matrix from a link
def get_spatial_inertia_matrix(inertial):
    inertia_elem = inertial.find('inertia')
    ixx = float(inertia_elem.attrib['ixx'])
    ixy = float(inertia_elem.attrib['ixy'])
    ixz = float(inertia_elem.attrib['ixz'])
    iyy = float(inertia_elem.attrib['iyy'])
    iyz = float(inertia_elem.attrib['iyz'])
    izz = float(inertia_elem.attrib['izz'])

    inertia_matrix = np.array([
        [ixx, ixy, ixz],
        [ixy, iyy, iyz],
        [ixz, iyz, izz]
    ])

    # If origin is specified, we should rotate the inertia matrix to align with the center of mass
    origin = inertial.find('origin')
    if origin is not None:
        rpy = origin.attrib.get('rpy', '0 0 0').split()
        rpy = [float(angle) for angle in rpy]
        print(rpy)
        R = euler_to_rotation_matrix(*rpy)
        inertia_matrix = R.T @ inertia_matrix @ R

    return inertia_matrix

# Convert Euler angles to a rotation matrix
def euler_to_rotation_matrix(roll, pitch, yaw):
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])
    Ry = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])
    Rz = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])
    return Rz @ Ry @ Rx

# Extract screw axes from a joint
def get_screw_axes(joint):
    print(joint.find('name'))
    axis = np.array([float(x) for x in joint.find('axis').attrib['xyz'].split()])
    print("axis", axis)
    origin = joint.find('origin')
    xyz = np.array([0, 0, 0])
    rpy = np.array([0, 0, 0])

    if origin is not None:
        xyz = np.array([float(x) for x in origin.attrib.get('xyz', '0 0 0').split()])
        rpy = np.array([float(x) for x in origin.attrib.get('rpy', '0 0 0').split()])

    R = euler_to_rotation_matrix(*rpy)
    p = xyz
    screw_axis = np.zeros(6)
    screw_axis[:3] = R @ axis
    if joint.attrib['type'] != 'prismatic':
        screw_axis[3:] = np.cross(p, screw_axis[:3])
    else:
        screw_axis[3:] = p

    return screw_axis

# Main function to extract information from URDF
def extract_urdf_info(urdf_file):
    root = load_urdf(urdf_file)
    S_list = []
    M_list = []
    G_list = []
    # Extract information from links
    for link in root.findall('link'):
        name = link.attrib['name']
        inertial = link.find('inertial')
        if inertial is not None:
            mass_matrix = get_mass_matrix(inertial)
            spatial_inertia_matrix = get_spatial_inertia_matrix(inertial)
            M_list.append(mass_matrix)
            G_list.append(spatial_inertia_matrix)

    # Extract information from joints
    for joint in root.findall('joint'):
        name = joint.attrib['name']
        print(name)
        try:
            screw_axis = get_screw_axes(joint)
            print(f"Screw Axis: {screw_axis}")
            S_list.append(screw_axis)
        except:
            pass
        print(f"Joint: {name}")
        print()
    return M_list, G_list, S_list


if __name__=='__main__':

    urdf_filepath = '../models/URDF/urdf/ur5e.urdf'

    M_list, G_list, S_list = extract_urdf_info(urdf_filepath)

    print("\nMass matrix list\n", M_list)
    print("M shape ", np.shape(M_list))
    print("\nSpatial Inertia Matrix list\n", G_list)
    print("G shape: ", np.shape(G_list))
    print("\nScrew axes\n", S_list)

    # Try inverse dynamics
    thetalist = [0, np.pi / 6, np.pi / 4, np.pi / 3, np.pi / 2, 2 * np.pi / 3]

    dthetalist = [0.2] * 6

    ddthetalist = [0.1] * 6

    g = [0, 0, -9.81]

    Ftip = [0.1] * 6

    joint_torques = inverse_dynamics(thetalist, dthetalist, ddthetalist, M_list, G_list, S_list, g, Ftip)
    print("Joint Torques/Forces:", joint_torques)