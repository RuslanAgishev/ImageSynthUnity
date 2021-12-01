import cv2
import numpy as np
import json
import os
from abc import ABC, abstractmethod
from pyquaternion import Quaternion
from time import sleep
import math


def compute_box_3d(dim, location, rotation_y):
    # dim: 3
    # location: 3
    # rotation_y: 1
    # return: 8 x 3
    cosine, sine = np.cos(rotation_y), np.sin(rotation_y)
    R = np.array([[cosine, 0, sine], [0, 1, 0], [-sine, 0, cosine]], dtype=np.float32)
    length, width, height = dim[2], dim[1], dim[0]
    x_corners = [length / 2, length / 2, -length / 2, -length / 2, length / 2, length / 2, -length / 2, -length / 2]
    y_corners = [0, 0, 0, 0, -height, -height, -height, -height]
    z_corners = [width / 2, -width / 2, -width / 2, width / 2, width / 2, -width / 2, -width / 2, width / 2]
    corners = np.array([x_corners, y_corners, z_corners], dtype=np.float32)
    corners_3d = np.dot(R, corners)
    corners_3d = corners_3d + np.array(location, dtype=np.float32).reshape(3, 1)
    return corners_3d.transpose(1, 0)
def project_to_image(pts_3d, P):
    # pts_3d: n x 3
    # P: 3 x 4
    # return: n x 2
    pts_3d_homo = np.concatenate(
        [pts_3d, np.ones((pts_3d.shape[0], 1), dtype=np.float32)], axis=1)
    pts_2d = np.dot(P, pts_3d_homo.transpose(1, 0)).transpose(1, 0)
    pts_2d = pts_2d[:, :2] / pts_2d[:, 2:]
    # import pdb; pdb.set_trace()
    return pts_2d
def draw_box_3d(image, corners, c=(0, 0, 255)):
    face_idx = [[0, 1, 5, 4],
                [1, 2, 6, 5],
                [2, 3, 7, 6],
                [3, 0, 4, 7]]
    for ind_f in range(3, -1, -1):
        f = face_idx[ind_f]
        for j in range(4):
            cv2.line(image, (corners[f[j], 0], corners[f[j], 1]),
                     (corners[f[(j + 1) % 4], 0], corners[f[(j + 1) % 4], 1]), c, 2, lineType=cv2.LINE_AA)
        if ind_f == 0:
            cv2.line(image, (corners[f[0], 0], corners[f[0], 1]),
                     (corners[f[2], 0], corners[f[2], 1]), c, 1, lineType=cv2.LINE_AA)
            cv2.line(image, (corners[f[1], 0], corners[f[1], 1]),
                     (corners[f[3], 0], corners[f[3], 1]), c, 1, lineType=cv2.LINE_AA)
    return image
def yaw_pitch_roll(q):
    """
    Calculate Euler angles from pyquaternion.
    Output: - `roll`: rotation about the new X-axis
            - `pitch`: rotation about the new Y-axis
            - `yaw`: rotation about the new Z-axis
    """
    q = q.normalised
    roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x ** 2 + q.y ** 2))
    pitch = math.asin(2 * (q.w * q.y - q.z * q.x))
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))
    return yaw, pitch, roll
def quat_orient_diff(q1, q2):
    """
    Outputs orientation difference of quaternion `q2` relative to `q1`.
    Result is a quaternion `q12`.
    Reference: https://stackoverflow.com/questions/57063595/how-to-obtain-the-angle-between-two-quaternions
    """
    pyq1 = Quaternion(q1).normalised # quaternion from world->BODY1
    pyq2 = Quaternion(q2).normalised # quaternion from world->BODY2
    q12 = pyq1.conjugate * pyq2      # quaternion from BODY1->BODY2
    return q12.normalised
class ObjectOrientedBase(ABC):
    def __init__(self, file_path):
        self._file_path = file_path
        self._name = self._file_path.split('/')[-1].split('_')[-1].split('.')[0]
        with open(self._file_path, 'r') as f:
            self._data = json.load(f)
        self._data = [self._transform_raw_frame(frame) for frame in self._data]
        self._data_map = dict()
        for frame in self._data:
            self._data_map[frame['frame_number']] = frame
    @abstractmethod
    def get_object_name(self) -> str:
        pass
    @abstractmethod
    def get_frame_data(self, index: int):
        pass
    @staticmethod
    @abstractmethod
    def _transform_raw_frame(raw_frame):
        pass
class ObjectOrientedOdometrySequence(ObjectOrientedBase):
    def __init__(self, file_path):
        super(ObjectOrientedOdometrySequence, self).__init__(file_path)
    @staticmethod
    def _transform_raw_frame(raw_frame):
        for key in raw_frame:
            if isinstance(raw_frame[key], str):
                raw_frame[key] = raw_frame[key].lstrip('(').rstrip(')')
                raw_frame[key] = raw_frame[key].split(', ')
                raw_frame[key] = np.array([float(value) for value in raw_frame[key]], dtype=np.float32)
        return raw_frame
    def get_object_name(self) -> str:
        return self._name
    def get_frame_data(self, index: int):
        return self._data_map[index]
class ObjectOrientedDetectionSequence(ObjectOrientedBase):
    def __init__(self, file_path):
        super(ObjectOrientedDetectionSequence, self).__init__(file_path)
    def _transform_raw_frame(self, raw_frame):
        for key in raw_frame:
            if isinstance(raw_frame[key], str):
                if key == 'tags':
                    raw_frame[key] = list(raw_frame[key].split(', '))
                else:
                    raw_frame[key] = list(raw_frame[key].split('), ('))
                    raw_frame[key] = [self._parse_tuple(obj) for obj in raw_frame[key]]
        return raw_frame
    @staticmethod
    def _parse_tuple(obj):
        obj = obj.lstrip('(').rstrip(')')
        obj = obj.split(', ')
        obj = np.array([float(value) for value in obj], dtype=np.float32)
        return obj
    def get_object_name(self) -> str:
        return self._name
    def get_frame_data(self, index: int):
        return self._data_map[index]
# IMAGES_PATH = '/home/sawseen/data/datasets/simulation/images-20200710T102857Z-001/images/'
# ODOMETRY_PATH = '/home/sawseen/data/datasets/simulation/odometry-20200710T103221Z-001/odometry/'
# DETECTION_PATH = '/home/sawseen/data/datasets/simulation/detections-20200710T103208Z-001/detections'
IMAGES_PATH = '/home/ruslan/Desktop/Unity/CScape/dataset/test/images/'
ODOMETRY_PATH = '/home/ruslan/Desktop/Unity/CScape/dataset/test/odometry/'
DETECTION_PATH = '/home/ruslan/Desktop/Unity/CScape/dataset/test/detections/'
CALIB = np.array([
    [615.7320556640625, 0.0, 312.435302734375, 0.0],
    [0.0, 615.7219848632812, 244.60476684570312, 0],
    [0.0, 0.0,                1.0, 0.0]
], dtype=np.float32)
CLASSES = [
    'Pedestrian', 'TrafficCar'
]
CLASSES_IDS = {cat: i + 1 for i, cat in enumerate(CLASSES)}
CLASSES_INFO = list()
for i, cat in enumerate(CLASSES):
    CLASSES_INFO.append({'name': cat, 'id': i + 1})
DEBUG = True
def _bbox_to_coco_bbox(bbox):
    return [(float(bbox[0])), (float(bbox[1])),
            (float(bbox[2]) - float(bbox[0])), (float(bbox[3]) - float(bbox[1]))]
def process_labels(images_dir_path, odometry_dir_path, detections_dir_path):
    host_to_odometry = dict()
    host_to_detections = dict()
    for detections_name in os.listdir(detections_dir_path):
        detections_file_path = os.path.join(detections_dir_path, detections_name)
        host_detections = ObjectOrientedDetectionSequence(detections_file_path)
        host_to_detections[host_detections.get_object_name()] = host_detections
    for odometry_name in os.listdir(odometry_dir_path):
        odometry_file_path = os.path.join(odometry_dir_path, odometry_name)
        host_odometry = ObjectOrientedOdometrySequence(odometry_file_path)
        host_to_odometry[host_odometry.get_object_name()] = host_odometry
    for image_name in list(sorted(os.listdir(images_dir_path))):
        image_path = os.path.join(images_dir_path, image_name)
        host_name, frame_index = image_name.split('_')[0], int(image_name.split('_')[1])
        host_odometry = host_to_odometry[host_name].get_frame_data(frame_index)
        host_detections = host_to_detections[host_name].get_frame_data(frame_index)
        ret = {'images': [], 'annotations': [], "categories": CLASSES_INFO}
        image_info = {
            'dataset': 'sim0',
            'file_name': image_path,
            'id': image_path,
            'calib': CALIB.tolist()
        }
        ret['images'].append(image_info)
        if DEBUG:
            image = cv2.imread(image_info['file_name'])
        host_position = host_odometry['pose_m']
        host_orientation = host_odometry['orient_quat']
        host_orientation = Quaternion([
            host_orientation[3], host_orientation[0], host_orientation[1], host_orientation[2]
        ]).normalised
        object_classes = host_detections['tags']
        object_poses = host_detections['poses_m']
        object_quats = host_detections['orients_quat']
        object_bbox_sizes = host_detections['bbox_sizes_m']
        for object_index in range(len(object_classes)):
            cat_id = CLASSES_IDS[object_classes[object_index]]
            size = object_bbox_sizes[object_index]
            # height, width, length
            dim = (size[1], size[0], size[2])
            host_rotation_matrix = host_orientation.rotation_matrix
            location = object_poses[object_index] - host_position
            location = host_orientation.inverse.rotate(location)
            # location = np.dot(location.reshape(1, 3), host_rotation_matrix).flatten()
            x, y, z = location
            location[1] = -y

            object_orientation = Quaternion([
                object_quats[object_index][3],
                object_quats[object_index][0],
                object_quats[object_index][1],
                object_quats[object_index][2]
            ]).normalised
            object_orientation_relative_to_host = quat_orient_diff(host_orientation, object_orientation)
            euler_angles = object_orientation_relative_to_host.yaw_pitch_roll
            # euler_angles = yaw_pitch_roll(object_orientation_relative_to_host)
            rot_z, rot_y, rot_x = euler_angles
            # if DEBUG and object_classes[object_index] == 'TrafficCar':
            if DEBUG:
                if z < 0.5:
                    continue
                print(f'euler_angles: {euler_angles}')
                print(f'host_name: {host_to_odometry[host_name].get_object_name()}')
                print(f'host_name: {host_to_detections[host_name].get_object_name()}')
                print(f'frame_index: {frame_index}')
                print(f'image_name: {image_name}')
                print(f'host_position: {host_position}')
                print(f'dim: {dim}')
                print(f'x: {x}, y: {y}, z: {z}')
                print(f'rotation: {euler_angles}')
                location = np.expand_dims(location, axis=0)
                uv = project_to_image(location, CALIB).flatten()
                u, v = uv
                print(f'u: {u}, v: {v}')
                cv2.circle(image, (int(u), int(v)), 10, (255, 0, 0))
                print(f'location: {location}')
                corners_3d = compute_box_3d(dim, location, rot_y+np.pi/2)
                box_2d_projected = project_to_image(corners_3d, CALIB)
                image = draw_box_3d(image, box_2d_projected.astype(np.int))
        if DEBUG:
            cv2.imshow('Detections result', image)
            cv2.waitKey(0)
            cv2.imwrite(f'./results/output_{image_name}.png', image)

if __name__ == '__main__':
    process_labels(IMAGES_PATH, ODOMETRY_PATH, DETECTION_PATH)