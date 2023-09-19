import logging
import os
import time

import cv2
import numpy as np
import scipy.io as scio
from PyQt5.QtCore import QByteArray, qChecksum, QDataStream, QIODevice, QBuffer
from PyQt5.QtNetwork import QUdpSocket, QHostAddress

from inference_step.inference_realtime import Inferencer
from inference_step.stream_info_display import StreamInfoUI

# logger init
logging.basicConfig()
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

TARGET_DISPLAY_SIZE = [960, 540]


class TransmitDataStruct:
    _fields = [('pos', 'float', 2), ('has_lock', 'int', 1), ('focal_length', 'float', 2)]

    def __init__(self):
        for name_, dtype_, len_ in self._fields:
            setattr(self, name_, np.zeros(len_))


def realtime_stream_main_cv2(inference_param, cam_param, device_id, port, ip,
                             crop_scale, show_img=True, debug=False, info_ui=None, **kwargs):
    inference_h = Inferencer(**inference_param)
    udp_send_sock = QUdpSocket()
    udp_send_addr = QHostAddress(ip)

    enable_recording = False
    if 'recording_path' in kwargs:
        enable_recording = True
        recording_path_ = kwargs['recording_path']

    cap = cv2.VideoCapture(device_id)

    # force mac resolution
    if "target_w_resolution" in kwargs:
        print("attempt to set resolution")
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(kwargs['target_w_resolution']))
    fps = cap.get(cv2.CAP_PROP_FPS)

    if cap.isOpened():
        print("device opened")
    else:
        raise RuntimeError("Capture device not opened")

    output_dim = np.array([int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))])
    print("Current resolution", output_dim)

    # get cropping ROI setting
    roi_start = (output_dim * crop_scale).astype(np.int32)
    roi_end = (output_dim * (1 - crop_scale)).astype(np.int32)

    # camera undistort
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(cam_param['intrinsic'], cam_param['distort_coff'],
                                                      tuple(output_dim), 1)
    mapx, mapy = cv2.initUndistortRectifyMap(cam_param['intrinsic'], cam_param['distort_coff'], None, newcameramtx,
                                             tuple(output_dim), 5)

    sensor_pos_coff = cam_param['sensor_cell_size'][0] * cam_param['native_resolution'][0] / output_dim
    # sensor_pos_coff = sensor_pos_coff / cam_param['focal_length']
    # print(sensor_pos_coff)
    # print(output_dim)

    # ui process
    if info_ui is not None:
        fps_queue = info_ui.get_fps_queue()
        info_ui.start()
    else:
        fps_queue = None

    if enable_recording:
        print("recording at :", recording_path_ + "original.mp4")
        # print((fps))
        fourcc = cv2.VideoWriter_fourcc('m', 'p', '4', 'v')
        original_img_recorder = cv2.VideoWriter(recording_path_ + "original.mp4", fourcc, 6, tuple(output_dim))
        # stream = original_img_recorder.add_stream('mpeg4', rate=int(fps))  # alibi frame rate
        # stream.width = output_dim[0]
        # stream.height = output_dim[1]
        # stream.pix_fmt = 'yuv420p'
        original_f_time_writer = open(recording_path_ + "original_time.txt", 'w')
    else:
        original_img_recorder = None
        original_f_time_writer = None

    out_data = TransmitDataStruct()

    try:
        print("entering loop")
        frame_time_start = time.time()
        fps_s_time = frame_time_start
        while True:
            ret, frame = cap.read()
            if ret:
                frame_undistorted = cv2.remap(frame, mapx, mapy, cv2.INTER_LINEAR)

                # frame_undistorted = frame.copy()
                frame_show = frame_undistorted.copy()
                if enable_recording and original_img_recorder is not None:
                    original_img_recorder.write(frame_show)
                    original_f_time_writer.write('{:.6f}\n'.format(time.time() - frame_time_start))

                tip_pos = inference_h.process_frame(frame_undistorted, debug=debug, show_img=show_img,
                                                    show_img_size=TARGET_DISPLAY_SIZE)

                cv2.rectangle(frame_show, roi_start, roi_end,
                              (0, 255, 0), 2)
                cv2.imshow("current", cv2.resize(frame_show, TARGET_DISPLAY_SIZE))

                if tip_pos is not None:
                    tip_pos = (tip_pos - output_dim / 2.0) * sensor_pos_coff
                    # print(tip_pos)
                    # tip_pose
                    """
                    calculate from center of the sensor, (in meter) offset on the image plane 
                    [---------------------------]
                    [                           ]
                    [           Center          ]
                    [             x --> +x      ] o--> view into the screen with optical center behind
                    [             |             ]  tip projection on sensor
                    [             v +y          ]
                    [___________________________]
                    """
                    out_data.pos = tip_pos
                    out_data.focal_length = cam_param['focal_length'][0]
                    out_data.has_lock = [True]
                else:
                    out_data.pos = [0, 0]
                    out_data.focal_length = cam_param['focal_length'][0]
                    out_data.has_lock = [False]

                outArray = QByteArray()
                outBuffer = QBuffer(outArray)
                outBuffer.open(QIODevice.WriteOnly)
                outStream = QDataStream(outBuffer)
                outStream.setVersion(18)
                outStream.setByteOrder(QDataStream.BigEndian)
                for key_, dtype_, length_ in out_data._fields:
                    seg = getattr(out_data, key_)
                    for ind in range(length_):
                        if dtype_ == 'int':
                            outStream.writeInt32(seg[ind])
                        if dtype_ == 'float':
                            outStream.writeFloat(seg[ind])
                        if type == 'bool':
                            outStream.writeBool(seg[ind])
                outStream.writeUInt16(qChecksum(outArray))
                ret = udp_send_sock.writeDatagram(outArray, udp_send_addr, port)
                if ret < 0:
                    logger.error("::UDP send failure..")

                if info_ui is not None:
                    time_now = time.time()
                    fps = 1.0 / (time_now - fps_s_time)
                    fps_s_time = time_now
                    fps_queue.put(fps)

            else:
                break
    except KeyboardInterrupt:
        print("Exit on Interrupt")
        if info_ui is not None:
            info_ui.exit()
    # Flush stream

    if enable_recording:
        original_img_recorder.release()
        original_f_time_writer.close()
    cap.release()

    if info_ui is not None:
        info_ui.join(timeout=1)

    print("All process joined")


recording_dir_path = "E:/ExperimentData/MSCameraAutomation"
# recording_dir_path = "/home/nml/Desktop/recording"
# recording_dir_path = "D:/ComputerHome/Videos/qlin"
cw_base_path = os.path.abspath(os.path.join(os.getcwd(), "..", ))
if __name__ == '__main__':
    _show_img = True
    _debug = False
    cam_mat_file_path = "../config/cv2_correction_param.mat"
    cam_data = scio.loadmat(cam_mat_file_path)

    intrinsic_mat_ = cam_data['intrinsicMatrix']
    distortion_coeff_ = cam_data['distortionCoefficients']
    focal_length = cam_data['focalLength']
    cell_size_ = cam_data['cellSize']
    sensor_res = cam_data['sensorResolution']
    cam_param_ = {
        "intrinsic": intrinsic_mat_,
        "distort_coff": distortion_coeff_,
        # made up number
        "focal_length": focal_length,
        "object_plane_h": 0.003,
        "object_plane_w": 0.005,
        "sensor_cell_size": cell_size_,
        "native_resolution": sensor_res,
    }

    # _device_id = 2  # Mooonshot Master PC
    _device_id = 0  # Local PC
    # _device_id = vid_path  # file

    _inference_param = {
        # "model_path": os.path.join(cw_base_path, "model", 'best_model_960.pth'),
        # "model_path": os.path.join(cw_base_path, "model", 'best_model.pth'),
        "model_path": os.path.join(cw_base_path, "model", 'best_model_576.pth'),
        # "network_img_size": [960, 544],
        # "network_img_size": [768, 768],
        "network_img_size": [576, 576],
        "inference_param": {
            "CLASSES": ['drill_tip'],
            "ENCODER": "resnet18",
            "ENCODER_WEIGHTS": "imagenet",
            "ACTIVATION": "sigmoid",
            "DEVICE": "cuda:0",
            "postive_detect_threshold": 70
        },
    }

    crop_offset_scale = np.array([0.2, 0.15])

    # start UI
    info_ui_h = StreamInfoUI()

    recording_dir_path = os.path.join(recording_dir_path, "experiment_recording")
    os.makedirs(recording_dir_path, exist_ok=True)
    recording_path = os.path.join(recording_dir_path, "no_adapt_0812_exp1_vid_")
    # recording_path = os.path.join(recording_dir_path, "adapt_0812_exp1_vid_")
    # recording_path = os.path.join(recording_dir_path, "adapt_lock_R1_0812_exp1_vid_")
    # recording_path = os.path.join(recording_dir_path, "teleop_0812_exp1_vid_")

    realtime_stream_main_cv2(
        inference_param=_inference_param,
        cam_param=cam_param_,
        device_id=_device_id,
        show_img=_show_img,
        debug=_debug,
        port=21039,
        ip="10.198.113.138",
        crop_scale=crop_offset_scale,
        info_ui=info_ui_h,
        # recording_path=recording_path,
        target_w_resolution=1920,
        # target_w_resolution=3840,
    )
