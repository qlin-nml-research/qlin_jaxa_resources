import time
import traceback

import cv2
import multiprocessing as mp
import numpy as np
import os
import logging

logging.basicConfig()
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


class _AsyncCaptureProcess(mp.Process):
    def __init__(self, device_id, exit_event=None):
        super(_AsyncCaptureProcess, self).__init__()
        self.device_id = device_id
        self.image_queue = mp.Queue()
        if exit_event is None:
            exit_event = mp.Event()
        self.exit_event = exit_event
        self.error_event = mp.Event()
        self.error_event.clear()
        self.exit_event.clear()
        self.initialized_event = mp.Event()

        # configuration queue
        self.config_queue_recv = mp.Queue()
        self.config_queue_send = mp.Queue()
        self.config_queue_send_params = mp.Queue()

    def exit(self):
        self.exit_event.set()

    def set(self, prop_id, value):
        self.config_queue_recv.put((prop_id, value))

    def get(self, prop_id):
        self.config_queue_send_params.put(prop_id)
        return self.config_queue_send_params.get()

    def has_new_image(self):
        return not self.image_queue.empty()

    def get_image(self):
        if self.image_queue.empty():
            return None
        return self.image_queue.get()

    def initialized(self):
        return self.initialized_event.is_set()

    def _has_set_request(self):
        return not self.config_queue_recv.empty()

    def _has_get_request(self):
        return not self.config_queue_send_params.empty()

    def run(self):
        cap = cv2.VideoCapture(self.device_id)
        logger.info(f"AsyncCaptureProcess::run::Start capturing from device {self.device_id}")
        if not cap.isOpened():
            logger.error(f"AsyncCaptureProcess::run::Error opening device {self.device_id}")
            self.error_event.set()
            return
        self.initialized_event.set()
        try:
            while not self.exit_event.is_set():
                if self._has_set_request():
                    request = self.config_queue_recv.get()
                    cap.set(request[0], request[1])
                if self._has_get_request():
                    request = self.config_queue_send_params.get()
                    value = cap.get(request)
                    self.config_queue_send.put(value)

                ret, frame = cap.read()
                if ret:
                    self.image_queue.put(frame)
                else:
                    self.error_event.set()
                    logger.error(f"AsyncCaptureProcess::run::Error reading frame from device {self.device_id}")
                    break
        except KeyboardInterrupt:
            logger.info(f"AsyncCaptureProcess::run::KeyboardInterrupt")
        except Exception as e:
            logger.error(f"AsyncCaptureProcess::run::Exception: {e}")
            traceback.print_exc()
        logger.info(f"AsyncCaptureProcess::run::Exit called")
        cap.release()


class AsyncVideoCapture:
    def __init__(self, device_id):
        self.device_id = device_id
        self.exit_event = mp.Event()
        self.exit_event.clear()
        self.async_capture_process = _AsyncCaptureProcess(device_id, self.exit_event)
        self.async_capture_process.start()

        logger.info(f"AsyncVideoCapture::init::initializing...")
        while not self.async_capture_process.initialized():
            time.sleep(0.01)
        logger.info(f"AsyncVideoCapture::init::initialized")

    def exit(self, timeout=0.1):
        self.exit_event.set()
        logger.info(f"AsyncVideoCapture::exit::exit called")
        self.async_capture_process.join(timeout=timeout)
        if self.async_capture_process.is_alive():
            logger.error(f"AsyncVideoCapture::exit::exit failed")
            self.async_capture_process.terminate()
        logger.info(f"AsyncVideoCapture::exit::exit completed")

    def set(self, prop_id, value):
        self.async_capture_process.set(prop_id, value)

    def get(self, prop_id):
        return self.async_capture_process.get(prop_id)

    def has_new_image(self):
        return self.async_capture_process.has_new_image()

    def read(self):
        image = self.async_capture_process.get_image()
        return image is not None, image

    def release(self):
        self.exit()
        logger.info(f"AsyncVideoCapture::release::released")


if __name__ == '__main__':
    async_video_capture = AsyncVideoCapture(0)
    try:
        while True:
            ret, frame = async_video_capture.read()
            if ret:
                cv2.imshow("frame", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    except KeyboardInterrupt:
        logger.info("KeyboardInterrupt")
    except Exception as e:
        logger.error(f"Exception: {e}")
        traceback.print_exc()

    async_video_capture.release()
    cv2.destroyAllWindows()
