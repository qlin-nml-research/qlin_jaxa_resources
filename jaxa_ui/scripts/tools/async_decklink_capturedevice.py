import time
import traceback

import cv2
import multiprocessing as mp
import numpy as np
import os
import logging
from .decklink.AR_CoppeliaSim_Py.decklink_interface import DeckLinkInterface

logging.basicConfig()
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

_decklink_params = {
    "decklink_index": 1,
    "display_mode_index": -1,
    "dll_path": os.path.join(os.path.dirname(__file__),
                             "decklink/DecklinkInterface/x64/Release/DecklinkInterface.dll")
}


class _AsyncCaptureProcess_dk(mp.Process):
    def __init__(self, device_param, exit_event=None, debug=False):
        super(_AsyncCaptureProcess_dk, self).__init__()
        self.device_param = device_param
        self.image_queue = mp.Queue()
        if exit_event is None:
            exit_event = mp.Event()
        self.exit_event = exit_event
        self.error_event = mp.Event()
        self.error_event.clear()
        self.exit_event.clear()
        self.initialized_event = mp.Event()
        self.debug = debug

        # configuration queue
        self.config_queue_recv = mp.Queue()
        self.config_queue_send = mp.Queue()
        self.config_queue_send_params = mp.Queue()

    def exit(self):
        self.exit_event.set()

    def set(self, prop_id, value):
        raise NotImplementedError("_AsyncCaptureProcess_dk::set::not implemented")

    def get(self, prop_id):
        raise NotImplementedError("_AsyncCaptureProcess_dk::get::not implemented")

        # self.config_queue_send_params.put(prop_id)
        # return self.config_queue_send_params.get()

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
        cap = DeckLinkInterface(dll_location=self.device_param["dll_path"])
        cap.start_capture_thread(deck_link_index=self.device_param['decklink_index'],
                                 display_mode_index=self.device_param['display_mode_index'],
                                 capture_interval=1, debug_mode=self.debug)

        logger.info(
            f"AsyncDecklinkCapture::run::Start capturing from decklink device {self.device_param['decklink_index']}")
        if cap.is_capture_running():
            logger.info("AsyncDecklinkCapture::decklink device opened")
        else:
            logger.error(
                f"AsyncDecklinkCapture::run::Error opening decklink device {self.device_param['decklink_index']}")
            self.error_event.set()
            return

        cap_convert_arg = cv2.COLOR_RGBA2BGR
        initialized = False
        output_width, output_height = None, None
        for _ in range(100):
            if cap.is_capture_running():
                iframe = cap.get_image()
                if iframe:
                    frame_ = cv2.cvtColor(np.array(iframe), cap_convert_arg)
                    logger.info("AsyncDecklinkCapture::init::got fist frame")
                    output_width = frame_.shape[1]
                    output_height = frame_.shape[0]
                    initialized = True
                    break
            else:
                logger.info("AsyncDecklinkCapture::init::cap not running")
            time.sleep(1)
        if not initialized:
            raise RuntimeError("No frame received")

        self.initialized_event.set()
        try:
            while not self.exit_event.is_set() and cap.is_capture_running():
                # if self._has_set_request():
                #     request = self.config_queue_recv.get()
                #     cap.set(request[0], request[1])
                # if self._has_get_request():
                #     request = self.config_queue_send_params.get()
                #     value = cap.get(request)
                #     self.config_queue_send.put(value)

                iframe = cap.get_image()
                frame_ = np.array(iframe)
                if iframe:
                    if cap_convert_arg is not None:
                        frame_ = cv2.cvtColor(frame_, cap_convert_arg)

                    self.image_queue.put(frame_)
                # else:
                #     self.error_event.set()
                #     logger.error(f"AsyncDecklinkCapture::run::Error reading frame from device {self.device_param}")
                #     break

        except KeyboardInterrupt:
            logger.info(f"AsyncDecklinkCapture::run::KeyboardInterrupt")
        except Exception as e:
            logger.error(f"AsyncDecklinkCapture::run::Exception: {e}")
            traceback.print_exc()
        logger.info(f"AsyncDecklinkCapture::run::Exit called")
        cap.stop_capture_thread()


class AsyncDecklinkCapture:
    def __init__(self, device_param=None):
        if device_param is None:
            self.device_param = _decklink_params
        else:
            self.device_param = device_param
            for key, value in _decklink_params.items():
                if key not in self.device_param:
                    self.device_param[key] = value
        self.exit_event = mp.Event()
        self.exit_event.clear()
        self.async_capture_process = _AsyncCaptureProcess_dk(device_param, self.exit_event)
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
    async_video_capture = AsyncDecklinkCapture(0)
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
