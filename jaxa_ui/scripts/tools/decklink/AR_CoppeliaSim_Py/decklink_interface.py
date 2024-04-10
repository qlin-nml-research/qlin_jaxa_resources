"""
Copyright (C) 2020 Murilo Marques Marinho

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later
version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program. If not,
see <https://www.gnu.org/licenses/>.
"""
from ctypes import *
from numpy.ctypeslib import ndpointer
from PIL import Image


class DeckLinkInterface:
    def __init__(self, dll_location):
        self.cpp_api = CDLL(dll_location)
        self.get_image_first_time_setup_flag = True
        self.python_image_index = 0

    def start_capture_thread(self, deck_link_index=0, display_mode_index=6, capture_interval=1, debug_mode=True):
        self.cpp_api.start_capture_thread(deck_link_index,
                                          display_mode_index,
                                          capture_interval,
                                          debug_mode)

    def is_capture_running(self):
        return self.cpp_api.is_capture_running()

    def stop_capture_thread(self):
        return self.cpp_api.stop_capture_thread()

    def get_image_index(self):
        return self.cpp_api.get_image_index()

    def get_image(self):
        try:
            cpp_image_index = self.get_image_index()
            if cpp_image_index > 0:

                # The first time we get a valid image, we get its size to correctly define the pointer in Python's side.
                if self.get_image_first_time_setup_flag:
                    self.cpp_api.get_image.restype = ndpointer(
                        dtype=c_uint,
                        shape=(self.cpp_api.get_image_height(),
                               self.cpp_api.get_image_width(),
                               4))
                    self.get_image_first_time_setup_flag = False

                # If the python index is less than the cpp index, we retrieve the image
                if self.python_image_index < cpp_image_index:
                    if self.cpp_api.image_mutex_try_lock():
                        cpp_image_data = self.cpp_api.get_image()
                        try:
                            img = Image.fromarray(cpp_image_data, 'RGBA')
                        except Exception as e:
                            print(e)
                        self.cpp_api.image_mutex_unlock()
                        self.python_image_index = cpp_image_index
                        return img
                    else:
                        return None

            # In any other situation, we return None
            return None
        except Exception as e:
            print('Error in DecklinkInterface.get_image: {}'.format(e))
