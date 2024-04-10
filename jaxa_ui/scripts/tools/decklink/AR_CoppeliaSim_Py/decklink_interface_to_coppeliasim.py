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


import time
import sim
from PIL import Image
from decklink_interface import DeckLinkInterface

configuration = {
    'decklinkinterface_dll_path': '../DecklinkInterface/x64/Release/DecklinkInterface',
    'ar_vision_sensor_name': 'ar_vision_sensor'
}

# Decklink Interface
decklink_interface = DeckLinkInterface(configuration['decklinkinterface_dll_path'])

if __name__ == '__main__':
    # Initialize capture device
    print("Initializing capture device...")
    decklink_interface.start_capture_thread(deck_link_index=2, debug_mode=False, display_mode_index=-1)

    # # CoppeliaSim
    # print("Initializing CoppeliaSim")
    # sim.simxFinish(-1)  # just in case, close all opened connections
    # coppeliasim_id = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)  # Connect to CoppeliaSim
    # print("Connected with id={}".format(coppeliasim_id))
    # if coppeliasim_id == -1:
    #     raise Exception('Error connection to CoppeliaSim')

    # res, ar_vision_sensor_handle = sim.simxGetObjectHandle(coppeliasim_id, configuration['ar_vision_sensor_name'],
    #                                                        sim.simx_opmode_blocking)
    # print("Found vision sensor={} with handle={}".format(configuration['ar_vision_sensor_name'], ar_vision_sensor_handle))

    # Loop
    print("Streaming DeckLink image to CoppeliaSim...")
    while True:
        img = decklink_interface.get_image()
        if img is not None:
            # Get initial time
            t0 = time.time()

            # The Alpha Channel has to be removed
            # img = img.convert(mode="RGB")
            # Image needs to be flipped to match CoppeliaSim's requirements
            # img = img.transpose(method=Image.FLIP_TOP_BOTTOM)
            # The sequence of bytes must be flattened
            # sequence_of_pixels = [item for sublist in list(img.getdata()) for item in sublist]
            # The converted image can then be sent to CoppeliaSim
            # sim.simxSetVisionSensorImage(coppeliasim_id, ar_vision_sensor_handle, sequence_of_pixels, 0, sim.simx_opmode_oneshot)
            # img.show(title='img')

            # Get final time
            t1 = time.time()
            # Print fps
            # print("(Streaming to CoppeliaSim) FPS: ", 1.0/(t1-t0))
            print('A')

# Stop capture device
print("Stopping capture device...")
decklink_interface.stop_capture_thread()
print("Capture device stopped.")
