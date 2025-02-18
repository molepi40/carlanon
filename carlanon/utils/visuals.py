# This file utilizes existing CARLA code:

# Copyright (c) 2018 Intel Labs.
# authors: German Ros (german.ros@intel.com)
#
# This work is licensed under the terms of the MIT license.
# Permission is hereby granted, free of charge, to any person
# obtaining a copy of this software and associated documentation
# files (the "Software"), to deal in the Software without
# restriction, including without limitation the rights to use,
# copy, modify, merge, publish, distribute, sublicense, and/or
# sell copies of the Software, and to permit persons to whom the
# Software is furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be
# included in all copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
# EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
# MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
# NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
# BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
# ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
# CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

# import datetime
# import math
# import weakref

import carla
# from carla import ColorConverter as cc
# import numpy as np
# import pygame
# import cv2


# ==============================================================================
# -- CameraManager -------------------------------------------------------------
# ==============================================================================


class CameraManager(object):
    def __init__(self, world, actor, camera_type='sensor.camera.rgb', resolution_index=1, image_path=""):
        self.sensor = None
        self._surface = None
        self._actor = actor
        self._path = image_path
        self.images = []
        # transform represents a specific position and orientation relative to a vehicle
        self._camera_transforms = [
            # 3rd person view
            carla.Transform(carla.Location(x=-5.5, z=2.8), carla.Rotation(pitch=-15)),
            # dashboard view (first-person)
            carla.Transform(carla.Location(x=1.6, z=1.7)),
        ]
        self._transform_index = 1
        # image resolution
        self._resolutions = [
            ['1920', '1080'],
            ['1280', '720'],
        ]
        resolution = self._resolutions[resolution_index]
        # set camera blue print
        self._world = world
        bp_library = self._world.get_blueprint_library()
        try:
            self._camera = bp_library.find(camera_type)
            self._camera.set_attribute("image_size_x", resolution[0])
            self._camera.set_attribute("image_size_y", resolution[1])
        except:
            raise RuntimeError(
                f'can not set camera blue print for {camera_type}'
            )
        self._index = None

    def toggle_camera(self):
        self.set_transform((self._transform_index + 1) % len(self._camera_transforms))

    def set_transform(self, idx):
        self._transform_index = idx
        self.sensor.set_transform(self._camera_transforms[self._transform_index])

    def set_sensor(self, index):
        self.sensor = self._world.spawn_actor(
            self._camera,
            self._camera_transforms[self._transform_index],
            attach_to=self._actor,
        )
        # set image path
        if self._path:
            self.sensor.listen(lambda image: image.save_to_disk(f"{self._path}/%06d.png" % image.frame))
        self._index = index

    def destroy_sensor(self):
        if self.sensor is not None:
            self.sensor.stop()
            self.sensor.destroy()
