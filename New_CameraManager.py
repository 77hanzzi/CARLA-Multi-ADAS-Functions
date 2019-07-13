
    
# ==============================================================================
# -- NewCameraManager -------------------------------------------------------------
# ==============================================================================
'''
This is a new version of NewCameraManager which supports adding different view of Supporting Sensors.
Remember to add the following codes in the world.destroy() function.


        for item in self.camera_manager.SupportingSensors:
            actors.append(item)


'''

class CameraManager(object):
    def __init__(self, parent_actor, hud):
        self.sensor = None
        self.SupportingSensors = []
        self._surface = None
        # self._surface_1 = pygame.Surface((hud.dim[0]/4,hud.dim[1]/4))
        self._parent = parent_actor
        self._hud = hud
        self._SupportSurfaceList = [pygame.Surface((hud.dim[0]/3.5,hud.dim[1]/3.5)) for i in range(1,4)]
        self._recording = False
        self._camera_transforms = [
            carla.Transform(carla.Location(x=-10, z=4.8), carla.Rotation(pitch=-17)),
            carla.Transform(carla.Location(x=1.6, z=1.7)),
            carla.Transform(carla.Location(x=0.1,y=-0.3, z=1.2), carla.Rotation(pitch=-15)),
            carla.Transform(carla.Location(x=-5.5, z=2.8), carla.Rotation(pitch=-15)),
            carla.Transform(carla.Location(x=0.2, z=1.6), carla.Rotation(pitch=0)),

            ]
        
        self._transform_index = 1
        self._sensors = [
            ['sensor.camera.rgb', cc.Raw, 'Camera RGB'],
            ['sensor.camera.depth', cc.Raw, 'Camera Depth (Raw)'],
            ['sensor.camera.depth', cc.Depth, 'Camera Depth (Gray Scale)'],
            ['sensor.camera.depth', cc.LogarithmicDepth, 'Camera Depth (Logarithmic Gray Scale)'],
            ['sensor.camera.semantic_segmentation', cc.Raw, 'Camera Semantic Segmentation (Raw)'],
            ['sensor.camera.semantic_segmentation', cc.CityScapesPalette, 'Camera Semantic Segmentation (CityScapes Palette)'],
            ['sensor.lidar.ray_cast', None, 'Lidar (Ray-Cast)']]
        self._blueprints = self._parent.get_world().get_blueprint_library()
        self._world = self._parent.get_world()
        bp_library = self._world.get_blueprint_library()
        for item in self._sensors:
            bp = bp_library.find(item[0])
            if item[0].startswith('sensor.camera'):
                bp.set_attribute('image_size_x', str(hud.dim[0]))
                bp.set_attribute('image_size_y', str(hud.dim[1]))
            elif item[0].startswith('sensor.lidar'):
                bp.set_attribute('range', '5000')
            item.append(bp)
        self._index = None

    def set_support_sensors(self):
        bp_library = self._blueprints
        bp_SupportingSensors = []
        
        for item in self._sensors:
            bp = bp_library.find(item[0])
            if item[0].startswith('sensor.camera'):
                bp.set_attribute('image_size_x', str(self._hud.dim[0]/3.5))
                bp.set_attribute('image_size_y', str(self._hud.dim[1]/3.5))
            elif item[0].startswith('sensor.lidar'):
                bp.set_attribute('range', '5000')
            bp_SupportingSensors.append(bp)
      
        Camera_1 = self._world.spawn_actor(
                            bp_SupportingSensors[5],
                            self._camera_transforms[4],
                            attach_to=self._parent)
        
        Camera_2 = self._world.spawn_actor(
                            bp_SupportingSensors[0],
                            self._camera_transforms[2],
                            attach_to=self._parent)
        
        Camera_3 = self._world.spawn_actor(
                            bp_SupportingSensors[3],
                            self._camera_transforms[4],
                            attach_to=self._parent)

        self.SupportingSensors.append(Camera_1)
        self.SupportingSensors.append(Camera_2)
        self.SupportingSensors.append(Camera_3)

        weak_self = weakref.ref(self)

        Camera_1.listen(lambda image: CameraManager._camera1_display(weak_self,image))      
        Camera_2.listen(lambda image: CameraManager._camera2_display(weak_self,image))      
        Camera_3.listen(lambda image: CameraManager._camera3_display(weak_self,image))      
    
    def toggle_camera(self):
        # This is used to change the installation location of the sensors or cameras.
        self._transform_index = (self._transform_index + 1) % len(self._camera_transforms)
        self.sensor.set_transform(self._camera_transforms[self._transform_index])

    def set_sensor(self, index, notify=True):
        self.set_support_sensors()
        index = index % len(self._sensors)
        needs_respawn = True if self._index is None \
            else self._sensors[index][0] != self._sensors[self._index][0]
        if needs_respawn:
            if self.sensor is not None:          

                self.sensor.destroy()
                self._surface = None
            self.sensor = self._parent.get_world().spawn_actor(
                self._sensors[index][-1],
                self._camera_transforms[self._transform_index],
                attach_to=self._parent)
            # We need to pass the lambda a weak reference to self to avoid
            # circular reference.
            weak_self = weakref.ref(self)
            self.sensor.listen(lambda image: CameraManager._parse_image(weak_self, image))
        if notify:
            self._hud.notification(self._sensors[index][2])
        self._index = index

    def next_sensor(self):
        self.set_sensor(self._index + 1)

    def toggle_recording(self):
        self._recording = not self._recording
        self._hud.notification('Recording %s' % ('On' if self._recording else 'Off'))

    def render(self, display):
        if self._surface is not None:
            display.blit(self._surface, (0, 0))
            # display.blit(self._SupportSurfaceList[0],(480,100))
            # display.blit(self._surface_1,(480,100))

            display.blit(self._SupportSurfaceList[0],(10,490))
            display.blit(self._SupportSurfaceList[1],(460,490))
            display.blit(self._SupportSurfaceList[2],(910,490))

    @staticmethod
    def _parse_image(weak_self, image):
        self = weak_self()
        if not self:
            return
        if self._sensors[self._index][0].startswith('sensor.lidar'):
            points = np.frombuffer(image.raw_data, dtype=np.dtype('f4'))
            points = np.reshape(points, (int(points.shape[0]/3), 3))
            lidar_data = np.array(points[:, :2])
            lidar_data *= min(self._hud.dim) / 100.0
            lidar_data += (0.5 * self._hud.dim[0], 0.5 * self._hud.dim[1])
            lidar_data = np.fabs(lidar_data)
            lidar_data = lidar_data.astype(np.int32)
            lidar_data = np.reshape(lidar_data, (-1, 2))
            lidar_img_size = (self._hud.dim[0], self._hud.dim[1], 3)
            lidar_img = np.zeros(lidar_img_size)
            lidar_img[tuple(lidar_data.T)] = (255, 255, 255)
            self._surface = pygame.surfarray.make_surface(lidar_img)
        else:
            image.convert(self._sensors[self._index][1])
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.width, 4))
            array = array[:, :, :3]
            array = array[:, :, ::-1]
            self._surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        if self._recording:
            image.save_to_disk('_out_2/%08d' % image.frame_number)

    @staticmethod
    def _camera1_display(weak_self, image):
        self = weak_self()
        if not self:
            return
        image.convert(self._sensors[5][1])
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]

        self._SupportSurfaceList[0] = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        if self._recording:
            image.save_to_disk('_out_2/%08d' % image.frame_number)

    @staticmethod
    def _camera2_display(weak_self, image):
        self = weak_self()
        if not self:
            return
        image.convert(self._sensors[0][1])
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]
        self._SupportSurfaceList[1] = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        if self._recording:
            image.save_to_disk('_out_2/%08d' % image.frame_number)

    @staticmethod
    def _camera3_display(weak_self, image):
        self = weak_self()
        if not self:
            return
        image.convert(self._sensors[3][1])
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]
        self._SupportSurfaceList[2] = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        if self._recording:
            image.save_to_disk('_out_2/%08d' % image.frame_number)