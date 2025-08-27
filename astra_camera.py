import cv2
import numpy as np
from openni import openni2
from openni import _openni2 as c_api

class Camera:
    def __init__(self, fps=30, width=640, height=480, openni_libs='libs/'):
        self.fps = fps
        self.width = width
        self.height = height
        self.openni_libs = openni_libs
        self.wait_time = int(1000.0 / float(fps))
        self.dev = None
        self.depth_stream = None
        self.color_stream = None
        self._initialized = False
        self.load()

    def __enter__(self):
        if not self._initialized:
            self.load()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.unload()

    def unload(self):
        try:
            if self.depth_stream:
                self.depth_stream.stop()
            if self.color_stream:
                self.color_stream.stop()
            openni2.unload()
        except Exception as e:
            print(f"[Camera.unload] warning during unload: {e}")
        finally:
            self._initialized = False

    def load(self):
        try:
            openni2.initialize(self.openni_libs)
            self.dev = openni2.Device.open_any()

            # Depth stream
            self.depth_stream = self.dev.create_depth_stream()
            self.depth_stream.start()
            self.depth_stream.set_video_mode(
                c_api.OniVideoMode(
                    pixelFormat=c_api.OniPixelFormat.ONI_PIXEL_FORMAT_DEPTH_100_UM,
                    resolutionX=self.width,
                    resolutionY=self.height,
                    fps=self.fps
                )
            )

            # Color stream
            self.color_stream = self.dev.create_color_stream()
            self.color_stream.start()
            self.color_stream.set_video_mode(
                c_api.OniVideoMode(
                    pixelFormat=c_api.OniPixelFormat.ONI_PIXEL_FORMAT_RGB888,
                    resolutionX=self.width,
                    resolutionY=self.height,
                    fps=self.fps
                )
            )

            # Registration
            self.dev.set_image_registration_mode(
                c_api.OniImageRegistrationMode.ONI_IMAGE_REGISTRATION_DEPTH_TO_COLOR
            )

            self._initialized = True
        except Exception as e:
            self._initialized = False
            print(f"[Camera.load] failed to initialize camera: {e}")
            raise RuntimeError(f"Camera initialization failed: {e}") from e

    def get_depth(self):
        if not self.depth_stream:
            raise RuntimeError("Depth stream not initialized")
        frame = self.depth_stream.read_frame()
        frame_data = frame.get_buffer_as_uint16()
        img = np.frombuffer(frame_data, dtype=np.uint16)
        # reshape according to actual resolution (avoid hardcode)
        img = img.reshape((self.height, self.width))
        # replicate channels if needed
        img3 = np.stack([img]*3, axis=2)  # HxWx3
        return img3

    def get_color(self):
        if not self.color_stream:
            raise RuntimeError("Color stream not initialized")
        frame = self.color_stream.read_frame()
        frame_data = frame.get_buffer_as_uint8()
        colorPix = np.frombuffer(frame_data, dtype=np.uint8)
        colorPix = colorPix.reshape((self.height, self.width, 3))
        return colorPix

    def get_depth_and_color(self):
        return self.get_depth(), self.get_color()