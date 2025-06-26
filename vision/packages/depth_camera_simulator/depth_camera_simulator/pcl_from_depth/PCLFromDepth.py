import numpy as np


class PCLFromDepth:
    def __init__(self, width, height):
        """
        Initialize with image width and height.
        width, height: image dimensions
        """
        self.width = width
        self.height = height

    def generate_pointcloud(self, rgb_image, depth_image):
        """
        Generate a point cloud from RGB and depth images using uncalibrated pinhole model.
        Args:
            rgb_image: HxWx3 uint8 numpy array
            depth_image: HxW float or uint16 numpy array (depth in meters)
        Returns:
            Nx6 numpy array (x, y, z, r, g, b)
        """
        h, w = depth_image.shape
        i, j = np.indices((h, w))
        z = depth_image.astype(np.float32)

        # Assume fx = fy = focal length = width (arbitrary, uncalibrated)
        fx = fy = float(self.width)
        cx = self.width / 2.0
        cy = self.height / 2.0

        x = (j - cx) * z / fx
        y = (i - cy) * z / fy

        mask = z > 0
        x = x[mask]
        y = y[mask]
        z = z[mask]
        colors = rgb_image[mask]

        pointcloud = np.column_stack((x, y, z, colors))
        return pointcloud
