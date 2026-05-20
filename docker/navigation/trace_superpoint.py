"""Trace SuperPoint model to TorchScript for RTABMap.

RTABMap's C++ SuperPoint expects a raw tensor [1,1,H,W] input
returning (scores, descriptors), not the dict-based SuperGlue interface.
This wrapper loads the SuperGlue repo's weights and re-exports them
in the format RTABMap expects.
"""

import torch
import torch.nn as nn
import torch.nn.functional as F
import sys

sys.path.insert(0, "/home/ros/SuperGlue")
from models.superpoint import SuperPoint


class SPWrapper(nn.Module):
    def __init__(self):
        super().__init__()
        cfg = {"nms_radius": 4, "keypoint_threshold": 0.005, "max_keypoints": -1}
        sp = SuperPoint(cfg).eval()
        self.relu = sp.relu
        self.conv1a, self.conv1b = sp.conv1a, sp.conv1b
        self.conv2a, self.conv2b = sp.conv2a, sp.conv2b
        self.conv3a, self.conv3b = sp.conv3a, sp.conv3b
        self.conv4a, self.conv4b = sp.conv4a, sp.conv4b
        self.convPa, self.convPb = sp.convPa, sp.convPb
        self.convDa, self.convDb = sp.convDa, sp.convDb

    def forward(self, image):
        x = self.relu(self.conv1a(image))
        x = self.relu(self.conv1b(x))
        x = F.max_pool2d(x, 2, 2)
        x = self.relu(self.conv2a(x))
        x = self.relu(self.conv2b(x))
        x = F.max_pool2d(x, 2, 2)
        x = self.relu(self.conv3a(x))
        x = self.relu(self.conv3b(x))
        x = F.max_pool2d(x, 2, 2)
        x = self.relu(self.conv4a(x))
        x = self.relu(self.conv4b(x))
        cPa = self.relu(self.convPa(x))
        scores = self.convPb(cPa)
        cDa = self.relu(self.convDa(x))
        desc = self.convDb(cDa)
        desc = F.normalize(desc, p=2, dim=1)
        return scores, desc


if __name__ == "__main__":
    wrapper = SPWrapper().eval().cpu()
    traced = torch.jit.trace(wrapper, torch.randn(1, 1, 480, 640))
    traced.save("/home/ros/models/superpoint_v1.pt")
    print("SuperPoint model traced successfully")
