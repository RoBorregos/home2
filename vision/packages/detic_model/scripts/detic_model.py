import cv2
import os
import sys
from detectron2.utils.logger import setup_logger
from detectron2.engine import DefaultPredictor
from detectron2.config import get_cfg
from detectron2.utils.visualizer import Visualizer
from detectron2.data import MetadataCatalog
sys.path.insert(0, 'Detic/third_party/CenterNet2/')
sys.path.insert(0, 'Detic/')
from centernet.config import add_centernet_config
from detic.config import add_detic_config
from detic.modeling.utils import reset_cls_test


class DeticModel:
    def __init__(
        self, config_path, weights_path, classifier_path, metadata_key, score_thresh=0.5
    ):
        """
        Initialize the Detic model with the given configuration and weights.
        """
        setup_logger()
        self.cfg = get_cfg()
        add_centernet_config(self.cfg)
        add_detic_config(self.cfg)
        self.cfg.merge_from_file(config_path)
        self.cfg.MODEL.WEIGHTS = weights_path
        self.cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = score_thresh
        self.cfg.MODEL.ROI_BOX_HEAD.ZEROSHOT_WEIGHT_PATH = "rand"
        self.cfg.MODEL.ROI_HEADS.ONE_CLASS_PER_PROPOSAL = True

        # Set up metadata and classifier
        self.metadata = MetadataCatalog.get(metadata_key)
        self.num_classes = len(self.metadata.thing_classes)
        reset_cls_test(
            DefaultPredictor(self.cfg).model, classifier_path, self.num_classes
        )

        # Initialize predictor
        self.predictor = DefaultPredictor(self.cfg)

    def load_image(self, image_path):
        """
        Load an image from the given path.
        """
        if not os.path.exists(image_path):
            raise FileNotFoundError(f"Image not found at {image_path}")
        return cv2.imread(image_path)

    def run_inference(self, image):
        """
        Run inference on the given image and return the visualization.
        """
        outputs = self.predictor(image)
        visualizer = Visualizer(image[:, :, ::-1], self.metadata)
        out = visualizer.draw_instance_predictions(outputs["instances"].to("cpu"))
        return out.get_image()[:, :, ::-1]


detic = DeticModel(
    config_path="Detic/configs/Detic_LCOCOI21k_CLIP_SwinB_896b32_4x_ft4x_max-size.yaml",
    weights_path="Detic_LCOCOI21k_CLIP_SwinB_896b32_4x_ft4x_max-size.pth",
    classifier_path="Detic/datasets/metadata/lvis_v1_clip_a+cname.npy",
    metadata_key="lvis_v1_val",
)
image = detic.load_image("path/to/image.jpg")
result = detic.run_inference(image)
cv2.imshow("Result", result)
cv2.waitKey(0)
