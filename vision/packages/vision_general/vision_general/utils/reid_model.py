from vision_general.utils.models.swin.model import ft_net_swin, ft_net, ft_net_dense
from torchvision import transforms
import math
import os
import yaml
import torch
import torch.nn as nn
from PIL import Image
from scipy.spatial.distance import cosine
import pathlib

version = torch.__version__
use_swin = True
use_dense = False
epoch = "last"
linear_num = 512
batch_size = 256
folder_path = str(pathlib.Path(__file__).parent)

# Resolve model weights from the source tree via the package file location.
# Works with --symlink-install (symlinks back to source) and direct execution.
_models_root = str(pathlib.Path(__file__).resolve().parent / "models")

use_gpu = torch.cuda.is_available()
# Enable FP16 for Orin AGX — halves memory and doubles throughput
use_fp16 = use_gpu and torch.cuda.get_device_capability()[0] >= 7
gpu_ids = [0]
ms = []
ms.append(math.sqrt(float(1)))

if use_swin:
    h, w = 224, 224
    name = "swin"

else:
    h, w = 256, 128
    name = "ft_net_dense"

interpolation_mode = transforms.InterpolationMode.BICUBIC

data_transforms = transforms.Compose(
    [
        transforms.Resize((h, w), interpolation=interpolation_mode),
        transforms.ToTensor(),
        transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225]),
    ]
)


config_path = os.path.join(_models_root, name, "opts.yaml")
with open(config_path, "r") as stream:
    config = yaml.load(
        stream, Loader=yaml.FullLoader
    )  # for the new pyyaml via 'conda install pyyaml'

if "nclasses" in config:  # tp compatible with old config files
    nclasses = config["nclasses"]
else:
    nclasses = 751

if "ibn" in config:
    ibn = config["ibn"]

if "linear_num" in config:
    linear_num = config["linear_num"]

if linear_num <= 0 and (use_swin or use_dense):
    linear_num = 1024

stride = config["stride"]


def get_structure():
    if use_swin:
        model_structure = ft_net_swin(nclasses, stride=stride, linear_num=linear_num)
    elif use_dense:
        model_structure = ft_net_dense(nclasses, stride=stride, linear_num=linear_num)
    else:
        model_structure = ft_net(
            nclasses, stride=stride, ibn=ibn, linear_num=linear_num
        )

    return model_structure


def load_network(network):
    save_path = os.path.join(_models_root, name, "net_%s.pth" % epoch)
    try:
        if use_gpu:
            print("Loading model from GPU")
            network.load_state_dict(torch.load(save_path))
        else:
            network.load_state_dict(
                torch.load(save_path, map_location=torch.device("cpu"))
            )
    except Exception as e:
        print(e)
        if (
            use_gpu
            and torch.cuda.get_device_capability()[0] > 6
            and len(gpu_ids) == 1
            and int(version[0]) > 1
        ):  # should be >=7
            print("Compiling model...")
            torch.set_float32_matmul_precision("high")
            network = torch.compile(
                network, mode="default", dynamic=True
            )  # pytorch 2.0
        if use_gpu:
            network.load_state_dict(torch.load(save_path))
        else:
            network.load_state_dict(
                torch.load(save_path, map_location=torch.device("cpu"))
            )

    return network


def extract_feature_from_img(image, model):
    model.eval()
    if use_gpu:
        image = data_transforms(image).unsqueeze(0).cuda()
        with torch.no_grad(), torch.cuda.amp.autocast(enabled=use_fp16):
            features = torch.zeros(1, linear_num, device="cuda")
            input_img = image
            for scale in ms:
                if scale != 1:
                    input_img = torch.nn.functional.interpolate(
                        input_img,
                        scale_factor=scale,
                        mode="bicubic",
                        align_corners=False,
                    )
                features += model(input_img)
            features /= torch.norm(features, p=2, dim=1, keepdim=True)
        return features
    else:
        image = data_transforms(image)
        with torch.no_grad():
            features = torch.zeros(linear_num)
            for i in range(2):
                if i == 1:
                    image = torch.flip(image, dims=[2])
                input_img = image.unsqueeze(0)
                for scale in ms:
                    if scale != 1:
                        input_img = torch.nn.functional.interpolate(
                            input_img,
                            scale_factor=scale,
                            mode="bicubic",
                            align_corners=False,
                        )
                    features += model(input_img).squeeze()
            features /= torch.norm(features, p=2, dim=0)
        return features.cpu()


def compare_images(features1, features2, threshold=0.55):
    # if features1.ndim != 1 or features2.ndim != 1:
    #     print("error comparing images")
    #     return False

    # # Compute Euclidean distance
    # distance = np.linalg.norm(features1 - features2)
    # print(distance)

    # return distance <= threshold

    # Compare with threshold
    if features1.ndim != 1 or features2.ndim != 1:
        if use_gpu:
            features1 = features1.squeeze()
            features2 = features2.squeeze()
        else:
            print("error comparing images")
            return False

    # Compute cosine similarity between feature vectors
    # features1 = features1.reshape(features1.shape[0], -1)
    # features2 = features2.reshape(features2.shape[0], -1)
    if not use_gpu:
        similarity_score = 1 - cosine(features1, features2)
    else:
        similarity_score = torch.nn.functional.cosine_similarity(
            features1, features2, dim=0
        ).item()

    # Compare similarity score with threshold
    print(f"Similarity score: {similarity_score}")
    if similarity_score >= threshold:
        print("REIDENTIFIED____________")
        return True  # Images are considered to be of the same person
    else:
        return False  # Images are considered to be of different persons


# Test
if __name__ == "__main__":
    print("Test")
    model_structure = get_structure()
    model = load_network(model_structure)
    model.classifier.classifier = nn.Sequential()

    if use_gpu:
        model = model.cuda()

    with torch.no_grad():
        image = Image.open(f"{folder_path}/angle_test_images/1.jpeg").convert("RGB")
        features1 = extract_feature_from_img(image, model)
        image2 = Image.open(f"{folder_path}/angle_test_images/1.jpeg").convert("RGB")
        features2 = extract_feature_from_img(image2, model)
    is_same_person = compare_images(features1, features2, threshold=0.7)

    if is_same_person:
        print("The images are of the same person.")
    else:
        print("The images are of different persons.")
