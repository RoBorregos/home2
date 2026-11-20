"""Names and locations shared by gallery_build, the matcher, the registry and fetch_models.

Pure strings: keep it free of heavy imports.
"""

# Built gallery; sits beside detectors/registry.py at runtime (its "gallery_dir").
GALLERY_DIRNAME = "gallery"
MANIFEST_NAME = "manifest.json"
PHOTOS_DIRNAME = "gallery_photos"  # one folder of enrollment photos per object
CROPS_DIRNAME = "_crops"  # the crop taken from each photo, kept for review
PHOTO_EXTENSIONS = (".jpg", ".jpeg", ".png")  # matched case-insensitively
