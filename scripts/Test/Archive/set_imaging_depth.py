"""
File containing the set_imaging_depth function definition.
"""


# Import standard python packages

# Import custom python packages
from thyroid_ultrasound_imaging_support.ImageData.ImageData import ImageData
from thyroid_ultrasound_imaging_support.ImageManipulation.load_folder_of_saved_image_data import \
    load_folder_of_saved_image_data


def set_imaging_depth(path_to_image_data: str, new_folder_path: str):
    """"""
    for image_data_object in load_folder_of_saved_image_data(path_to_image_data):
        image_data_object: ImageData
        image_data_object.imaging_depth = 6.0
        image_data_object.save_object_to_file(new_folder_path)


if __name__ == '__main__':
    existing_path = '/home/ben/thyroid_ultrasound_data/testing_and_validation/BagFileVolumeEstimation' \
           '/RightLobe/ImageData_RightLobe'
    new_path = '/home/ben/thyroid_ultrasound_data/testing_and_validation/BagFileVolumeEstimation' \
           '/RightLobe/ImageData_RightLobe_Updated'
    set_imaging_depth(path_to_image_data=existing_path, new_folder_path=new_path)
