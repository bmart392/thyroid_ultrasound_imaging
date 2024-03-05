import plotly.express as ex
from thyroid_ultrasound_imaging_support.ImageManipulation.load_folder_of_image_files import load_folder_of_image_files

all_images = load_folder_of_image_files('/home/ben/thyroid_ultrasound_data/testing_and_validation/'
                                        'raw_images/2023-11-29_19-14')


for image in all_images:
    fig = ex.imshow(img=image)
    fig.show()
    print("shown")
