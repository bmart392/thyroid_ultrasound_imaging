

from ImageFilter import *

if __name__ == '__main__':
    image_data = ImageData(image_filepath='~/Pictures/thyroid_us.jpg')

    image_filter = ImageFilter()

    image_filter.fully_filter_image(image_data)

    image_data.plot_images()

    image_data.generate_contours_in_image()

    image_data.calculate_image_centroids()



