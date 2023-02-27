import cv2
import numpy as np
import cv2 as cv
import io
from matplotlib import pyplot as plt
from matplotlib.patches import Rectangle, Circle
from enum import Enum


class ShapeTypes(Enum):
    RECTANGLE, CIRCLE = range(2)


class RectangleTypes(Enum):
    FLAT_CORNER, FLAT_CORNER_SQUARE, FLAT_CENTER, ANGLED_CORNER, ANGLED_CORNER_SQUARE, ANGLED_CENTER = range(6)


def draw_shape(shape_type, origin_coordinate, value, array_to_modify, rect_type=None, radius=None, width=None, height=None):
    boundaries_x = (0, 0)
    boundaries_y = (0, 0)
    if shape_type == ShapeTypes.CIRCLE:
        boundaries_x = (origin_coordinate[0] - radius, origin_coordinate[1] + radius)
        boundaries_y = (origin_coordinate[1] - radius, origin_coordinate[1] + radius)
    elif shape_type == ShapeTypes.RECTANGLE:
        if rect_type is None:
            raise Exception("Rectangle type not specified.")

        if rect_type == RectangleTypes.FLAT_CORNER:
            boundaries_x = (origin_coordinate[0], origin_coordinate[0] + width)
            boundaries_y = (origin_coordinate[1], origin_coordinate[1] + height)
        if rect_type == RectangleTypes.FLAT_CENTER:
            boundaries_x = (origin_coordinate[0] - width, origin_coordinate[1] + width)
            boundaries_y = (origin_coordinate[1] - height, origin_coordinate[1] + height)
        if rect_type == RectangleTypes.FLAT_CORNER_SQUARE:
            if (width is None and height is None) or (width is not None and height is not None and width != height):
                raise Exception("Either width of height must be specified. If both are specified they must be the same.")
            if width is None:
                dimension = height
            else:
                dimension = width
            boundaries_x = (origin_coordinate[0], origin_coordinate[0] + dimension)
            boundaries_y = (origin_coordinate[1], origin_coordinate[1] + dimension)

    else:
        raise Exception("Shape type provided was not recognized.")

    for x in range(boundaries_x[0], boundaries_x[1]):
        for y in range(boundaries_y[0], boundaries_y[1]):

            if shape_type == ShapeTypes.RECTANGLE and (rect_type == RectangleTypes.FLAT_CORNER or
                                                       rect_type == RectangleTypes.FLAT_CENTER or
                                                       rect_type == RectangleTypes.FLAT_CORNER_SQUARE):
                array_to_modify[y][x] = value
            else:
                fill_in_condition_met = False

                if shape_type == ShapeTypes.CIRCLE:
                    distance_to_center = ((x - origin_coordinate[0]) ** 2 + (y - origin_coordinate[1]) ** 2) ** .5
                    fill_in_condition_met = distance_to_center < radius

                if fill_in_condition_met:
                    array_to_modify[y][x] = value

    return array_to_modify


def draw_rectangle(rectangle_type, coordinate, width_x, height_y, value, array_to_modify):
    if rectangle_type == "corner":
        boundaries_x = (coordinate[0], coordinate[0] + width_x)
        boundaries_y = (coordinate[1], coordinate[1] + height_y)
    elif rectangle_type == "center":
        boundaries_x = (round(coordinate[0] - width_x / 2), round(coordinate[0] + width_x / 2))
        boundaries_y = (round(coordinate[1] - height_y / 2), round(coordinate[1] + height_y / 2))
    else:
        return

    for x in range(boundaries_x[0], boundaries_x[1]):
        for y in range(boundaries_y[0], boundaries_y[1]):
            array_to_modify[y][x] = value

    return array_to_modify


def draw_circle(origin, radius, value, array_to_modify):
    boundaries_x = (origin[0] - radius, origin[1] + radius)
    boundaries_y = (origin[1] - radius, origin[1] + radius)

    for x in range(boundaries_x[0], boundaries_x[1]):
        for y in range(boundaries_y[0], boundaries_y[1]):
            distance_to_center = ((x - origin[0]) ** 2 + (y - origin[1]) ** 2) ** .5
            if distance_to_center < radius:
                array_to_modify[y][x] = value

    return array_to_modify


if __name__ == "__main__":

    # TODO - Some logic should be use UserInput on first segmentation and previous mask on later iterations
    # Choose to use the mask generated from the previous segmentation
    use_previous_image_mask = True

    # Choose the type of segmentation style
    use_mask = True

    # Choose which image to use
    use_test_image = False

    # Read the image
    if use_test_image:
        raw_image = cv.imread('Test.png')
    else:
        raw_image = cv.imread('thyroid_ultrasound.png')

    print("Image Width: " + str(raw_image.shape[1]))
    print("Image Height: " + str(raw_image.shape[0]))

    # Check that the image was successfully read
    assert raw_image is not None, "File could not be read, check with os.path.exists()"

    # Set the number of columns in the plot
    plot_cols = int(2)

    # Save titles of all plots
    plot_titles = ["Original Image",
                   "Initialized Image",
                   "Segmentation Rectangle",
                   "Resulting Mask",
                   "Foreground of Image",
                   "Background of Image",
                   "Sure Foreground Mask",
                   "Sure Background Mask",
                   "Sure Foreground Image",
                   "Sure Background Image"
                   ]

    # Set the number of rows of plots
    plot_rows = int(len(plot_titles) / plot_cols)

    # Create a figure with 4 sub-plots
    figure_window, axes = plt.subplots(plot_rows, plot_cols)

    # Add titles and labels
    for row in range(plot_rows):
        for col in range(plot_cols):
            i = plot_cols * row + col
            axes[row, col].set_title(plot_titles[i])
            axes[row, col].set_xlabel("X")
            axes[row, col].set_ylabel("Y")

    # Recolor the image to properly show the original image
    recolored_image = cv.cvtColor(raw_image, cv.COLOR_BGR2RGB)

    # Plot the original image
    axes[0, 0].imshow(recolored_image)

    # Grayscale the original image
    colorized_image = cv.cvtColor(raw_image, cv.COLOR_BGR2GRAY)

    if use_test_image:

        # Define the coordinates of the rectangle
        # Coordinates are defined in the form of (x_origin, y_origin, width, height)
        initialized_rectangle_cords = (750, 300, 1450 - 750, 950 - 300, 'yellow')

        # Define each of the rectangles in the mask
        # Add the rectangle coordinates to the list of rectangles
        mask_rectangles = [# (750, 300, 1450 - 750, 950 - 300, 'blue', cv2.GC_PR_FGD),
                          (820, 350, 900 - 820, 430 - 350, 'red', cv2.GC_BGD),
                          (1350, 350, 1420 - 1350, 430 - 350, 'red', cv2.GC_BGD),
                          (820, 820, 900 - 820, 900 - 820, 'red', cv2.GC_BGD),
                          (1350, 820, 1420 - 1350, 900 - 820, 'red', cv2.GC_BGD),
                          (1000, 500, 1300 - 1000, 800 - 500, 'green', cv2.GC_FGD)
                          ]
    else:
        # Thyroid Image

        # Calculate the fractional image width and height
        fractional_image_width = int(np.round(raw_image.shape[1] / 20, 0))
        fractional_image_height = int(np.round(raw_image.shape[0] / 20, 0))

        # Define corners of the rectangle relative to the size of the image
        # Coordinates are defined in the form of (x_origin, y_origin, width, height)
        initialized_rectangle_cords = (fractional_image_width * 2, fractional_image_height * 2,
                                       fractional_image_width * 16, fractional_image_height * 15, 'red')

        # Define each of the rectangles in the mask
        # Add the rectangle coordinates to the list of rectangles
        mask_rectangles = [# (ShapeTypes.RECTANGLE, RectangleTypes.FLAT_CORNER, 170, 110, 300 - 170, 260 - 110, None, 'red', cv2.GC_BGD),
                           (ShapeTypes.CIRCLE, None, 240, 150, None, None, 70, 'red', cv2.GC_BGD),
                           (ShapeTypes.RECTANGLE, RectangleTypes.FLAT_CORNER, 450, 0, 500 - 450, 272 - 0, None, 'red', cv2.GC_BGD),
                           (ShapeTypes.RECTANGLE, RectangleTypes.FLAT_CORNER, 50, 10, 140 - 50, 50 - 10, None, 'red', cv2.GC_BGD),
                           (ShapeTypes.RECTANGLE, RectangleTypes.FLAT_CORNER, 350, 10, 430 - 350, 70 - 10, None, 'red', cv2.GC_BGD),
                           (ShapeTypes.RECTANGLE, RectangleTypes.FLAT_CORNER, 70, 120, 120 - 70, 180 - 120, None, 'green', cv2.GC_FGD),
                           (ShapeTypes.RECTANGLE, RectangleTypes.FLAT_CORNER, 335, 110, 385 - 335, 175 - 110, None, 'green', cv2.GC_FGD)]

    # Update the mask with the values
    initialized_mask = np.ones(raw_image.shape[:2], np.uint8) * cv2.GC_PR_BGD
    for shape in mask_rectangles:
        initialized_mask = draw_shape(shape_type=shape[0],
                                      rect_type=shape[1],
                                      origin_coordinate=(shape[2], shape[3]),
                                      width=shape[4],
                                      height=shape[5],
                                      radius=shape[6],
                                      value=shape[8],
                                      array_to_modify=initialized_mask)

    # Print the size of the initialized rectangle coordinates
    print("Initialized Rectangle origin coordinates: " + str(initialized_rectangle_cords[0]) + ", " +
          str(initialized_rectangle_cords[1]))
    print("Initialized Rectangle width: " + str(initialized_rectangle_cords[2]))
    print("Initialized Rectangle height: " + str(initialized_rectangle_cords[3]))

    # Plot the raw image on the next subplot
    axes[0, 1].imshow(recolored_image)

    # Overlay the initialized rectangle on the original image
    axes[0, 1].add_patch(
        Rectangle((initialized_rectangle_cords[0], initialized_rectangle_cords[1]),
                  initialized_rectangle_cords[2],
                  initialized_rectangle_cords[3],
                  fill=False, edgecolor=initialized_rectangle_cords[4]))

    # Plot the raw image on the next subplot
    axes[1, 0].imshow(recolored_image)

    # Plot each of the mask_rectangles on the image
    for shape in mask_rectangles:
        if shape[0] == ShapeTypes.CIRCLE:
            new_patch = Circle((shape[2], shape[3]),
                               shape[6],
                               fill=False,
                               edgecolor=shape[7])

        elif shape[0] == ShapeTypes.RECTANGLE:
            new_patch = Rectangle((shape[2], shape[3]),
                                  shape[4],
                                  shape[5],
                                  fill=False,
                                  edgecolor=shape[7])
        else:
            raise Exception("Shape type not recognized.")

        axes[1, 0].add_patch(new_patch)

    # Define background and foreground models for grabcut algorithm
    bgdModel = np.zeros((1, 65), np.float64)
    fgdModel = np.zeros((1, 65), np.float64)

    if not use_mask:
        # Create an empty mask array
        mask_to_use = np.zeros(raw_image.shape[:2], np.uint8)
        method = cv2.GC_INIT_WITH_RECT
    else:
        # Use the mask initialized above
        mask_to_use = initialized_mask
        method = cv2.GC_INIT_WITH_MASK

    previous_image_mask = np.load('previous_image_mask.npy')
    if previous_image_mask is not None and use_previous_image_mask:
        mask_to_use = previous_image_mask

    # TODO - Check how number of iterations affects segmentation time
    # TODO - First segmentation needs less iterations than later segmentations
    # Segment the image RGB image (Image must be three channel image)
    cv.grabCut(img=raw_image, mask=mask_to_use, rect=initialized_rectangle_cords[:4], bgdModel=bgdModel,
               fgdModel=fgdModel, iterCount=10, mode=method)

    # Convert the mask to a true binary masking
    clean_mask = np.where((mask_to_use == cv2.GC_PR_BGD) | (mask_to_use == cv2.GC_BGD), 0, 1).astype('uint8')

    # Plot the mask generated by the segmentation as a BGR image
    axes[1, 1].imshow(mask_to_use)

    # Mask the recolored image using the clean mask
    segmented_image = recolored_image * clean_mask[:, :, np.newaxis]

    # Plot the segmented image
    axes[2, 0].imshow(segmented_image)

    # Plot the inverse fo the segmented image
    axes[2, 1].imshow(recolored_image * (1 - clean_mask)[:, :, np.newaxis])

    # Create masks for next iteration
    sure_foreground_mask = cv2.morphologyEx(clean_mask, cv2.MORPH_ERODE, np.ones(10, np.uint8), iterations=2)
    sure_background_mask = 1 - cv2.morphologyEx(clean_mask, cv2.MORPH_DILATE, np.ones(10, np.uint8), iterations=15)

    # Plot the next masks
    axes[3, 0].imshow(sure_foreground_mask * 255)
    axes[3, 1].imshow(sure_background_mask * 255)

    # Plot the images created by the next mask
    axes[4, 0].imshow(recolored_image * sure_foreground_mask[:, :, np.newaxis])
    axes[4, 1].imshow(recolored_image * sure_background_mask[:, :, np.newaxis])

    # Save masks for next iteration
    np.save('previous_image_mask', (sure_background_mask * cv2.GC_BGD) + (sure_foreground_mask * cv2.GC_FGD))
    np.save('sure_foreground_mask', sure_foreground_mask * cv2.GC_FGD)
    np.save('sure_background_mask', sure_background_mask * cv2.GC_BGD)

    # Show the resulting figure
    plt.show()

    # Code to save a matplotlib figure as an array
    # This code came from: https://stackoverflow.com/questions/7821518/matplotlib-save-plot-to-numpy-array by JUN_NETWORKS
    # Use this code to create a masking array from patches drawn on the figure and then segment the image
    #with io.BytesIO() as buff:
    #    axes[0, 0].savefig(buff, format='raw')
    #    buff.seek(0)
    #    data = np.frombuffer(buff.getvalue(), dtype=np.uint8)
    #w, h = axes[0, 0].canvas.get_width_height()
    #im = data.reshape((int(h), int(w), -1))
