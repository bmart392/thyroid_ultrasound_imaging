# Import standard python packages
from csv import DictReader

from matplotlib.pyplot import show, subplots

# Import custom python packages
from experimentation_column_headers import TEST_NUM, IMAGE_NUM, INITIAL_MASK_DICE_SCORE, DOWN_SAMPLING_RATE, \
    NUM_ITERATIONS, PROB_BKGD_EXP_FACTOR, SEGMENTATION_TIME_MEDIAN, SEGMENTATION_TIME_AVERAGE, \
    SEGMENTATION_TIME_STD_DEV, DICE_SCORE_MEDIAN, DICE_SCORE_AVERAGE, DICE_SCORE_STD_DEV

# Define the location of the CSV file that should be opened
INITIAL_MASK_DICE_SCORE_SOURCE = '/home/ben/thyroid_ultrasound/src/thyroid_ultrasound_imaging/scripts' \
                                 '/Test/Experimentation/Experiment_2024-01-12/Results/CSV/' \
                                 'Results_2024-01-18_17-20-21-668102.csv'
DOWN_SAMPLING_RATE_SOURCE = '/home/ben/thyroid_ultrasound/src/thyroid_ultrasound_imaging/scripts' \
                            '/Test/Experimentation/Experiment_2024-01-12/Results/CSV/' \
                            'Results_2024-01-18_17-23-12-688947.csv'
NUM_ITERATIONS_SOURCE = '/home/ben/thyroid_ultrasound/src/thyroid_ultrasound_imaging/scripts' \
                        '/Test/Experimentation/Experiment_2024-01-12/Results/CSV/Results_2024-01-18_17-25-01-577723.csv'
PROB_BKGD_EXP_FACTOR_SOURCE = '/home/ben/thyroid_ultrasound/src/thyroid_ultrasound_imaging/scripts' \
                              '/Test/Experimentation/Experiment_2024-01-12/Results/CSV/' \
                              'Results_2024-01-18_17-28-42-328135.csv'

# Create a figure and axes to plot the results
fig, axes = subplots(nrows=4, ncols=2)

# Define a counter to determine which plot to plot on
axis_counter = 0

# Title the figure
fig.suptitle('Affect of Individual Factors on Segmentation Score')

for csv_source, selected_data in zip([INITIAL_MASK_DICE_SCORE_SOURCE, DOWN_SAMPLING_RATE_SOURCE,
                                      NUM_ITERATIONS_SOURCE, PROB_BKGD_EXP_FACTOR_SOURCE],
                                     [INITIAL_MASK_DICE_SCORE, DOWN_SAMPLING_RATE,
                                      NUM_ITERATIONS, PROB_BKGD_EXP_FACTOR]):

    # Define a dictionary to store the results in
    results_dictionary = {}

    # Open the excel sheet
    source_file = open(csv_source, mode='r')
    source_file_reader = DictReader(source_file)

    # For each row in the sheet,
    for row in source_file_reader:

        # Save the image number of each entry
        image_number = int(row[IMAGE_NUM])

        # If the image number is already in the dictionary
        if image_number not in results_dictionary.keys():
            # create a new dictionary
            results_dictionary[image_number] = {TEST_NUM: [],
                                                INITIAL_MASK_DICE_SCORE: [],
                                                DOWN_SAMPLING_RATE: [],
                                                NUM_ITERATIONS: [],
                                                PROB_BKGD_EXP_FACTOR: [],
                                                SEGMENTATION_TIME_MEDIAN: [],
                                                SEGMENTATION_TIME_AVERAGE: [],
                                                SEGMENTATION_TIME_STD_DEV: [],
                                                DICE_SCORE_MEDIAN: [],
                                                DICE_SCORE_AVERAGE: [],
                                                DICE_SCORE_STD_DEV: [],
                                                }

        # add the values to the dictionary here
        results_dictionary[image_number][TEST_NUM].append(int(row[TEST_NUM]))
        results_dictionary[image_number][INITIAL_MASK_DICE_SCORE].append(float(row[INITIAL_MASK_DICE_SCORE]))
        results_dictionary[image_number][DOWN_SAMPLING_RATE].append(float(row[DOWN_SAMPLING_RATE]))
        results_dictionary[image_number][NUM_ITERATIONS].append(int(row[NUM_ITERATIONS]))
        results_dictionary[image_number][PROB_BKGD_EXP_FACTOR].append(float(row[PROB_BKGD_EXP_FACTOR]))
        results_dictionary[image_number][SEGMENTATION_TIME_MEDIAN].append(float(row[SEGMENTATION_TIME_MEDIAN]))
        results_dictionary[image_number][SEGMENTATION_TIME_AVERAGE].append(float(row[SEGMENTATION_TIME_AVERAGE]))
        results_dictionary[image_number][SEGMENTATION_TIME_STD_DEV].append(float(row[SEGMENTATION_TIME_STD_DEV]))
        results_dictionary[image_number][DICE_SCORE_MEDIAN].append(float(row[DICE_SCORE_MEDIAN]))
        results_dictionary[image_number][DICE_SCORE_AVERAGE].append(float(row[DICE_SCORE_AVERAGE]))
        results_dictionary[image_number][DICE_SCORE_STD_DEV].append(float(row[DICE_SCORE_STD_DEV]))

    # For the results from each image, plot the corresponding data
    for image_number_to_plot in results_dictionary.keys():
        # Plot the scatter plot for each data set
        axes[axis_counter][0].scatter(results_dictionary[image_number_to_plot][selected_data],
                                      results_dictionary[image_number_to_plot][SEGMENTATION_TIME_MEDIAN])
        axes[axis_counter][0].set_xlabel(selected_data)
        axes[axis_counter][0].set_ylabel(SEGMENTATION_TIME_MEDIAN)
        axes[axis_counter][1].scatter(results_dictionary[image_number_to_plot][selected_data],
                                      results_dictionary[image_number_to_plot][DICE_SCORE_MEDIAN])
        axes[axis_counter][1].set_xlabel(selected_data)
        axes[axis_counter][1].set_ylabel(DICE_SCORE_MEDIAN)

        # Add the data labels to each graph
        for i, text in enumerate(results_dictionary[image_number_to_plot][TEST_NUM]):
            axes[axis_counter][0].annotate(text, (results_dictionary[image_number_to_plot][selected_data][i],
                                                  results_dictionary[image_number_to_plot][SEGMENTATION_TIME_MEDIAN][
                                                      i]))
            axes[axis_counter][1].annotate(text, (results_dictionary[image_number_to_plot][selected_data][i],
                                                  results_dictionary[image_number_to_plot][DICE_SCORE_MEDIAN][i]))

    # Update the counter to plot on the next row
    axis_counter = axis_counter + 1

# Display the graphs
show()
