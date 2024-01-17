# Import standard python packages
from _csv import reader
from csv import reader, DictReader

from matplotlib.pyplot import scatter, show, title, xlabel, ylabel, gca, xlim, ylim, figure

# Import custom python packages
from experimentation_column_headers import TEST_NUM, IMAGE_NUM, INITIAL_MASK_DICE_SCORE, DOWN_SAMPLING_RATE, \
    NUM_ITERATIONS, PROB_BKGD_EXP_FACTOR, SEGMENTATION_TIME_MEDIAN, SEGMENTATION_TIME_AVERAGE, \
    SEGMENTATION_TIME_STD_DEV, DICE_SCORE_MEDIAN, DICE_SCORE_AVERAGE, DICE_SCORE_STD_DEV

# Define the location of the CSV file that should be opened
CSV_SOURCE = '/home/ben/thyroid_ultrasound/src/thyroid_ultrasound_imaging/scripts' \
             '/Test/Experimentation/Experiment_2024-01-12/Results/CSV/Results_2024-01-12_15-26-04-722455.csv'

# Define a dictionary to store the results in
results_dictionary = {}

# Open the excel sheet
source_file = open(CSV_SOURCE, mode='r')
source_file_reader = DictReader(source_file)

# For each row in the sheet,
for row in source_file_reader:

    # Save the image number of each entry
    image_number = int(row[IMAGE_NUM])

    # If the image number is already in the dictionary
    if image_number not in results_dictionary.keys():

        # create a new dictionary
        results_dictionary[image_number] = {TEST_NUM: [], DICE_SCORE_MEDIAN: [], SEGMENTATION_TIME_MEDIAN: []}

    # add the values to the dictionary here
    results_dictionary[image_number][TEST_NUM].append(int(row[TEST_NUM]))
    results_dictionary[image_number][DICE_SCORE_MEDIAN].append(float(row[DICE_SCORE_MEDIAN]))
    results_dictionary[image_number][SEGMENTATION_TIME_MEDIAN].append(float(row[SEGMENTATION_TIME_MEDIAN]))

scatter(results_dictionary[15][DICE_SCORE_MEDIAN], results_dictionary[15][SEGMENTATION_TIME_MEDIAN])
title("DICE Score vs Segmentation Time")
xlabel(DICE_SCORE_MEDIAN)
ylabel(SEGMENTATION_TIME_MEDIAN)
xlim(0.75, 1.0)
ylim(0, 30)
ax = gca()
for i, text in enumerate(results_dictionary[15][TEST_NUM]):
    ax.annotate(text, (results_dictionary[15][DICE_SCORE_MEDIAN][i], results_dictionary[15][SEGMENTATION_TIME_MEDIAN][i]))
# ax.set_xlimits([0.60, 1.00])
# ax.set_ylimits([0, 50])
show()


