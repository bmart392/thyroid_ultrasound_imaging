#!/usr/bin/env python3

"""
File containing ExperimentControlApplication class.
"""

# Import standard packages
from tkinter import *
import tkinter.ttk as ttk

# Import ROS packages
from rospy import init_node, Publisher, Subscriber
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Bool, String

# Define constants for GUI elements
START_IMAGE_FILTER: str = "Start Image Filtering"
STOP_IMAGE_FILTER: str = "Stop Image Filtering"
START_FORCE_CONTROL: str = "Start Force Control"
STOP_FORCE_CONTROL: str = "Stop Force Control"
START_ROBOT_MOVEMENT: str = "Start Robot Commands"
STOP_ROBOT_MOVEMENT: str = "Stop Robot Commands"

# Define constants for widget states
WIDGET_STATE_ENABLED: str = "normal"
WIDGET_STATE_DISABLED: str = "disable"

# Define constants for parameters of widgets
WIDGET_TEXT: str = 'text'
WIDGET_STATE: str = 'state'

# Define grid geometry constants
FULL_WIDTH: int = int(3)
TWO_COLUMN: int = int(2)
SINGLE_COLUMN: int = int(1)
LEFT_COLUMN: int = int(0)
MIDDLE_COLUMN: int = int(1)
RIGHT_COLUMN: int = int(2)

# Define empty status string
EMPTY_STATUS: str = "STATUS: "


class ExperimentControlApplication(Frame):
    def __init__(self, parent=None):
        super().__init__()
        self.parent = parent

        # ---------------------------------------
        # Create ROS components of the User Interface
        # ---------------------------------------

        # Startup the node
        init_node('gui')

        # Create a subscriber to listen to the external force felt by the robot
        self.robot_sensed_force_subscriber = Subscriber('/franka_state_controller/F_ext', WrenchStamped,
                                                        self.robot_sensed_force_callback)

        # Create a subscriber to hear debug messages
        self.debug_status_messages_subscriber = Subscriber('/debug/status_messages', String,
                                                           self.debug_status_messages_callback)

        # Create a publisher to publish the command to start and stop filtering images
        self.filter_images_command_publisher = Publisher('/command/filter_images', Bool, queue_size=1)

        # Create a publisher to publish the command to use force feedback
        self.use_force_feedback_command_publisher = Publisher('/command/use_force_feedback', Bool,
                                                              queue_size=1)

        # Create a publisher to publish the command to start and stop the robot motion
        self.stop_robot_motion_command_publisher = Publisher('/command/stop_motion', Bool, queue_size=1)

        # Create a publisher to publish the command to have the user select the crop coordinates
        self.select_crop_coordinates_command_publisher = Publisher('/command/select_crop_coordinates', Bool,
                                                                   queue_size=1)

        # Create a publisher to publish the command to have the user select the initial mask for the grabcut filter
        self.generate_grabcut_filter_mask_command_publisher = Publisher('/command/generate_grabcut_mask', Bool,
                                                                        queue_size=1)

        # Create a publisher to publish the command to have the user generate the threshold parameters of the
        # thresholding filter
        self.generate_threshold_filter_parameters_command_publisher = Publisher(
            '/command/generate_threshold_parameters', Bool, queue_size=1)

        # Create a publisher to publish the desired force for the robot to exert
        self.goal_force_publisher = Publisher('/goal/force', WrenchStamped, queue_size=1)

        # -------------------------------------------
        # Create GUI components of the User Interface
        # -------------------------------------------

        # Define parameters used in the logic of the GUI
        self.currently_filtering = False
        self.currently_force_controlling = False
        self.currently_robot_moving = False

        # Set the title of the window
        self.winfo_toplevel().title("Experiment Control Window")

        # Define the frame in which all objects will be created
        window_content_frame = Frame(parent)

        # Define buttons that will have their values changed
        self.image_filtering_button = ttk.Button(window_content_frame, text=START_IMAGE_FILTER,
                                                 command=self.image_filtering_button_callback)
        self.force_control_button = ttk.Button(window_content_frame, text=START_FORCE_CONTROL,
                                               command=self.force_control_button_callback)
        self.robot_movement_button = ttk.Button(window_content_frame, text=START_ROBOT_MOVEMENT,
                                                command=self.robot_movement_button_callback, state="disable")
        self.current_force_label = ttk.Label(window_content_frame, text="NaN")
        self.desired_force_entry = ttk.Entry(window_content_frame)
        self.status_label = ttk.Label(window_content_frame, text=EMPTY_STATUS)

        # Define a list of widgets to create
        # List items are defined as (<widget object>, <y axis padding>, <column position>, <column span>)
        widgets = [
            # Define widgets for the initialization section
            (ttk.Label(window_content_frame, text="Filter Initialization Options"), 10,
             LEFT_COLUMN, FULL_WIDTH),
            (ttk.Button(window_content_frame, text="Generate Image Crop Coordinates.",
                        command=self.select_crop_coordinates_button_callback), 2,
             LEFT_COLUMN, FULL_WIDTH),
            (ttk.Button(window_content_frame, text="Generate GrabCut Filter Mask",
                        command=self.generate_grabcut_filter_mask_button_callback), 2,
             LEFT_COLUMN, FULL_WIDTH),
            (ttk.Button(window_content_frame, text="Generate Threshold Filter Parameters",
                        command=self.generate_threshold_filter_parameters_button_callback), 2,
             LEFT_COLUMN, FULL_WIDTH),

            # Define spacer widget to space out the two sections
            (ttk.Label(window_content_frame, text=""), 1,
             LEFT_COLUMN, FULL_WIDTH),

            # Define widgets for the ROS control portion
            (ttk.Label(window_content_frame, text="ROS Control Options"), 10, LEFT_COLUMN, FULL_WIDTH),
            (ttk.Label(window_content_frame, text="Current Force:"), 2, LEFT_COLUMN, SINGLE_COLUMN),
            (self.current_force_label, 2, MIDDLE_COLUMN, SINGLE_COLUMN),
            (self.image_filtering_button, 2, RIGHT_COLUMN, SINGLE_COLUMN),
            (ttk.Label(window_content_frame, text="Desired Force:"), 2, LEFT_COLUMN, SINGLE_COLUMN),
            (self.desired_force_entry, 2, MIDDLE_COLUMN, SINGLE_COLUMN),
            (self.force_control_button, 2, RIGHT_COLUMN, SINGLE_COLUMN),
            (ttk.Button(window_content_frame, text="Send Desired Force to Robot",
                        command=self.send_desired_force), 2, LEFT_COLUMN, TWO_COLUMN),
            (self.robot_movement_button, 2, RIGHT_COLUMN, SINGLE_COLUMN),
            (self.status_label, 10, LEFT_COLUMN, FULL_WIDTH),
        ]

        # Add the parent frame as the only grid object in the window
        window_content_frame.grid(column=0, row=0)

        i = 0
        # Add each widget to the grid
        for widget in widgets:
            widget[0].grid(column=widget[2], columnspan=widget[3], row=i, pady=widget[1])
            if widget[3] == FULL_WIDTH or widget[2] == RIGHT_COLUMN:
                i = i + 1

        # Allow the window to be resized
        parent.columnconfigure(0, weight=1)
        parent.rowconfigure(0, weight=1)

    ############################################################################
    # Define GUI button callbacks
    ############################################################################

    def select_crop_coordinates_button_callback(self):
        self.select_crop_coordinates_command_publisher.publish(Bool(True))
        self.update_status("Selecting crop coordinates")

    def generate_grabcut_filter_mask_button_callback(self):
        self.generate_grabcut_filter_mask_command_publisher.publish(Bool(True))
        self.update_status("Selecting grabcut filter mask")

    def generate_threshold_filter_parameters_button_callback(self):
        self.generate_threshold_filter_parameters_command_publisher.publish(Bool(True))
        self.update_status("Selecting threshold filter parameters")

    def image_filtering_button_callback(self):
        """
        Toggles if the ultrasound images will be filtered, based on the user input.
        """
        # Get the current text of the button
        button_text = self.image_filtering_button[WIDGET_TEXT]

        # If the button currently says "Start"
        if button_text == START_IMAGE_FILTER:

            # Publish the command to start filtering images
            self.filter_images_command_publisher.publish(Bool(True))

            # Set it to say "Stop"
            new_button_text = STOP_IMAGE_FILTER

            # Set the state to be that the image is currently being filtered
            self.currently_filtering = True

            # Update the status label
            self.update_status("Image filtering has started")

        # If the button currently says "Stop"
        else:

            # Publish the command to stop filtering images
            self.filter_images_command_publisher.publish(Bool(False))

            # Set the button to say "Stop"
            new_button_text = START_IMAGE_FILTER

            # Set the state to be that the image is not currently being filtered
            self.currently_filtering = False

            # Update the status label
            self.update_status("Image filtering has stopped")

        # Set the new text of the button
        self.image_filtering_button[WIDGET_TEXT] = new_button_text

        # Check to see if the state of the robot command button should be toggled
        self.robot_toggle_state_change()

    def force_control_button_callback(self):
        """
        Toggles if the robot will use force feedback, based on the user input.
        """

        # Get the current text of the button
        button_text = self.force_control_button[WIDGET_TEXT]

        # If the button currently says "Start"
        if button_text == START_FORCE_CONTROL:

            # Publish the command to start using force feedback
            self.use_force_feedback_command_publisher.publish(Bool(True))

            # Set it to say "Stop"
            new_button_text = STOP_FORCE_CONTROL

            # Set the state to be that the robot is currently using force feedback
            self.currently_force_controlling = True

            # Update the status label
            self.update_status("Force control has started")

        # If the button currently says "Stop"
        else:

            # Publish the command to stop using force feedback
            self.use_force_feedback_command_publisher.publish(Bool(False))

            # Set it to say "Start"
            new_button_text = START_FORCE_CONTROL

            # Set the state to be that the robot is not currently using force_feedback
            self.currently_force_controlling = False

            # Update the status label
            self.update_status("Force control has stopped")

        # Set the new text of the button
        self.force_control_button[WIDGET_TEXT] = new_button_text

        # Check to see if the state of the robot command button should be toggled
        self.robot_toggle_state_change()

    def robot_movement_button_callback(self):
        """
        Toggles if the robot will move, based on user input.
        """

        # Get the current text of the button
        button_text = self.robot_movement_button[WIDGET_TEXT]

        # If the button currently says "Start"
        if button_text == START_ROBOT_MOVEMENT:

            # Set it to say "Stop"
            new_button_text = STOP_ROBOT_MOVEMENT

            # Set the state to be that the robot is currently moving
            self.currently_robot_moving = True

            # Update the status label
            self.update_status("Robot has started moving")

        # If the button currently says "Stop"
        else:

            # Set it to say "Start"
            new_button_text = START_ROBOT_MOVEMENT

            # Set the state to be that the robot is currently not moving
            self.currently_robot_moving = False

            # Update the status label
            self.update_status("Robot has stopped moving")

        # Set the new text of the button
        self.robot_movement_button[WIDGET_TEXT] = new_button_text

        # Toggle the state of the other buttons based on the state of the robot movement
        self.robot_movement_toggle_other_button_states()

    def robot_toggle_state_change(self):
        """
        Toggle the state of the robot movement button, based on the state of the image filtering and force feedback.
        """

        # If image filtering and force feedback are both on, allow the button to be clicked
        if self.currently_filtering and self.currently_force_controlling:
            self.robot_movement_button[WIDGET_STATE] = WIDGET_STATE_ENABLED

        # Otherwise, disable it
        else:
            self.robot_movement_button[WIDGET_STATE] = WIDGET_STATE_DISABLED

    def robot_movement_toggle_other_button_states(self):
        """
        Toggle the states of the image filtering and force control buttons based on the robot moving state.
        """

        # If the robot is currently moving, disable the image filtering and force control buttons
        if self.currently_robot_moving:
            new_state = WIDGET_STATE_DISABLED

        # If the robot is not currently moving, enable the image filtering and force control buttons
        else:
            new_state = WIDGET_STATE_ENABLED

        # Set the states of the buttons
        self.image_filtering_button[WIDGET_STATE] = new_state
        self.force_control_button['state'] = new_state

    def send_desired_force(self):
        new_wrench = WrenchStamped()
        try:
            new_wrench.wrench.force.z = float(self.desired_force_entry.get())
            self.goal_force_publisher.publish(new_wrench)
            self.update_status("New force sent")
        except ValueError:
            self.desired_force_entry.delete(0, END)
            self.update_status("Invalid force value given")
        pass

    #############################################################################
    # Define ROS callbacks
    #############################################################################
    def robot_sensed_force_callback(self, data: WrenchStamped):
        """
        A callback function to update the GUI with the current Z force felt by the robot.

        Parameters
        ----------
        data
            The WrenchStamped message sent by the robot containing the current force experienced by the robot.
        """
        self.current_force_label.config(text=str(data.wrench.force.z))

    def debug_status_messages_callback(self, data: String):
        self.update_status(data.data)

    #############################################################################
    # Define Helpers
    #############################################################################
    def update_status(self, message: str = None):
        """
        Defines a function to update the status label at the bottom of the window with a given message.

        Parameters
        ----------
        message
            The message to be displayed. A period will be added to the end of this message automatically.
            If no message is given, the empty status string will be shown.
        """
        new_message = EMPTY_STATUS
        if message is not None:
            new_message = EMPTY_STATUS + message + "."
        self.status_label.config(text=new_message)


if __name__ == "__main__":

    # Create a root window
    root = Tk()

    # Create the control application within the root window
    ExperimentControlApplication(root)

    print("Close the GUI window to terminate this node.")

    # Run until the window is closed.
    root.mainloop()
