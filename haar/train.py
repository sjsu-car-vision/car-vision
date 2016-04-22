"""
This script calls:
1. opencv_creasesamples
2. opencv_traincascade
"""

import os
import subprocess
import sys


def main():
    """
    Main
    :return:
    """

    """
    ~ Initialization
    """
    workspace_dirpath = os.path.join(os.getcwd(), "workspace")
    positive_dirpath = os.path.join(workspace_dirpath, "positive")
    negative_dirpath = os.path.join(workspace_dirpath, "negative")

    background_path = os.path.join(negative_dirpath, "negative.txt")

    number_of_positive_sample = 100
    number_of_negative_sample = 100

    opencv_createsamples_filepath = "/usr/local/bin/opencv_createsamples"
    opencv_traincascade_filepath = "/usr/local/bin/opencv_traincascade"

    """
    1. Create sample
    """
    output_vec_filepath = os.path.join(workspace_dirpath, "output.vec")

    # Call subprocess
    #
    createsamples_cmd = [opencv_createsamples_filepath,
                         "-img", positive_dirpath,
                         "-vec", output_vec_filepath,
                         "-num", number_of_positive_sample,
                         "-bg", background_path]
    error = subprocess.call(createsamples_cmd)
    print error

    """
    2. Train cascade
    """
    output_cascade_dirpath = workspace_dirpath
    number_of_stages = 3

    # Call subprocess
    #
    traincascade_cmd = [opencv_traincascade_filepath,
                        "-data", output_cascade_dirpath,
                        "-vec", output_vec_filepath,
                        "-bg", background_path,
                        "-numPos", number_of_positive_sample,
                        "-numNeg", number_of_negative_sample,
                        "-numStages", number_of_stages]
    error = subprocess.call(traincascade_cmd)
    print error

    sys.exit(error)


if __name__ == "__main__":
    main()
