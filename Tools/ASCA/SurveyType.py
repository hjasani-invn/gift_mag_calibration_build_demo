import plot_images
import numpy as np
import os


def get_survey_type_colormap():

    a = np.array([[140, 212, 252],  # light blue
                  [100, 100, 100],  # gray
                  [255, 128, 0],  # orange
                  [0, 0, 160],  # dark blue
                  [128, 0, 255]])  # purple

    return a


def one_cell_convert_survey_type(survey_type):
    res = 0
    if survey_type == -1:
        res = 0
    elif survey_type == 0:
        res = 1
    elif survey_type == 1:
        res = 2
    elif survey_type == 2:
        res = 3
    elif survey_type == 5:
        res = 4

    return res


def convert_survey_type(survey_type_matrix):
    m_conv = np.zeros([survey_type_matrix.shape[0], survey_type_matrix.shape[1]])

    for i in range(0, survey_type_matrix.shape[0]):
        for j in range(0, survey_type_matrix.shape[1]):
            m_conv[i, j] = one_cell_convert_survey_type(survey_type_matrix[i, j])

    return m_conv


def plot_survey_type(survey_type_matrix, out_folder, output_plot_file_name, real_floor):
    output_plot_file_name = output_plot_file_name[:-4] + '_' + str(real_floor) + output_plot_file_name[-4:]

    survey_type_cmap = get_survey_type_colormap()
    survey_type_matrix_conv = convert_survey_type(survey_type_matrix.transpose())
    outpath = os.path.join(out_folder, output_plot_file_name)

    plot_images.draw_picture(survey_type_matrix_conv, survey_type_cmap, outpath)
