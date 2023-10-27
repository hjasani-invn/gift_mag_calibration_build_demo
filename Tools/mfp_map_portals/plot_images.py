from PIL import Image, ImageDraw
import numpy as np


def get_pix_cell_size(max_size):
    max_pixel_count = 4000
    approx_cell_size = max_pixel_count / max_size
    if approx_cell_size % 5 > 0:
        pix_cell_size = (approx_cell_size // 5 + 1) * 5
    else:
        pix_cell_size = approx_cell_size

    if pix_cell_size < 5:
        pix_cell_size = 5

    return int(pix_cell_size)


def get_cell_coordinates(i, j, pix_cell_size):
    # provides cell bottom left and cell top right pixel coordinates based on matrix indexes
    x_min = pix_cell_size * i + 1
    y_min = pix_cell_size * j + 1
    x_max = pix_cell_size * i + pix_cell_size - 1
    y_max = pix_cell_size * j + pix_cell_size - 1

    return x_min, y_min, x_max, y_max


def draw_picture(M, colormap, outpath, *args):

    x_cell_count = M.shape[0]
    y_cell_count = M.shape[1]
    max_size = max(x_cell_count, y_cell_count)

    pix_cell_size = get_pix_cell_size(max_size)

    x_size_pix = x_cell_count * pix_cell_size
    y_size_pix = y_cell_count * pix_cell_size

    background_color = (colormap[0][0], colormap[0][1], colormap[0][2])
    image = Image.new(mode='RGBA', size=(x_size_pix, y_size_pix), color=background_color)
    draw = ImageDraw.Draw(image)

    # adding a grid of black lines to the image
    # each square "contains" only one line for each dimension
    # it should look better with single pixel lines

    y_start = 0
    y_end = image.height
    step_size = pix_cell_size

    for x in range(0, image.width, step_size):
        line = ((x, y_start), (x, y_end))
        draw.line(line, fill='black')

    x_start = 0
    x_end = image.width

    for y in range(0, image.height, step_size):
        line = ((x_start, y), (x_end, y))
        draw.line(line, fill='black')

    for i in range(0, x_cell_count):
        for j in range(0, y_cell_count):

            if M[i][j] > 0:
                x_min, y_min, x_max, y_max = get_cell_coordinates(i, j, pix_cell_size)
                c_map_value = int(M[i][j])
                color = (colormap[int(c_map_value)][0], colormap[int(c_map_value)][1], colormap[int(c_map_value)][2])
                draw.rectangle(((x_min, y_min), (x_max, y_max)), fill=color)

    # marking of cs cells (optional arguement availability_cs_table)
    if len(args) > 0:
        availability_cs_table = args[0]

        for i in range(0, x_cell_count):
            for j in range(0, y_cell_count):
                index_sum = i * y_cell_count + j
                value = availability_cs_table.get(index_sum)
                if value is not None:
                    x_min, y_min, x_max, y_max = get_cell_coordinates(i, j, pix_cell_size)

                    line_width = 2
                    if pix_cell_size < 20:
                        line_width = 1

                    line = ((x_min, y_min), (x_max, y_max))
                    draw.line(line, fill='purple', width=line_width)
                    line = ((x_min, y_max), (x_max, y_min))
                    draw.line(line, fill='orange', width=line_width)

    del draw
    image = image.rotate(90, expand=True)
    image.save(outpath)


def draw_empty_grid(M, outpath):
    max_size = max(M.shape[0], M.shape[1])

    pix_cell_size = get_pix_cell_size(max_size)

    x_size_pix = M.shape[0] * pix_cell_size
    y_size_pix = M.shape[1] * pix_cell_size

    image = Image.new(mode='RGBA', size=(x_size_pix, y_size_pix), color='white')
    draw = ImageDraw.Draw(image)

    y_start = 0
    y_end = image.height
    step_size = pix_cell_size

    for x in range(0, image.width, step_size):
        line = ((x, y_start), (x, y_end))
        draw.line(line, fill='black')

    x_start = 0
    x_end = image.width

    for y in range(0, image.height, step_size):
        line = ((x_start, y), (x_end, y))
        draw.line(line, fill='black')

    del draw
    image = image.rotate(90, expand=True)
    image.save(outpath)


def write_stats_for_picture(M, colormap, outpath):
    # this function saves a file with percentage of gray, red, yellow and green squares on the image

    colors_count = colormap.shape[0]
    color_keys = []
    color_name_values = []
    for i in range(1, colors_count):
        color_keys.append(i)
        color = colormap[i]
        color_name = ''
        if np.array_equal(color, [255, 0, 0]):
            color_name = 'red'
        elif np.array_equal(color, [255, 255, 0]):
            color_name = 'yellow'
        elif np.array_equal(color, [0, 255, 0]):
            color_name = 'green'
        else:
            color_name = 'gray'

        color_name_values.append(color_name)

    color_dict = dict(zip(color_keys, color_name_values))
    color_cells_list = []
    total_colored_cells = 0
    for ck in color_keys:
        condition = np.fabs(M - ck) < 0.01
        current_color_cells_count = np.count_nonzero(condition)
        color_cells_list.append([ck, current_color_cells_count])
        total_colored_cells += current_color_cells_count

    if total_colored_cells > 0:
        with open(outpath, 'w') as outf:
            for element in color_cells_list:
                line = ''
                color_name = color_dict[element[0]]
                percentage = element[1]/total_colored_cells

                line += color_name + ' : ' + str(percentage) + '\n'

                outf.write(line)


