#!/usr/bin/python
import xlrd
from box_class import *

def database(filename):
    given_bin_list = []
    box_list = []
    position = [0, 0, 0]
    workbook = xlrd.open_workbook(filename)
    bin_sheet = workbook.sheet_by_index(0)
    box_sheet = workbook.sheet_by_index(1)
    for i in range(1, bin_sheet.nrows):
        given_bin_list.append(box_class(bin_sheet.row_values(i)[0], [bin_sheet.row_values(i)[4], bin_sheet.row_values(i)[5], bin_sheet.row_values(i)[6]], [bin_sheet.row_values(i)[1], bin_sheet.row_values(i)[2], bin_sheet.row_values(i)[3]]))
    for i in range(1, box_sheet.nrows):
        box_list.append(box_class(box_sheet.row_values(i)[0], position, [box_sheet.row_values(i)[1], box_sheet.row_values(i)[2], box_sheet.row_values(i)[3]]))
    return given_bin_list, box_list
