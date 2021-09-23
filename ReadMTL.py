import math
import sys
import numpy as np
from numpy import load, save, concatenate

## search the line contain SUN_ELEVATION and SUN_AZIMUTH in MTL.file
def search_multiple_strings_in_file(file_name, list_of_strings):
    """Get line from the file along with line numbers, which contains any string from the list"""
    line_number = 0
    list_of_results = []
    # Open the file in read only mode
    with open(file_name, 'r') as read_obj:
        # Read all lines in the file one by one
        for line in read_obj:
            line_number += 1
            # For each line, check if line contains any string from the list of strings
            for string_to_search in list_of_strings:
                if string_to_search in line:
                    # If any string is found in line, then append that line along with line number in list
                    list_of_results.append((string_to_search, line_number, line.rstrip()))
    # Return list of tuples containing matched string, line numbers and lines where string is found
    return list_of_results

# Get sun angles
def get_sun_angles():
    list_strings = ['SUN_ELEVATION', 'SUN_AZIMUTH']
    matched_lines = search_multiple_strings_in_file('E:\ACProgramML\Landsat8_Image\LC08_L1TP_127045_20190626_20190705_01_T1\LC08_L1TP_127045_20190626_20190705_01_T1_MTL.txt', list_strings)
    print('Total Matched lines : ', len(matched_lines))
    for elem in matched_lines:
        print('Word = ', elem[0], ' :: Line Number = ', elem[1], ' :: Line = ', elem[2])
        sun_azimuth_line = matched_lines[0]
        sun_azi = sun_azimuth_line[2]
        sun_azi_value = float(sun_azi[18:30])
        sun_elevation_line = matched_lines[1]
        sun_ele = sun_elevation_line[2]
        sun_ele_value = float(sun_ele[20:32])
        sun_zenith = 90 - sun_ele_value
    return sun_ele_value, sun_azi_value, sun_zenith
sun_ele_value, sun_azi_value, sun_zenith = get_sun_angles()


# Convert raster to xyz to see long-lattitude of the pixel
from raster2xyz import Raster2xyz

def runtest():
    input_raster = "E:\ACProgramML\Landsat8_Image\LC08_L1TP_127045_20190813_20190820_01_T1\LC08_L1TP_127045_20190813_20190820_01_T1_B1.TIF"
    out_csv = "E:/ACProgramML/ANNmodel/18Aug2019out_xyz_2.csv"

    rtxyz = Raster2xyz()
    rtxyz.translate(input_raster, out_csv)

if __name__ == "__main__":
    runtest()


## CALCULATE SENSOR ANGLES ( SENSOR ZENITH AND SENSOR AZIMUTH ANGLE)
def get_sensor_angles(pixlon, sat_lon, pixlat, sat_lat):
    """ Sensor zenith angle
    Defined as zeros at Nadir and positive in both direction away from nadir. This is not the scan angle.
    It accounts for the earth's curvature. Ranges from 0 to 90 degrees"""
    sensor_zenith = 0.0 # because it look at NADIR and ROLL ANGLE = -0.001 approximately ~ 0

    """ calculate sensor azimuth angle
    sensor azimuth angle represents the angle from the pixel to location on the earth where sensor zenith angle is 0 (the sub-sat point)
             if the sub-sat point is due south, the value is 180 or - 180
             if the sub-sat point is due north, the value is 0
             if the sub-sat point is due west, the value is -90
             if the sub-sat point is due east, the value is 90
              the range is from -180 to 180 degrees
    input: satlon, satlat, pixlon, pixlat
    pixlon, pixlat are the real scene corner points
    satlon, satlat are the points at black corner"""

    pi = 3.14159265
    DTOR = pi/180 # convert degree to radiance
    xlon = (pixlon - sat_lon)*DTOR
    xlat = (pixlat - sat_lat)*DTOR
    beta = math.acos(math.cos(xlat)*math.cos(xlon))
    sin_beta = math.sin(beta)
    if abs(sin_beta) > sys.float_info.epsilon:
        sen_azimuth = math.sin(xlon)/sin_beta
        sen_azi_min = min(1.0, max(-1.0, sen_azimuth))
        sen_azi_value = math.asin(sen_azi_min)/DTOR
    else:
        sen_azi_value = 0.0
    if xlat < 0.0:
        sen_azi_value = 180 - sen_azi_value
    if sen_azi_value < 0.0:
        sen_azi_value = sen_azi_value + 360.0

    return sensor_zenith, sen_azi_value

pixlon = 104.823944
pixlat =  22.716431
sat_lon = 104.399190
sat_lat =  22.718650

sensor_zenith, sen_azi_value = get_sensor_angles(pixlon, sat_lon, pixlat, sat_lat)


## CALCULATE RELATIVE AZIMUTH ANGLE FROM SUN AZIMUTH AND SENSOR AZIMUTH ANGLE
def relative_azimuth_angle(sun_azi_value, sen_azi_value):
    difference_value = abs(sun_azi_value - sen_azi_value)
    if difference_value > 180.0:
        rel_azi_value = 360.0 - difference_value
    else:
        rel_azi_value = 180.0 - difference_value
    return rel_azi_value
rel_azi_value = relative_azimuth_angle(sun_azi_value, sen_azi_value)



path_save = "E:\\ACProgramML\\InputforANN\\GeometryAngle\\"
save(path_save + 'sensor_zenith_angle26Jun19.npy', sensor_zenith)

path_save = "E:\\ACProgramML\\InputforANN\\GeometryAngle\\"
save(path_save + 'sun_zenith_angle26Jun19.npy', sun_zenith)

path_save = "E:\\ACProgramML\\InputforANN\\GeometryAngle\\"
save(path_save + 'relative_azimuth_angle26Jun19.npy', rel_azi_value)


