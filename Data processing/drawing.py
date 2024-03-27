import csv
import matplotlib.pyplot as plt
import argparse

def plot_line_chart(csv_file_path,save_path):
    x_values =[]
    y_values =[]

    with open(csv_file_path, 'r') as csvfile:
        reader = csv.reader(csvfile)
        next(reader)
        for row in reader:
            x_values.append(float(row[0]))
            y_values.append(float(row[1]))
    
    plt.plot(x_values, y_values)
    plt.xlabel('Time stamp')
    plt.ylabel('Value')
    plt.savefig(save_path)
    plt.close()

parser = argparse.ArgumentParser(description='Drawing figures')
parser.add_argument('csv_file_path',type=str,help='csv file path')
parser.add_argument('save_path',type=str, help='Image save path')

args = parser.parse_args()

# csv_file_path = '/home/mlabszw/autoware_with_caret/my_evaluate/perception/lidar_centerpoint/input_pointcloud_num.csv'
# save_path = '/home/mlabszw/autoware_with_caret/my_evaluate/image/lidar_centerpoint_inputpc_num.png'
# plot_line_chart(csv_file_path,save_path)

# csv_file_path = '/home/mlabszw/autoware_with_caret/my_evaluate/perception/lidar_centerpoint/latency.csv'
# save_path = '/home/mlabszw/autoware_with_caret/my_evaluate/image/lidar_centerpoint_latency.png'
# plot_line_chart(csv_file_path,save_path)
plot_line_chart(args.csv_file_path, args.save_path)