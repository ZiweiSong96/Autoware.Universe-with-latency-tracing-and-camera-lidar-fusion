import os
import csv

def process_txt_files(folder_path):

# If want put csv files in seperate folder, use the follow sentences:
    # csv_folder_path = os.path.join(folder_path, 'csv_files')
    # os.makedirs(csv_folder_path, exist_ok=True)

    for root, dirs, files in os.walk(folder_path):
        for file in files:
            if file.endswith('.txt'):
                file_path = os.path.join(root,file)
                csv_file_path = os.path.splitext(file_path)[0]+'.csv'
 

                with open (file_path,'r') as txtfile, open(csv_file_path,'w',newline='') as csvfile:
                    writer = csv.writer(csvfile)
                    lines = txtfile.readlines()
                    for line in lines:
                        writer.writerow(line.split())
print('Down!')
folder_path = '/home/mlabszw/autoware_with_caret/my_evaluate'

process_txt_files(
  folder_path  
)