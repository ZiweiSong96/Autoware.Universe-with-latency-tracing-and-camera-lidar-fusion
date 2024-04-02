import csv

with open('pointcloud_preprocessor/concatenate_combinePC_timestamp.csv','r') as file1:
    reader1 = csv.reader(file1)
    data1=list(reader1)

with open('perception/multi_object_tracker/latency.csv','r') as file2:
    reader2 = csv.reader(file2)
    data2=list(reader2)

matched_data=[]

for row2 in data2:
    flag2=row2[2]
    match_found=False

    for row1 in data1:
        flag1=row1[1]

        if flag1==flag2:
            diff = (float(row2[1])-float(row1[0]))*1000
            matched_data.append([flag2, row1[0], row2[1],diff])
            match_found=True
            break
    if not match_found:
        continue

with open('e2e_latency_data.csv','w',newline='') as outfile:
    writer = csv.writer(outfile)
    writer.writerow(['Time Stamp','Start Time','End Time','E2E Latency'])
    writer.writerows(matched_data)