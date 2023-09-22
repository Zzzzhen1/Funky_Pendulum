import csv, os
import pandas as pd
from datetime import datetime, timedelta

# Hopefully there is less than one auto_freq_scan per day
# To facilitate the manual search for the wrong fitted pdf file
# This code will save your input file_name to the wrong_fit.csv file for future re-processing
if __name__ == "__main__":
    # Input the folder that contains the wrong fitted pdf file (in the format of
    # "auto_scan_fit_pdf")
    parent_folder_path = input("Enter the path of the parent folder: ")
    parent_name = os.path.basename(parent_folder_path)
    
    # Grand parent folder is named as "init-dd-mm-long-one"
    grand_parent_folder_path = os.path.dirname(parent_folder_path)
    grand_parent_name = os.path.basename(grand_parent_folder_path)
    
    # Then tries to find the reference parameters file
    csv_folder_list = []
    for file in os.listdir(grand_parent_folder_path):
        if file.startswith("reference_parameters"):
            reference_file_path = os.path.join(grand_parent_folder_path, file)
            ref_data = pd.read_csv(reference_file_path)
        if file.endswith("-csv") \
        and not file.endswith("-fft-csv")\
        and not file.endswith("amp-csv"):
            csv_folder_list.append(os.path.join(grand_parent_folder_path, file))
    
    # Creating the wrong_fit.csv file if it does not exist
    wrong_fit_path = os.path.join(parent_folder_path, "wrong_fit.csv")
    if not os.path.exists(wrong_fit_path):
        with open(wrong_fit_path, "w", newline = "") as f:
            writer = csv.writer(f)
            writer.writerow(["date",
                             "pdf_absolute_path",
                             "csv_absolute_path",
                             ])
            f.close()
    # "Renormalised_Data\init-20-09-short-one\auto_scan_fit_pdf\auto_freq_scan-00-01-38.pdf"
    # Starts the while loop to read the file_name from input
    try:
        # Enter the file name of the wrong fitted pdf file
        while True: 
            # Type in the absolute path
            # in the format of "os.getcwd()\grandparents\parents\auto_freq_scan-HH-MM-SS.pdf"
            file_path = input("Enter the wrongly fitted file path: ")
            file_name = os.path.basename(file_path)
            # Rectifying the file_path
            if not os.path.exists(file_path):
                file_path = os.path.join(os.getcwd(), file_path)
            
            pdf_time_list = file_name.split("-")[1:]
            # Date list is non-trivial due to the way of export, need to find the date from 
            # reference csv file
            for row in ref_data["start_time"].to_numpy():
                date = datetime(hour = row.split(" ")[0].split(":")[0],
                                minute = row.split(" ")[0].split(":")[1],
                                seconds = row.split(" ")[0].split(":")[2],)
                if(timedelta(seconds = 2) > abs(date - datetime(hour = int(pdf_time_list[0]),
                                                              minute = int(pdf_time_list[1]),
                                                              second = int(pdf_time_list[2]),))):
                    pdf_date_list = row.split(" ")[1].split("-")
                    break
                
            # pdf_datetime = datetime(int(pdf_date_list[2]),
            #                         int(pdf_date_list[1]),
            #                         int(pdf_date_list[0]),
            #                         int(pdf_time_list[0]),
            #                         int(pdf_time_list[1]),
            #                         int(pdf_time_list[2]),
            #                         )
            # print("The datetime of the pdf file is {}".format(pdf_datetime))
            
            # flag_found = False 
            # # There might be some very horrible cases like the pdf and csv are created in 
            # # two different days
            
            # for csv_folder in csv_folder_list:
            #     for file in os.listdir(csv_folder):
            #         if file.endswith(".csv"):
            #             csv_time_list = file.split("-")[1:]
            #             csv_date_list = os.path.basename(csv_folder).split("-")[:-2]
            #             csv_datetime = datetime(year = 0,
            #                                     month = int(csv_date_list[1]),
            #                                     day = int(csv_date_list[0]),
            #                                     hour = int(csv_time_list[0]),
            #                                     minute = int(csv_time_list[1]),
            #                                     second = int(csv_time_list[2]),
            #                                     )
            #             if (csv_datetime - pdf_datetime) < timedelta(seconds = 2):
            #                 flag_found = True
            #                 csv_file_path = os.path.join(csv_folder, file)
            #                 break
            #         else:
            #             continue
            #     if flag_found:
            #         break
            # if flag_found:
            #     print("Found the csv file: {}".format(csv_file_path))
            #     with open(wrong_fit_path, "a", newline = "") as f:
            #         writer = csv.writer(f)
            #         writer.writerow([csv_datetime.strftime("%d-%m-%Y"),
            #                          file_path,
            #                          csv_file_path,
            #                          ])
            #         f.close()
            # else:
            #     print("Cannot find the csv file")
            # TODO: 
            # 1. Try to append the csv file path to the wrong_fit.csv file
            # 2. Need to know about the time and the date of both the csv and pdf file
    except KeyboardInterrupt:
        print("End of search for this auto_freq_scan_folder")
        
        