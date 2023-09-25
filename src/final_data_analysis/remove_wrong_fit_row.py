import os
import pandas as pd

# Read the wrong_fit.csv file

if __name__ == "__main__":
    wrong_fit_csv_path = input("Enter the path of wrong_fit.csv file: ")
    wrong_fit_data = pd.read_csv(wrong_fit_csv_path)
    parent_name = os.path.basename(wrong_fit_csv_path).split(".")[0]
    
    grand_parent_path = os.path.dirname(wrong_fit_csv_path)

    for file in os.listdir(grand_parent_path):
        if file.startswith("scan_data"):
            scan_data_path = os.path.join(grand_parent_path, file)
            break
    # read the scan_data.csv file
    scan_data = pd.read_csv(scan_data_path)
    
    # Delete the wrong fit row from scan_data.csv
    for index, csv_file_path in enumerate(wrong_fit_data["csv_absolute_path"].unique()):
        for index1, filename in enumerate(scan_data["file_name"]):
            if os.path.basename(csv_file_path) == filename:
                scan_data = scan_data.drop(scan_data.index[index1])
                break
    # Save the scan_data_remove.csv file
    scan_data.to_csv(os.path.join(grand_parent_path, "scan_data_remove.csv"), index = False)