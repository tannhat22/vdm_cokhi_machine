def sort_rows_by_plc_address(rows, plc_model):
    try:
        # Sắp xếp các hàng dựa trên giá trị tăng dần của row[4]
        sorted_rows = sorted(rows, key=lambda x: x[3])

        result = {
            'quantity': len(sorted_rows),
            'idMachines': [],
            'machineName': [],
            'machineType': [],
            'PLC_address': [],
        }
        for row in sorted_rows:
                result['idMachines'].append(row[0])
                result['machineName'].append(row[1])
                result['machineType'].append(row[2])
                result['PLC_address'].append(row[3])
        return result
    except Exception as e:
        print("Error:", e)
        return False

# Danh sách các hàng mẫu (giả lập)
rows = [
    (1, 'Machine1', 'TypeA', 123),
    (2, 'Machine2', 'TypeB', 456),
    (3, 'Machine3', 'TypeA', 789),
    (4, 'Machine4', 'TypeC', 1)
]

# Thử sử dụng hàm
plc_model = '789'  # Giả sử giá trị PLC_model cần tìm là '789'
result = sort_rows_by_plc_address(rows, plc_model)
print(result)