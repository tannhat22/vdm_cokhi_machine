import os
import sqlite3


database_path = os.path.join(os.path.expanduser("~"),
                             'ros2_ws/src/vdm_cokhi_machine/vdm_cokhi_machine/database/machine.db')

conn = sqlite3.connect(database_path)
cur = conn.cursor()

# Tạo bảng database history cho máy được cài đặt nếu chưa có:
def create_table_machine_history_db(tableName):
    try:
        cur.execute('CREATE TABLE IF NOT EXISTS ' + tableName +
        ''' (ID INTEGER PRIMARY KEY AUTOINCREMENT NOT NULL,
                DATE TIMESTAMP NOT NULL,
                SHIFT TEXT NOT NULL,
                NOLOAD INTEGER NOT NULL,
                UNDERLOAD INTEGER NOT NULL,
                OFFTIME INTEGER NOT NULL);''')
    except Exception as e:
        print(Exception)
    return

def copy_data_to_new_table(old_table_name, new_table_name):
    try:
        # Lấy dữ liệu từ bảng cũ
        cur.execute(f'SELECT * FROM {old_table_name}')
        rows = cur.fetchall()

        # Insert dữ liệu vào bảng mới với cột SHIFT được thiết lập mặc định là "Day"
        for row in rows:
            cur.execute(f"INSERT INTO {new_table_name} (DATE, SHIFT, NOLOAD, UNDERLOAD, OFFTIME) VALUES (?, ?, ?, ?, ?)",
                        (row[1], "CN", row[2], row[3], row[4]))

        # Lưu thay đổi và commit
        conn.commit()
        print(f"Đã sao chép dữ liệu từ '{old_table_name}' sang '{new_table_name}'.")

        # Xóa bảng cũ
        cur.execute(f"DROP TABLE IF EXISTS {old_table_name}")
        conn.commit()
        print(f"Đã xóa bảng cũ '{old_table_name}'.")

        # Đổi tên bảng mới thành tên bảng cũ
        cur.execute(f"ALTER TABLE {new_table_name} RENAME TO {old_table_name}")
        conn.commit()
        print(f"Đã đổi tên bảng mới thành '{old_table_name}'.")
    except Exception as e:
        print(e)

# Sử dụng hàm để tạo bảng mới và sao chép dữ liệu từ bảng cũ

try:
    cur.execute("SELECT * from " + 'MACHINES')
    rows = cur.fetchall()
    if len(rows) > 0:
        for row in rows:
            create_table_machine_history_db(row[1]+'_mmtb')
            copy_data_to_new_table(row[1], row[1]+'_mmtb')

except Exception as e:
    print(Exception)
