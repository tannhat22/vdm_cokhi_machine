import datetime
import subprocess

def set_system_time(year, month, day, hour, minute, second):
    # Xác định định dạng thời gian
    new_time = datetime.datetime(year, month, day, hour, minute, second)
    # Chuyển định dạng thời gian thành chuỗi
    time_str = new_time.strftime('%Y-%m-%d %H:%M:%S')
    # Sử dụng lệnh date để cài đặt thời gian hệ thống
    subprocess.run(['sudo', 'date', '-s', time_str])
    print('Đã cài đặt thời gian hệ thống thành công.')

# Gọi hàm set_system_time để cài đặt thời gian
set_system_time(2024, 2, 23, 10, 30, 0)