import datetime

# Tạo đối tượng datetime cho self.real_time_plc với giờ, phút và giây mặc định là 0
real_time_plc = datetime.datetime(year=2024, month=4, day=4, hour=7, minute=0, second=0)

# Tạo các đối tượng datetime cho giờ bắt đầu và kết thúc của khoảng thời gian
gio_bat_dau = datetime.time(hour=6, minute=0, second=0)
gio_ket_thuc = datetime.time(hour=17, minute=59, second=59)

# Kiểm tra xem real_time_plc có nằm trong khoảng từ 06:00:00-17:59:59 không
if gio_bat_dau <= real_time_plc.time() <= gio_ket_thuc:
    print("Giờ của real_time_plc nằm trong khoảng từ 06:00:00 đến 17:59:59")
else:
    print("Giờ của real_time_plc không nằm trong khoảng từ 06:00:00 đến 17:59:5")