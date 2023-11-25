import sqlite3
import datetime

database_path = '/home/tannhat/ros2_ws/src/vdm_cokhi_machine/vdm_cokhi_machine/database/machine.db'
conn = sqlite3.connect(database_path)
cur = conn.cursor()

cur.execute("SELECT * from MACHINES")
rows = cur.fetchall()
query = 'ALTER TABLE MACHINES ADD COLUMN TYPE TEXT NOT NULL DEFAULT GS'
print(query)
cur.execute(query)
# for row in rows:
#     print(row[1])
#     query = 'ALTER TABLE MACHINES ADD COLUMN TYPE TEXT NOT NULL DEFAULT GS'
#     print(query)
#     cur.execute(query)
#     # date = datetime.datetime.now()
#     # query = "INSERT INTO " + row[1] + " (DATE, NOLOAD, UNDERLOAD, OFFTIME) VALUES (?, ?, ?, ?)"
#     # print(query)
#     # cur.execute(query, (date, 1, 1, 1))
conn.commit()
conn.close()
