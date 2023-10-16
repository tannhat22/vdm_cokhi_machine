# import datetime
# import sqlite3

# # make a database connection and cursor object
# connection = sqlite3.connect('StudentAssignment.db',
# 							detect_types=sqlite3.PARSE_DECLTYPES |
# 							sqlite3.PARSE_COLNAMES)
# cursor = connection.cursor()

# cursor.execute("CREATE TABLE IF NOT EXISTS id1 ( _id INTEGER PRIMARY KEY AUTOINCREMENT,value VARCHAR(32))")

# cursor.execute("INSERT INTO test (value) VALUES ('Value#1')")
# cursor.execute("INSERT INTO test (value) VALUES ('Value#2')")
# cursor.execute("INSERT INTO test (value) VALUES ('Value#3')")

# cursor.execute("DELETE FROM test WHERE _id=2")

# cursor.execute("CREATE TABLE test2 AS SELECT * FROM test")

# cursor.execute("DELETE FROM test")
# cursor.execute("DELETE FROM sqlite_sequence WHERE name='test'")

# cursor.execute("INSERT INTO test (value) SELECT value FROM test2")

# cursor.execute("DROP TABLE test2")
# connection.commit()

# # select query to retrieve data
# cursor.execute("SELECT value from test WHERE _id = 1")
# # cursor.execute("SELECT rowid, * FROM ASSIGNMENT WHERE rowid = 7")
# # cursor.execute("DELETE FROM ASSIGNMENT WHERE _id = 4")
# # connection.commit()
# rows = cursor.fetchone()
# print(rows)
# for row in rows:
#     print(row)

# cursor.close()
# connection.close()


# import datetime
# import sqlite3

# # get the current datetime and store it in a variable
# currentDateTime = datetime.datetime.now()

# # make the database connection with detect_types 
# connection = sqlite3.connect('StudentAssignment.db',
# 							detect_types=sqlite3.PARSE_DECLTYPES |
# 							sqlite3.PARSE_COLNAMES)
# cursor = connection.cursor()

# # create table in database
# createTable = '''CREATE TABLE IF NOT EXISTS ASSIGNMENT (
# 	StudentId INTEGER,
# 	StudentName VARCHAR(100),
# 	SubmissionDate TIMESTAMP);'''
# cursor.execute(createTable)

# # create query to insert the data
# insertQuery = """INSERT INTO ASSIGNMENT
# 	VALUES (?, ?, ?);"""

# # insert the data into table
# cursor.execute(insertQuery, (1, "Virat Kohli", 
# 							currentDateTime))
# cursor.execute(insertQuery, (2, "Rohit Pathak",
# 							currentDateTime))
# print("Data Inserted Successfully !")

# # commit the changes,
# # close the cursor and database connection 
# connection.commit()
# cursor.execute("SELECT * FROM ASSIGNMENT WHERE StudentId = 1")
# rows = cursor.fetchall()
# print(rows)
# for row in rows:
#     print(type(row[2].date().strftime("%m/%d/%Y")))
#     print(row[2].date().strftime("%m/%d/%Y"))

# cursor.close()
# connection.close()


import datetime

a = [1,2,3,4,5,6,7,8,9,10]
b = {
    'a': 1
}

b['a'] += 1
c = '0x00'
d = c.split(' ')

e = datetime.datetime(2023,12,29,23,59,59)
print(e)
print(d)
print(b)
print(a[0:])