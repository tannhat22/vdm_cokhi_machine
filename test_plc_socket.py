from collections import Counter

def array_to_dict(input_array):
    # Sử dụng Counter để đếm số lần xuất hiện của từng phần tử
    count_dict = Counter(input_array)
    
    # Chuyển Counter thành một từ điển
    result_dict = dict(count_dict)
    
    return result_dict

# Ví dụ sử dụng
input_array = ['GS', 'BD', 'GR', 'GS', 'GS', 'GR', 'LN']
result_dictionary = array_to_dict(input_array)
a = {'GS': [5,]}
print(result_dictionary)
print(len(result_dictionary))

result = {}
for i in input_array:
    if i in result:
        result[i][0] += 1
    else:
        result[i] = [1]


