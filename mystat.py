# patients = {
#     N: [x, y, color_num]
#     color_num: 1-red, 2-yellow, 3-green
#     1: [0, 2, 0],
#     2: [1, 3, 0],
#     3: [1, 1, 0],
#     4: [4, 1, 0],
#     5: [4, 3, 0],
#     6: [3, 0, 0],
#     7: [2, 2, 0],
#     8: [2, 1, 0],
#     9: [3, 2, 0],
#     10: [5, 0, 0]
# }

def put_to_stat(N, x, y, color_num):
    with open('t.txt', 'w') as f:
        contents = f.read()
        patients = ast.literal_eval(contents)
        patients[N][2] = color_num
        print(patients, file=f)
