# {N: [x, y, color_num, qr]}
# d = {
#     1: [0, 2.72, 0, 0],
#     2: [0.72, 3.94, 0, 0],
#     3: [0.72, 1.5, 0, 0],
#     4: [2.88, 1.5, 0, 0],
#     5: [2.88, 3.94, 0, 0],
#     6: [2.16, 0.28, 0, 0],
#     7: [1.44, 2.72, 0, 0],
#     8: [1.44, 1.5, 0, 0],
#     9: [2.16, 2.72, 0, 0],
#     10: [3.6, 0.28, 0, 0]
# }

def put_to_stat(N, x, y, color_num):
    with open('t.txt', 'w') as f:
        contents = f.read()
        patients = ast.literal_eval(contents)
        patients[N][2] = color_num
        print(patients, file=f)

def give_me_COVID_data():
    with open('t.txt', 'w') as f:
        contents = f.read()
        patients = ast.literal_eval(contents)
        data = {}
        i = 1
        while i <= 10:
            if patients[i][2] == 2 or patients[i][2] == 1:
                data[i] = patients[i]
            i += 1
        return data
