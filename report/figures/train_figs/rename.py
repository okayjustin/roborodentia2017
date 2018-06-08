import os
path = './all_transitions/'

for filename in os.listdir(path):
    prefix, num = filename[:-4].split('_')
    if ((int(num) - 1) % 200 == 0):
        num = num.zfill(5)
        new_filename = "a" + prefix + "_" + num + ".pdf"
        os.rename(os.path.join(path, filename), os.path.join(path, new_filename))