import shutil
import os


data_dir = r'D:\ctz\code\data\final_预处理步骤\indoor_data_mini'
target_data_path = r'D:\ctz\code\PC_I\PCD2BIM_paper\door\data'
for data_name in os.listdir(data_dir):
    temp0 = data_name.split('-')[0] + '-' + data_name.split('-')[1]
    if temp0 == 'Room-1':
        room_data_path = os.path.join(data_dir, data_name) + '\\Annotations'
        for i in os.listdir(room_data_path):
            if i.split("_")[0] == 'door':
                old_data_path = os.path.join(room_data_path, i)
                new_data_path = os.path.join(target_data_path, data_name+'_'+i)
                print(old_data_path)
                print(new_data_path)
                shutil.copyfile(old_data_path, new_data_path)