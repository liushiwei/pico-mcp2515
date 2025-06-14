import os
import shutil

ret  = ["A:\\", "B:\\", "C:\\", "D:\\", "E:\\", "F:\\", "G:\\", "H:\\", "I:\\",
                  "J:\\", "K:\\", "L:\\", "M:\\", "N:\\", "O:\\", "P:\\", "Q:\\", "R:\\",
                  "S:\\", "T:\\", "U:\\", "V:\\", "W:\\", "X:\\", "Y:\\", "Z:\\"]
print(ret)


# 判断是否是U盘
def get_u_disk(drives):
    u_disk = []
    for item in drives:
        try:
            
            # 转化成GB
            if os.path.exists(item+"INFO_UF2.TXT"):
                u_disk.append(item)
                print('append')
            else:
                print('not exists')
        except:
            break
    return u_disk


lst = get_u_disk(ret)
print(lst)
# 复制文件夹中的内容到U盘里
usb_path = lst[0] 
pc_path = r"build\pico-mcp2515.uf2"

def copy_file():
    shutil.copy(pc_path, usb_path)
copy_file()