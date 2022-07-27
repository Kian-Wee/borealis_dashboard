import os

def myping(host):
    response = os.popen("ping -c 1 " + host).read()
    response_split = response.split()
    idx = 0 
    for string in response_split:
        # found the index that that has min/avg/max/mdev
        if string == 'min/avg/max/mdev':
            # min_avg_max_mdev index 
            min_avg_max_mdev = response_split[idx + 2]
            # split them up\
            min_avg_max_mdev_split = min_avg_max_mdev.split('/')
            # the avg value
            print(min_avg_max_mdev_split[2])
        idx += 1
        
out = myping("192.168.1.64")