#sudo hcitool lescan
#./ble_task.py BC:6A:29:AC:1D:59 --all #2541
#./ble_task.py 68:C9:0B:06:BA:8B --all
#python -tt ./ble_task.py 78:A5:04:86:DD:24 --all
#./ble_task.py 7C:66:9D:80:25:54 --all
#./ble_task.py 78:A5:04:86:DB:69 --all #kv
#python ./btle.py BC:6A:29:AC:1D:59 #2541
#python ./btle.py 78:A5:04:86:DB:69  #kv
#python ./btle.py 78:A5:04:86:DD:24  #qn
python ./bluepy/bluepy/sensortag.py BC:6A:29:AC:1D:59 --all #qn
