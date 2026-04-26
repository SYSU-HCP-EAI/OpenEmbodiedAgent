VERSION=0-17-0 # 通过修改该版本号来安装不同版本的franky-control以适配不同版本的机械臂系统
# you can modify the version number to install different versions of franky-control to adapt to different versions of robot server
wget https://github.com/TimSchneider42/franky/releases/latest/download/libfranka_${VERSION}_wheels.zip
unzip libfranka_${VERSION}_wheels.zip
pip install numpy
pip install --no-index --find-links=./dist franky-control