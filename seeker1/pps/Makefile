obj-m += pps_s.o

# 'all' 目标现在只构建内核模块
all:
    make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

# 'clean' 目标也只清理内核模块
clean:
    make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean

# 我们删除了 'gcc pps_s_main.c ...' 这一行，
# 因为 seeker1_ros2/CMakeLists.txt 现在负责它。