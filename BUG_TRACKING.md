# std::__throw_length_error SIGABRT

Thread 11 "component_conta" received signal SIGABRT, Aborted.
[Switching to Thread 0x7f4e356836c0 (LWP 55070)]
__pthread_kill_implementation (no_tid=0, signo=6, threadid=<optimized out>) at ./nptl/pthread_kill.c:44
warning: 44     ./nptl/pthread_kill.c: No such file or directory
(gdb) bt
#0  __pthread_kill_implementation (no_tid=0, signo=6, threadid=<optimized out>) at ./nptl/pthread_kill.c:44
#1  __pthread_kill_internal (signo=6, threadid=<optimized out>) at ./nptl/pthread_kill.c:78
#2  __GI___pthread_kill (threadid=<optimized out>, signo=signo@entry=6) at ./nptl/pthread_kill.c:89
#3  0x00007f4f5926c27e in __GI_raise (sig=sig@entry=6) at ../sysdeps/posix/raise.c:26
#4  0x00007f4f5924f8ff in __GI_abort () at ./stdlib/abort.c:79
#5  0x00007f4f5950cff5 in ?? () from /lib/x86_64-linux-gnu/libstdc++.so.6
#6  0x00007f4f595220da in ?? () from /lib/x86_64-linux-gnu/libstdc++.so.6
#7  0x00007f4f5950ca55 in std::terminate() () from /lib/x86_64-linux-gnu/libstdc++.so.6
#8  0x00007f4f59522391 in __cxa_throw () from /lib/x86_64-linux-gnu/libstdc++.so.6
#9  0x00007f4f595102d2 in std::__throw_length_error(char const*) () from /lib/x86_64-linux-gnu/libstdc++.so.6
#10 0x00007f4f1013991d in ?? () from /lib/x86_64-linux-gnu/libpcl_common.so.1.14
#11 0x00007f4f101329fe in pcl::PCLBase[pcl::PointXYZINormal](pcl::PointXYZINormal)::initCompute() () from /lib/x86_64-linux-gnu/libpcl_common.so.1.14
#12 0x00007f4e8038166d in LidarProcessor::process(SyncPackage&) () from /home/firebot/firebot_dragon/install/fastlio2/lib/liblio_component.so
#13 0x00007f4e80323149 in LIONode::processOnce() () from /home/firebot/firebot_dragon/install/fastlio2/lib/liblio_component.so
#14 0x00007f4e803245bc in LIONode::processLoop() () from /home/firebot/firebot_dragon/install/fastlio2/lib/liblio_component.so
--Type <RET> for more, q to quit, c to continue without paging--
#15 0x00007f4f59553db4 in ?? () from /lib/x86_64-linux-gnu/libstdc++.so.6
#16 0x00007f4f592c3aa4 in start_thread (arg=<optimized out>) at ./nptl/pthread_create.c:447
#17 0x00007f4f59350c6c in clone3 () at ../sysdeps/unix/sysv/linux/x86_64/clone3.S:78
(gdb)
