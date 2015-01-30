xterm -T Master -e sudo ./pman-master-tester pman-tester.cfg &
sleep 1
xterm -T Slave1 -e ./pman-slave-tester slave1 &
sleep 1
xterm -T Slave2 -e ./pman-slave-tester slave2

sudo killall pman-master-tester pman-slave-tester
sudo ipcrm -M 0x777777 -S 0x777777 -M 0x9011 -S 0x9013

