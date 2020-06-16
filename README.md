# M480BSP_FreeRTOS
 M480BSP_FreeRTOS

update @ 2020/06/16

1. Create 3 timer task (10ms , 50ms , 100ms ) with GPIO toggle (PH0 , PH1 , PH2)

2. Create task A to rising a flag per 500 ms 
	
	- with task B by polling , if the flag true then create a task C (sub task)
	
	- with task C by polling , set flag to false , and delete task C (sub task)
	
3. below is GPIO toggle capture , 10 ms , 50 ms , 100 ms , to monitor the 3 timer task

![image](https://github.com/released/M480BSP_FreeRTOS/blob/master/10ms.jpg)
	
![image](https://github.com/released/M480BSP_FreeRTOS/blob/master/50ms.jpg)

![image](https://github.com/released/M480BSP_FreeRTOS/blob/master/100ms.jpg)

4. output log to monitor task (rising flag , create task , delete sub task)

![image](https://github.com/released/M480BSP_FreeRTOS/blob/master/log.jpg)