import rclpy
from rclpy.clock import Clock, ROSClock
from datetime import datetime
import time
def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('time_between_events')

    clock = ROSClock()

    print("ROSCLOCK(): "+str((ROSClock().now().to_msg().sec)+((ROSClock().now().to_msg().nanosec)/1e9)))
    print("ROSCLOCK:", ((clock.now().to_msg().sec)+((clock.now().to_msg().nanosec)/1e9)))
    #node.set_parameter(rclpy.parameter.Parameter('use_sim_time', rclpy.ParameterType.BOOL, True))

    start_time = clock.now().to_msg()
    #print(start_time)
    # Do something here
    time.sleep(1.5)
    end_time = clock.now().to_msg()
    difference_in_sec = (end_time.sec + end_time.nanosec / 1e9) - (start_time.sec + start_time.nanosec / 1e9)
    print("Difference in Sec: " + str(difference_in_sec))
    
    start_time = datetime.fromtimestamp(start_time.sec + start_time.nanosec / 1e9)

    end_time = datetime.fromtimestamp(end_time.sec + end_time.nanosec / 1e9)

    time_diff = end_time - start_time
    #print(f'Time difference between two events: {time_diff}')

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()