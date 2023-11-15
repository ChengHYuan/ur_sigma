import signal
import sys
import rospy
import select
from sensor_msgs.msg import Joy
import tty
import termios

def main():
    rospy.init_node('keyboard')
    # pub = rospy.Publisher('keys',String,queue_size = 1)
    pub = rospy.Publisher('/pedal/buttons',Joy,queue_size = 1)
    rate = rospy.Rate(100)     #设置发布频率
    msg=Joy()
    msg.buttons.append(0)
    count=0
    
    
    def keyboard_interrupt(signal, frame):
        print("Keyboard Interrupt detected!")
        sys.exit(0)

    signal.signal(signal.SIGINT, keyboard_interrupt)

    print("Press Ctrl+C to exit...")

    # 将终端设置为非规范模式
    orig_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin)

    try:
        while True:
            # 检查标准输入是否有可读数据
            rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
            if rlist:
                # 读取单个字符并处理
                input_data = sys.stdin.read(1)
            else:
                input_data=''
                # count=0
            # print("You typed:", input_data)
            if input_data=='a':
                # msg.buttons[0]=1
                count=count+1
                if count>=2:
                    # print("OK")
                    msg.buttons[0]=1
                else:
                    msg.buttons[0]=0
            else:
                # print(input_data)
                msg.buttons[0]=0
                count=0
            # if msg.buttons[0]==0:
            #     print("---------------empty-----------")
            pub.publish(msg)
            rate.sleep()
            
            # 程序中断处理
            
    except KeyboardInterrupt:
        print("Keyboard Interrupt detected!")
    # 恢复终端设置
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, orig_settings)
        print("------恢复终端设置-----")

if __name__ == '__main__':
    main()
    
    