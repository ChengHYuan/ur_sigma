
import select,termios,tty,sys
from numpy import False_
import rospy
from sensor_msgs.msg import Joy
import signal
import sys

## 获取键盘按键信息
def listenKey():
    ## sys.stdin表示标准化输入
    ## termios.tcgetattr(fd)返回一个包含文件描述符fd的tty属性的列表
    property_list = termios.tcgetattr(sys.stdin)
    ## tty.setraw(fd, when=termios.TCSAFLUSH)将文件描述符fd的模式更改为raw。如果when被省略，则默认为termios.TCSAFLUSH，并传递给termios.tcsetattr()
    tty.setraw(sys.stdin.fileno())
    ## 第一个参数是需要监听可读的套接字, 第二个是需要监听可写的套接字, 第三个是需要监听异常的套接字, 第四个是时间限制设置
    ## 如果监听的套接字满足了可读可写条件, 那么所返回的can_read 或 can_write就会有值, 然后就可以利用这些返回值进行后续操作
    can_read, _, _ = select.select([sys.stdin], [], [], 0.1)
    if can_read:
        keyValue = sys.stdin.read(1)
    else:
        keyValue = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, property_list)
    return keyValue

def keyboard_interrupt(signal, frame):
    print("Keyboard Interrupt detected!")
    sys.exit(0)



def main():
    rospy.init_node('keyboard')
    # pub = rospy.Publisher('keys',String,queue_size = 1)
    pub = rospy.Publisher('/pedal/buttons',Joy,queue_size = 1)
    rate = rospy.Rate(100)     #设置发布频率
    msg=Joy()
    msg.buttons.append(0)
    
    signal.signal(signal.SIGINT, keyboard_interrupt)

    while True:
        try:
            keyValue=listenKey()
            if keyValue:
                if keyValue=='a':
                    # print("OK")
                    msg.buttons[0]=1
                else:
                    print(keyValue)
                    msg.buttons[0]=0
            pub.publish(msg)
            rate.sleep() 
        except rospy.ROSInterruptException:
            print("program end!")
            sys.exit(0)
    return


if __name__ == '__main__':
    
    main()
    



