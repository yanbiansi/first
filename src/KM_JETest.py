# conda环境：py3.10     pip install pyzmq
import zmq
import json
import time

def WaitForHandshake(pub, sub, timeout=10.0):
    end_time = time.time() + timeout
    last_send = 0

    print("Waiting for handshake...")

    while time.time() < end_time:
        if time.time() - last_send > 0.1:
            pub.send_string("Hello {}")
            last_send = time.time()

        msg = GetRawMessage(sub)
        if msg is None:
            time.sleep(0.01)
            continue

        topic, payload = msg
        if topic == "HelloAck":
            print("Handshake success ✔ (Ack received)")
            pub.send_string("Ready {}")
            return True

    print("Handshake timeout ✘")
    return False

def GetRawMessage(sub): # 读服务端发布的消息
    try:
        msg = sub.recv_string() # 阻塞接收，超时会返回zmq.Again异常
    except zmq.Again:
        # print("recv_string no msg")
        return None
    
    pos = msg.find(' ') # 找第一个空格，作为协议分隔符，找不到返回-1
    if pos == -1:
        return None # 如果没有空格，说明消息不符合协议 → 返回 None

    topic = msg[:pos]
    payload = msg[pos + 1:]

    try:
        data = json.loads(payload)
    except json.JSONDecodeError:
        return None

    return topic, data

def init_socket():
    context = zmq.Context()

    publisher = context.socket(zmq.PUB)
    subscriber = context.socket(zmq.SUB)

    publisher.setsockopt(zmq.SNDHWM, 1) # 发送队列长度
    publisher.setsockopt(zmq.CONFLATE, 1)
    publisher.setsockopt(zmq.IMMEDIATE, 1) # 没有已建立连接时，send 直接丢弃
    publisher.setsockopt(zmq.LINGER, 0); # socket 关闭时，是否等待队列消息发完,0关闭即丢
    publisher.bind("tcp://*:8001")

    subscriber.setsockopt_string(zmq.SUBSCRIBE, "") # 设置订阅前缀过滤，"" = 全订阅
    subscriber.setsockopt(zmq.RCVHWM, 1) # 队列长度为1
    subscriber.setsockopt(zmq.CONFLATE, 1) # 保留最新的消息
    subscriber.setsockopt(zmq.RCVTIMEO, 15) # recv 最多阻塞 x ms，使用定时阻塞接收，需要使用try语句
    subscriber.setsockopt(zmq.LINGER, 0)    # socket 关闭时，是否等待队列消息发完,0关闭即丢
    subscriber.connect("tcp://192.168.0.99:8000")

    return publisher, subscriber

def GetRobotState(sub):
    """
    从 SUB socket 中读取一条 State 消息
    返回解析后的 JSON(dict)，如果不是 State 或未收到则返回 None
    """
    msg = GetRawMessage(sub)
    if msg is None:
        return None

    topic, data = msg
    if topic != "State":
        return None

    return data

def RobotPower(pub, value):
    pub.send_string("Switch " + json.dumps({"Switch": value}))
    time.sleep(0.1) 


def moveA(pub, sub, joint_name, speed_name):
    data = {
        "joint_name": joint_name,
        "speed_name": speed_name
    }
    # 转换为 JSON 字符串
    message = "moveA " + json.dumps(data)
    pub.send_string(message) 
    time.sleep(0.05)
    try:
        while True:
            state = GetRobotState(sub)
            move_state = state["Robot0"]["MoveState"]
            print("MoveState:", move_state)

            # === 关键判断 ===
            if move_state == 0:
                print("运动结束")
                break

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\n循环被用户中断！")


def moveL(pub, sub, pose_name, speed_name):
    data = {
        "pose_name": pose_name,
        "speed_name": speed_name
    }
    # 转换为 JSON 字符串
    message = "moveL " + json.dumps(data)
    pub.send_string(message) 
    time.sleep(0.05)
    try:
        while True:
            state = GetRobotState(sub)
            move_state = state["Robot0"]["MoveState"]
            print("MoveState:", move_state)

            # === 关键判断 ===
            if move_state == 0:
                print("运动结束")
                break

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\n循环被用户中断！")

def main():

    publisher, subscriber = init_socket()
    time.sleep(1)

    if not WaitForHandshake(publisher, subscriber):
        print("通信未建立，禁止发送控制指令")
        return
    time.sleep(1)

    state = GetRobotState(subscriber)
    print(state)
    print(state["Robot0"]["MoveState"])
    print(state["Robot0"]["PowerState"])

    RobotPower(publisher, True)
    time.sleep(1)
    print("上使能")
    moveA(publisher, subscriber, "R0_L_HOME", "R0_speed20")
    moveA(publisher, subscriber, "R0_P1", "R0_speed20")
    moveL(publisher, subscriber, "R0_D3", "R0_speed20")
    moveA(publisher, subscriber, "R0_L_HOME", "R0_speed20")
    time.sleep(1)

    RobotPower(publisher, False)
    time.sleep(1)
    print("下使能")

    publisher.send_string("stop {}") # 停止服务端发送状态，避免下次连接影响握手
    time.sleep(0.01)

if __name__ == "__main__":
    main()

