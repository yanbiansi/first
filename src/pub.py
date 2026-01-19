
import zmq, time

ctx = zmq.Context()
pub = ctx.socket(zmq.PUB)

pub.setsockopt(zmq.SNDHWM, 1) # 0无限队列 默认值1000，存1000条
    # 1当前没有任何 SUB 连接send 立即失败 / 丢弃.无论 IMMEDIATE 是 0 还是 1，PUB 都不会为“未来的 SUB”保存历史消息
pub.setsockopt(zmq.LINGER, 0) # 默认为-1，ZeroMQ 会 无限期等待直到队列里的消息“理论上发送完”
    # 程序 退出卡死，Ctrl+C 没反应，LINGER = 0  →  立即丢弃所有未发消息
pub.setsockopt(zmq.IMMEDIATE, 1) # 默认为0，发布者会排队消息，即使没有连接的订阅者当有订阅者连接时，会收到队列中的历史消息可能导致消息堆积在内存中
    # 设置为1，当前没有任何 SUB 连接，消息会被 直接丢弃 

pub.bind("tcp://*:5555")

i = 0
while True:
    msg = f"State {i}"
    print("PUB send:", msg)
    pub.send_string(msg)
    i += 1
    time.sleep(1)
