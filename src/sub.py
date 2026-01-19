
import zmq, time

ctx = zmq.Context()
sub = ctx.socket(zmq.SUB)

sub.setsockopt(zmq.RCVHWM, 1)  # 默认值是1000，0无限队列
sub.setsockopt(zmq.LINGER, 0) # # 默认为-1，ZeroMQ 会 无限等待直到 SUB 内部队列“被认为已经处理完”
    # 程序 退出卡死，Ctrl+C 没反应，LINGER = 0  →  直接丢弃队列
# sub.setsockopt(zmq.RCVTIMEO, 2000) # 默认值-1，sub.recv()没消息会一直阻塞，ctrl+c也无法退出
    # 设置为10，则最多阻塞10ms，没有消息抛 zmq.Again，需设置try except处理异常代码

sub.connect("tcp://127.0.0.1:5555")
sub.setsockopt_string(zmq.SUBSCRIBE, "State")

# while True:
#     try:
#         msg = sub.recv_string()
#         print("SUB recv:", msg)
#     except zmq.Again:
#         print("SUB no data")
while True:
    msg = sub.recv_string()
    print("SUB recv:", msg)
   
        
