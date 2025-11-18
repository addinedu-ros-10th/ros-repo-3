
import socket
import time
import queue

class TCPSocket():
    def __init__(self, _ip: int, _port: int) -> None:
        self.ip = _ip
        self.port = _port
        self.connected = False
        self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    
    def init_socket(self):
        while True:
            try:
                self.tcp_socket.bind((self.ip, self.port))
                break
            except Exception as e:
                print(e)
                continue

    def connect_socket(self):
        print("Socket listening")
        while True:
            if self.connected == False:
                try:
                    self.tcp_socket.settimeout(1)
                    self.tcp_socket.listen()
                    self.conn, _addr = self.tcp_socket.accept()
                    print(f"Socket Connected!: {_addr}")
                    self.connected = True
                    self.tcp_socket.settimeout(None)
                except Exception as e:
                    pass
            else:
                time.sleep(1)

    def recv_data(self, queue: queue.Queue):
        data = b""
        while True:
            if self.connected == True:
                try:
                    buffer = self.conn.recv(1024)
                    if len(buffer) > 0:
                        data += buffer
                        if (data[-4:] == b"DONE"):
                            if queue.full():
                                queue.get_nowait()
                            queue.put_nowait(data[:-4])
                            data = b""
                            buffer = b""
                        else:
                            pass
                    else:
                        pass
                except BrokenPipeError:
                    print("Pipe Broken")
                    self.connected = False
                except Exception as e:
                    print(e)
                    self.connected = False
            else:
                pass
            time.sleep(0.1)
    
    def send_data(self, data):
        self.conn.send(data)
        self.conn.send(b"DONE")
    
    def __del__(self):
        self.tcp_socket.close()
        

        
        
        


        
