
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
        while True:
            if self.connected == False:
                print("Socket listening")
                self.tcp_socket.listen()
                self.conn, _ = self.tcp_socket.accept()
                self.connected = True
            else:
                time.sleep(1)

    def recv_data(self, ongui_recv_queue: queue.Queue):
        data = b""
        while True:
            if self.connected == True:
                try:
                    buffer = self.conn.recv(1024)
                    if len(buffer) > 0:
                        if (buffer == b"\n"):
                            if ongui_recv_queue.full():
                                ongui_recv_queue.get_nowait()
                            ongui_recv_queue.put_nowait(data)
                            data = b""
                            break
                        else:
                            data += buffer
                    else:
                        pass
                except BrokenPipeError:
                    self.tcp_socket.close()
                    self.connected = False
                    break
                except Exception as e:
                    print(e)
            else:
                pass
            time.sleep(0.1)
    
    def send_data(self, data):
        self.tcp_socket.send(data)
        self.tcp_socket.send(b"\n")
    
    def __del__(self):
        self.tcp_socket.close()
        

        
        
        


        
