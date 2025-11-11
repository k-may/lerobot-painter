import threading

import websocket
websocket.enableTrace(True)
connected_event = threading.Event()
def on_open(wsapp):
    wsapp.send("Hello")
    print("Connected")
    connected_event.set()

def on_message(ws, message):
    print(message)
    ws.send("Send a ping", websocket.ABNF.OPCODE_PING)

def on_pong(wsapp, message):
    print("Got a pong! No need to respond")

#get server ip (ipconfig | findstr "IPv4")
wsapp = websocket.WebSocketApp("ws://172.18.224.1:8765", on_open=on_open, on_message=on_message, on_pong=on_pong)

# blocks thread...
# wsapp.run_forever()

t = threading.Thread(target=wsapp.run_forever, daemon=True)
t.start()
connected_event.wait()