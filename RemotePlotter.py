import socket

import threading
import time

import matplotlib.pyplot as plt

globalVals = {
    "x" : "0.0",
    "y1" : "0.0",
    "y2" : "0.0",
    "xlim_min" : "-1.0",
    "xlim_max" : "1.0",
    "ylim_min" : "-1.0",
    "ylim_max" : "1.0",
    "reset" : "0"
}

fig = plt.figure()
ax = plt.axes()

def Retrieve_Params(from_data : str, to_params : dict):
    if (not from_data):
        print('No data')
        return False
    for el in from_data.split(";;"):
        keyVal = el.split(":=")
        try:
            to_params[keyVal[0]] = keyVal[1]
        except:
            pass
    return True
    
def Server():

    HOST = ''
    PORT = 4004

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as soc:
        soc.bind((HOST, PORT))
        soc.listen()
        alive = True
        while alive:
            logmsg = ""
            try:
                conn, addr = soc.accept()

                x = [0.0, 0.0]
                y1 = [0.0, 0.0]
                y2 = [0.0, 0.0]

                def RecieveAndSend() -> bool:
                    
                    k = 0
                    data = bytearray(conn.recv(1024))
                    try:
                        while 1:
                            if k > 15:
                                return False
                            if data[k] == 10:   #'\n'
                                k += 1
                                break
                            k += 1
                    except:
                        return False

                    payload = int(data[:k])
                    data = data[k:]

                    while (len(data) < payload):
                        data.extend(conn.recv(1024))

                    if (payload > 0 and not Retrieve_Params(data.decode('utf-8'), globalVals)):
                        return False

                    if (globalVals["reset"] != "0"):
                        ax.cla()
                        x[0] = float(globalVals["x"])
                        y1[0] = float(globalVals["y1"])
                        y2[0] = float(globalVals["y2"])
                    else:
                        x[1] = float(globalVals["x"])
                        y1[1] = float(globalVals["y1"])
                        y2[1] = float(globalVals["y2"])
                        ax.plot(x, y1, color = 'blue', linewidth = 1.0)
                        ax.plot(x, y2, color = 'red', linewidth = 1.0)
                        x[0] = x[1]
                        y1[0] = y1[1]
                        y2[0] = y2[1]
        
                    ax.set(xlim = (float(globalVals["xlim_min"]), float(globalVals["xlim_max"])), \
                           ylim = (float(globalVals["ylim_min"]), float(globalVals["ylim_max"])))

                    fig.canvas.draw_idle()

                    conn.sendall(bytes(globalVals["x"], 'utf-8'))

                    return True

                with conn:

                    if RecieveAndSend():
                        print('Connected by', addr)
                        conn.settimeout(10.0)

                        while True:
                            try:
                                if not RecieveAndSend():
                                    break
                            except:
                                logmsg = "Timed out or connection lost. "
                                break
                    else:
                        print("Initial recieve failed")
                        
            except ConnectionError:
                pass
            except:
                alive = False
                raise
            finally:
                print(logmsg + "Disconnected from " + str(addr))
                conn.close()

threading.Thread(target = Server).start()

plt.show()