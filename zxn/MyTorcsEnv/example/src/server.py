import socket
import random
from struct import *
import traceback
import collections
import numpy as np

class preprocess(object):

    def function(self, pos_info, time, stuck):
        tmp = 23
        #tmp_l = pos_info[26]
        l = pos_info[0:19] + pos_info[23:25]  #sensor, wheel, pos_info
        l = list(l)
        l = l + [pos_info[tmp + 6], pos_info[tmp + 7], pos_info[tmp + 8], pos_info[50]] #speedX,Y,Z,rpm,23+4+4 = 31
        #r = l[21]*np.cos(l[19]) - np.abs(l[21]*np.sin(l[19])) - l[21]*np.abs(l[20])
        r = l[21]*np.cos(l[19])
        for i in range(19):
            l[i] = l[i]/200
        l[19] = l[19]/3.1416   #angle
        l[20] = l[20]/width   #tomiddle
        for i in range(21, 24):
            l[i] = l[i] * 3.6/300

        l[24] = l[24]/10000

        if (time % 20 == 0):
            for i in range(0, 24):
                print(l[i], end = ' ')
            print(l[24])
        ##l = pos_info[0:tmp]
        '''for i in range(4):
            if pos_info[tmp + 4*i] == 1:
                l = l + (1.0, 0.0, 0.0)
            if pos_info[tmp + 4*i] == 2:
                l = l + (0.0, 1.0, 0.0)
            if pos_info[tmp + 4*i] == 3:
                l = l + (0.0, 0.0, 1.0)
            l = l + pos_info[tmp + 4*i + 1: tmp + 4*i + 4]
        l = l + pos_info[25] + pos_info[26]
        l = list(l)
        #normal
        #angle and tostart
        l[0] = l[0]/3.1415926
        #disttomid/left/right
        l[1] = l[1]/width
        l[2] = l[2]/width
        l[3] = l[3]/width
        #fuel and weight
        l[4] = l[4]/100.0
        l[5] = l[5]/1500.0
        #speed
        l[6] = l[6]/300.0
        l[7] = l[7]/300.0
        l[8] = l[8]/300.0
        #kfiction and length
        l[13] = l[13]/10
        l[14] = l[14]/600
        l[13 + 6] = l[13 + 6]/10
        l[14 + 6] = l[14 + 6]/1000
        l[13 + 6*2] = l[13 + 6*2]/10
        l[14 + 6*2] = l[14 + 6*2]/1000
        l[13 + 6*3] = l[13 + 6*3]/10
        l[14 + 6*3] = l[14 + 6*3]/1000

        print('state------>%.3f %.3f %.3f %.3f %.3f %.3f %.3f' %(np.cos(l[0]*3.1415926), np.sin(l[0]*3.1415926), l[14], l[15]*1000, l[16], l[17], l[20]*1000))
        r = l[6]*np.cos(l[0]*3.1415926)*300 - np.abs(l[6]*np.sin(l[0]*3.1415926)*300)
        '''
        if l[20] < -1 + 0.3:
            r = -10
            stuck = 1
        if l[20] > 1 - 0.3:
            r = -10
            stuck = 1
        #if time > TIMELIMIT:
        #    r = -10
        #    stuck = 1
        if np.cos(l[19]*3.1416) < 0:
            r = -10
            stuck = 1 
        #if time > 300:
        #    if l[6] < 5.0/300:
        #        r = -1
        #        stuck = 1
        l = tuple(l)
        return r, l, stuck

def main():
    ip_port = ('127.0.0.1', 9999)
    s = socket.socket()
    s.bind(ip_port)
    s.listen()
    while(True):
        conn, addr = s.accept()
        epsilon -= 1.0 / explore
        while(True):
            try:
                recv_data = conn.recv(1024)
                if not recv_data:
                    break
                pos_info = unpack('51f', recv_data)
                
                r, info, stuck = pre.function(pos_info, time, stuck)

                answer = [0,0,0,0]

                conn.send(pack('4f', answer[0], answer[1], answer[2], 1.0))

if __name__ == '__main__':
    main()
